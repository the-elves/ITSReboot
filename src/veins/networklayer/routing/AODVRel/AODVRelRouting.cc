//
// Copyright (C) 2014 OpenSim Ltd.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "AODVRelRouting.h"
#include "IPv4Route.h"
#include "Ieee80211Frame_m.h"
#include "InterfaceTableAccess.h"
#include "IPSocket.h"
#include "IPv4ControlInfo.h"
#include "UDPControlInfo.h"
#include "ModuleAccess.h"
#include "NodeOperations.h"
#include "RoutingTableAccess.h"

//changes 1.1
#include "veins/base/modules/BaseMobility.h"
//#include "inet/physicallayer/contract/packetlevel/IMediumLimitCache.h"
#define DEBUG false


Define_Module(AODVRelRouting);


// changes 1.5
simsignal_t AODVRelRouting::statLinkFailure = registerSignal("statLinkFailure");

// changes 1.1
double AODVRelRouting::prevHopReliability(AODVRelControlPacket *packet){
    BaseMobility *mobility = check_and_cast<BaseMobility *>(host->getModuleByPath(".veinsmobility"));
    double routeReliability = 1.0;

    if(packet->getPacketType() == RREQ){
        EV_DEBUG << "###&&&$$$ Entering handling stage for rreq prevHop" << endl;
        AODVRelRREQ *rreq = check_and_cast<AODVRelRREQ *>(packet);
        Coord prevPos;
        prevPos.x = rreq->getXPosition();
        prevPos.y = rreq->getYPosition();
        Coord prevVelocity;
        prevVelocity.x = rreq->getXVelocity();
        prevVelocity.y = rreq->getYVelocity();
        routeReliability = computeStandardLinkReliability(prevPos, prevVelocity, mobility->getCurrentPosition(), mobility->getCurrentSpeed());
        EV_DEBUG << "###&&&$$$ prevNodeMmobility: " << prevPos.x << " " << prevPos.y << " " << prevVelocity.x << " " << prevVelocity.y << endl;
        EV_DEBUG << "###&&&$$$ currentNodeMobility: " << mobility->getCurrentPosition().x << " " << mobility->getCurrentPosition().y << " " << mobility->getCurrentSpeed().x << " " << mobility->getCurrentSpeed().y << endl;
    }
    else if(packet->getPacketType() == RREP){
        EV_DEBUG << "###&&&$$$ Entering handling stage for rrep prevHop" << endl;
        AODVRelRREP *rrep = check_and_cast<AODVRelRREP *>(packet);
        Coord prevPos;
        prevPos.x = rrep->getXPosition();
        prevPos.y = rrep->getYPosition();
        Coord prevVelocity;
        prevVelocity.x = rrep->getXVelocity();
        prevVelocity.y = rrep->getYVelocity();
        routeReliability = computeStandardLinkReliability(prevPos, prevVelocity, mobility->getCurrentPosition(), mobility->getCurrentSpeed());
        EV_DEBUG << "###&&&$$$ prevNodeMobility: " << prevPos.x << " " << prevPos.y << " " << prevVelocity.x << " " << prevVelocity.y << endl;
        EV_DEBUG << "###&&&$$$ currentNodeMobility: " << mobility->getCurrentPosition().x << " " << mobility->getCurrentPosition().y << " " << mobility->getCurrentSpeed().x << " " << mobility->getCurrentSpeed().y << endl;
    }
    EV_DEBUG << "###&&&$$$ previous hop reliability: " << routeReliability << endl;
    return routeReliability;
}

void AODVRelRouting::setAdditionalPacketArguments(AODVRelControlPacket *packet, std::string &state){
    // get mobility module.
    BaseMobility *mobility = check_and_cast<BaseMobility *>(host->getModuleByPath(".veinsmobility"));

    if(packet->getPacketType() == RREQ){
        AODVRelRREQ *rreq = check_and_cast<AODVRelRREQ *>(packet);

        if(state == "create"){
            // initially link reliability set to one.
            EV_DEBUG << "###&&&$$$ Entering create stage for rreq" << endl;
            rreq->setRouteReliability(1.0);
        }
        else if(state == "handle"){
            // multiply route reliability with link reliability and store it back.
            EV_DEBUG << "###&&&$$$ Entering handling stage for rreq" << endl;
            EV_DEBUG << "###&&&$$$ oldRouteReliability: " << rreq->getRouteReliability() << endl;
            Coord prevPos;
            prevPos.x = rreq->getXPosition();
            prevPos.y = rreq->getYPosition();
            Coord prevVelocity;
            prevVelocity.x = rreq->getXVelocity();
            prevVelocity.y = rreq->getYVelocity();
            rreq->setRouteReliability(rreq->getRouteReliability()*computeStandardLinkReliability(prevPos, prevVelocity, mobility->getCurrentPosition(), mobility->getCurrentSpeed()));
            EV_DEBUG << "###&&&$$$ prevNodeMobility: " << prevPos.x << " " << prevPos.y << " " << prevVelocity.x << " " << prevVelocity.y << endl;
            EV_DEBUG << "###&&&$$$ currentNodeMobility: " << mobility->getCurrentPosition().x << " " << mobility->getCurrentPosition().y << " " << mobility->getCurrentSpeed().x << " " << mobility->getCurrentSpeed().y << endl;
        }

        // important to update speed and velocity after updating reliability.
        rreq->setXPosition(mobility->getCurrentPosition().x);
        rreq->setYPosition(mobility->getCurrentPosition().y);

        // changes 1.2
        if(mobility->getCurrentPosition().x < 0 || mobility->getCurrentPosition().y < 0){
            throw cRuntimeError("Error while setting position argument in routing packet, value can't be negative.");
        }

        rreq->setXVelocity(mobility->getCurrentSpeed().x);
        rreq->setYVelocity(mobility->getCurrentSpeed().y);

        EV_DEBUG << "###&&&$$$ newRouteReliability: " << rreq->getRouteReliability() << endl;
    }
    else if(packet->getPacketType() == RREP){
        AODVRelRREP *rrep = check_and_cast<AODVRelRREP *>(packet);

        if(state == "create"){
            // initially link reliability set to one.
            EV_DEBUG << "###&&&$$$ Entering create stage for rrep" << endl;
            rrep->setRouteReliability(1.0);
            rrep->setReverseRouteReliability(1.0);
        }
        else if(state == "handle"){
            // multiply route reliability with link reliability and store it back.
            EV_DEBUG << "###&&&$$$ Entering handling stage for rrep" << endl;
            EV_DEBUG << "###&&&$$$ oldRouteReliability: " << rrep->getRouteReliability() << endl;
            EV_DEBUG << "###&&&$$$ oldReverseRouteReliability: " << rrep->getReverseRouteReliability() << endl;
            Coord prevPos;
            prevPos.x = rrep->getXPosition();
            prevPos.y = rrep->getYPosition();
            Coord prevVelocity;
            prevVelocity.x = rrep->getXVelocity();
            prevVelocity.y = rrep->getYVelocity();
            rrep->setReverseRouteReliability(rrep->getReverseRouteReliability()*computeStandardLinkReliability(prevPos, prevVelocity, mobility->getCurrentPosition(), mobility->getCurrentSpeed()));
            EV_DEBUG << "###&&&$$$ prevNodeMobility: " << prevPos.x << " " << prevPos.y << " " << prevVelocity.x << " " << prevVelocity.y << endl;
            EV_DEBUG << "###&&&$$$ currentNodeMobility: " << mobility->getCurrentPosition().x << " " << mobility->getCurrentPosition().y << " " << mobility->getCurrentSpeed().x << " " << mobility->getCurrentSpeed().y << endl;
        }

        // important to update speed and velocity after updating reliability.
        rrep->setXPosition(mobility->getCurrentPosition().x);
        rrep->setYPosition(mobility->getCurrentPosition().y);

        //changes 1.2
        if(mobility->getCurrentPosition().x < 0 || mobility->getCurrentPosition().y < 0){
            throw cRuntimeError("Error while setting position argument in routing packet, value can't be negative.");
        }

        rrep->setXVelocity(mobility->getCurrentSpeed().x);
        rrep->setYVelocity(mobility->getCurrentSpeed().y);

        EV_DEBUG << "###&&&$$$ newRouteReliability: " << rrep->getRouteReliability() << endl;
        EV_DEBUG << "###&&&$$$ newReverseRouteReliability: " << rrep->getReverseRouteReliability() << endl;
    }
}

double AODVRelRouting::computeStandardLinkReliability(Coord prevPos, Coord prevVelocity, Coord currPos, Coord currVelocity){
    // changes 1.2
    if(prevPos.x < 0 || prevPos.y < 0 || currPos.x < 0 || currPos.y < 0){

        throw cRuntimeError("Position variable negative in computeStandardLinkReliability");
    }

    Coord displacement = findRelative(prevPos, currPos);
    Coord relativeVelocity = findRelative(prevVelocity, currVelocity);

//    IMediumLimitCache *mediumCache = check_and_cast<IMediumLimitCache *>(host->getModuleByPath(".mediumLimitCache"));;
    double communicationRange = 800; // don't hard code this, get it from channel radio cache
//    mediumCache->getMaxCommunicationRange();
//    EV_DETAIL << "###&&&$$$ communication Range: " << communicationRange << endl;

    double timeInterval = findTimeInterval(displacement, relativeVelocity, communicationRange);
    // changes 1.2
    if(timeInterval < 0){
        //throw cRuntimeError("Error in computing timeInterval in routing for link reliability.");
    }

    double simulationTime = SIMTIME_DBL(simTime());

    double meanRelativeVelocity = findMeanRelativeVelocity(prevVelocity, currVelocity);
    double standardDeviation = findStandardDeviation(prevVelocity, currVelocity);

    double routeReliability = 1.0;

    if(!strcmp(this->routingType, "AODVReliability") || !strcmp(this->routingType, "AODVError")){

        // Common for all
        if(relativeVelocity.x != 0.0 || relativeVelocity.y != 0.0){
            double denominator = standardDeviation*sqrt(2);
            routeReliability = erf((((2*communicationRange)/(simulationTime)) - meanRelativeVelocity)/(denominator)) - erf((((2*communicationRange)/(simulationTime + timeInterval)) - meanRelativeVelocity)/(denominator));
        }

        // For propagation error based routing

        // changes 1.2 -added use of Error Model in aodvRel and other models
        if(!strcmp(this->routingType, "AODVError") || par("useErrorModel")){
            double magnitudeDisplacement = sqrt(displacement.x*displacement.x + displacement.y*displacement.y);
            if(magnitudeDisplacement > 200.0){
                routeReliability /= magnitudeDisplacement;
            }
        }
    }
    else{
        // route reliability is not a deciding factor for normal aodv, hence kept constant
    }
    EV_DEBUG << "###&&&$$$ Exiting computeStandardLinkReliability routeReliability: " << routeReliability << endl;

    // changes 1.2
    if(isnan(routeReliability)){
        throw cRuntimeError("Route reliability is not a number in routing, check time interval, communication range or mobility");
    }
    return routeReliability;
}

Coord AODVRelRouting::findRelative(Coord prev, Coord curr){
    // changes 1.2 -changed initialization.
    Coord relative;
    relative.x = prev.x;
    relative.y = prev.y;

    relative.x -= curr.x;
    relative.y -= curr.y;
    return relative;
}

double AODVRelRouting::findTimeInterval(Coord dis, Coord vel, double range){
    double timeInterval = 0.0;
    if(!strcmp(this->routingType, "AODVReliability") || !strcmp(this->routingType, "AODVError")){
        double p = vel.x*vel.x + vel.y*vel.y;
        double q = 2*(vel.x*dis.x + vel.y*dis.y);
        double r = dis.x*dis.x + dis.y*dis.y - range*range;
        double d = sqrt(q*q - 4*p*r);

        // changes 1.2 -corrected denominator
        timeInterval = (d-q)/(2*p);
    }
    return timeInterval;
}

double AODVRelRouting::findMeanRelativeVelocity(Coord prevVelocity, Coord currVelocity){
    double meanRelativeVelocity = 0.0;
    if(!strcmp(this->routingType, "AODVReliability") || !strcmp(this->routingType, "AODVError")){
        return abs(findStandardMeanVelocity(prevVelocity) - findStandardMeanVelocity(currVelocity));
    }
    return meanRelativeVelocity;
}

double AODVRelRouting::findStandardMeanVelocity(Coord velocity){
    double speed = sqrt(velocity.x*velocity.y + velocity.x*velocity.y);
    double mean = 0.0;
    // changes 1.2
    speed = (speed*18)/5; // speed in m/s convert to km/hr

    if(speed <= 40.0){
        mean = 30.00;
    }else if(speed <= 65.0){
        mean = 50.00;
    }else if(speed <= 90.0){
        mean = 70.0;
    }else if(speed <= 120.0){
        mean = 90.0;
    }else if(speed <= 145.0){
        mean = 110.0;
    }else if(speed <= 170.0){
        mean = 130.0;
    }else{
        mean = 150.0;
    }

    // changes 1.2
    mean = (mean*5)/18; // mean in km/hr convert to m/s
    return mean;
}

double AODVRelRouting::findStandardDeviation(Coord prevVelocity, Coord currVelocity){
    double standardDeviation = 0.0;
    if(!strcmp(this->routingType, "AODVReliability") || !strcmp(this->routingType, "AODVError")){
        standardDeviation = sqrt(findStandardDeviation_standard(prevVelocity)*findStandardDeviation_standard(prevVelocity) + findStandardDeviation_standard(currVelocity)*findStandardDeviation_standard(currVelocity));
    }
    return standardDeviation;
}

double AODVRelRouting::findStandardDeviation_standard(Coord velocity){
    double speed = sqrt(velocity.x*velocity.y + velocity.x*velocity.y);
    double standardDeviation = 0.0;
    // changes 1.2
    speed = (speed*18)/5; // speed in m/s convert to km/hr

    if(speed <= 40.0){
        standardDeviation = 30.00;
    }else if(speed <= 65.0){
        standardDeviation = 50.00;
    }else if(speed <= 90.0){
        standardDeviation = 70.0;
    }else if(speed <= 120.0){
        standardDeviation = 90.0;
    }else if(speed <= 145.0){
        standardDeviation = 110.0;
    }else if(speed <= 170.0){
        standardDeviation = 130.0;
    }else{
        standardDeviation = 150.0;
    }

    // changes 1.2
    standardDeviation = (standardDeviation*5)/18; // mean in km/hr convert to m/s
    return standardDeviation;
}


void AODVRelRouting::initialize(int stage)
{
    if (stage == 0) {
        lastBroadcastTime = SIMTIME_ZERO;
        rebootTime = SIMTIME_ZERO;
        rreqId = sequenceNum = 0;
        rreqCount = rerrCount = 0;
        host = getContainingNode(this);
        routingTable = RoutingTableAccess().get();
        interfaceTable = InterfaceTableAccess().get();
        networkProtocol = check_and_cast<INetfilter *>(getModuleByPath(par("networkProtocolModule")));

        AODVRelUDPPort = par("udpPort");
        askGratuitousRREP = par("askGratuitousRREP");
        useHelloMessages = par("useHelloMessages");
        activeRouteTimeout = par("activeRouteTimeout");
        helloInterval = par("helloInterval");
        allowedHelloLoss = par("allowedHelloLoss");
        netDiameter = par("netDiameter");
        nodeTraversalTime = par("nodeTraversalTime");
        rerrRatelimit = par("rerrRatelimit");
        rreqRetries = par("rreqRetries");
        rreqRatelimit = par("rreqRatelimit");
        timeoutBuffer = par("timeoutBuffer");
        ttlStart = par("ttlStart");
        ttlIncrement = par("ttlIncrement");
        ttlThreshold = par("ttlThreshold");
        localAddTTL = par("localAddTTL");
        jitterPar = &par("jitter");
        periodicJitter = &par("periodicJitter");

        myRouteTimeout = par("myRouteTimeout");
        deletePeriod = par("deletePeriod");
        blacklistTimeout = par("blacklistTimeout");
        netTraversalTime = par("netTraversalTime");
        nextHopWait = par("nextHopWait");
        pathDiscoveryTime = par("pathDiscoveryTime");

        // changes 1.1
        if(!DEBUG){
            routingType = par("routingType");
        }
        else{
            routingType = "AODV";
        }

    }
    else if (stage == 4) {
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(host->getSubmodule("status"));
        isOperational = !nodeStatus || nodeStatus->getState() == NodeStatus::UP;

        IPSocket socket(gate("ipOut"));
        socket.registerProtocol(IP_PROT_MANET);
        networkProtocol->registerHook(0, this);
        nb = NotificationBoardAccess().get();
        nb->subscribe(this, NF_LINK_BREAK);

        if (useHelloMessages) {
            helloMsgTimer = new cMessage("HelloMsgTimer");

            // RFC 5148:
            // Jitter SHOULD be applied by reducing this delay by a random amount, so that
            // the delay between consecutive transmissions of messages of the same type is
            // equal to (MESSAGE_INTERVAL - jitter), where jitter is the random value.
            if (isOperational)
                scheduleAt(simTime() + helloInterval - periodicJitter->doubleValue(), helloMsgTimer);
        }

        expungeTimer = new cMessage("ExpungeTimer");
        counterTimer = new cMessage("CounterTimer");
        rrepAckTimer = new cMessage("RREPACKTimer");
        blacklistTimer = new cMessage("BlackListTimer");

        if (isOperational)
            scheduleAt(simTime() + 1, counterTimer);
    }
}

void AODVRelRouting::handleMessage(cMessage *msg)
{
    if (!isOperational) {
        if (msg->isSelfMessage())
            throw cRuntimeError("Model error: self msg '%s' received when isOperational is false", msg->getName());

        EV_ERROR << "Application is turned off, dropping '" << msg->getName() << "' message\n";
        delete msg;
    }

    if (msg->isSelfMessage()) {
        if (dynamic_cast<WaitForRREP *>(msg))
            handleWaitForRREP((WaitForRREP *)msg);
        else if (msg == helloMsgTimer)
            sendHelloMessagesIfNeeded();
        else if (msg == expungeTimer)
            expungeRoutes();
        else if (msg == counterTimer) {
            rreqCount = rerrCount = 0;
            scheduleAt(simTime() + 1, counterTimer);
        }
        else if (msg == rrepAckTimer)
            handleRREPACKTimer();
        else if (msg == blacklistTimer)
            handleBlackListTimer();
        else
            throw cRuntimeError("Unknown self message");
    }
    else {
        // changes 1.2 -handled error thrown for casting
        if(!dynamic_cast<UDPPacket *>(msg)){
            return;
        }

        UDPPacket *udpPacket = dynamic_cast<UDPPacket *>(msg);
        AODVRelControlPacket *ctrlPacket = check_and_cast<AODVRelControlPacket *>(udpPacket->decapsulate());
        IPv4ControlInfo *udpProtocolCtrlInfo = dynamic_cast<IPv4ControlInfo *>(udpPacket->getControlInfo());
        ASSERT(udpProtocolCtrlInfo != NULL);
        IPv4Address sourceAddr = udpProtocolCtrlInfo->getSrcAddr();
        unsigned int arrivalPacketTTL = udpProtocolCtrlInfo->getTimeToLive();

        switch (ctrlPacket->getPacketType()) {
            case RREQ:
                handleRREQ(check_and_cast<AODVRelRREQ *>(ctrlPacket), sourceAddr, arrivalPacketTTL);
                break;

            case RREP:
                handleRREP(check_and_cast<AODVRelRREP *>(ctrlPacket), sourceAddr);
                break;

            case RERR:
                handleRERR(check_and_cast<AODVRelRERR *>(ctrlPacket), sourceAddr);
                break;

            case RREPACK:
                handleRREPACK(check_and_cast<AODVRelRREPACK *>(ctrlPacket), sourceAddr);
                break;

            default:
                throw cRuntimeError("AODVRel Control Packet arrived with undefined packet type: %d", ctrlPacket->getPacketType());
        }
        delete udpPacket;
    }
}

INetfilter::IHook::Result AODVRelRouting::ensureRouteForDatagram(IPv4Datagram *datagram)
{
    Enter_Method("datagramPreRoutingHook");
    const IPv4Address& destAddr = datagram->getDestAddress();
    const IPv4Address& sourceAddr = datagram->getSrcAddress();

    if (destAddr.isLimitedBroadcastAddress() || routingTable->isLocalAddress(destAddr) || destAddr.isMulticast())
        return ACCEPT;
    else {
        EV_INFO << "Finding route for source " << sourceAddr << " with destination " << destAddr << endl;
        IPv4Route *route = routingTable->findBestMatchingRoute(destAddr);
        AODVRelRouteData *routeData = route ? dynamic_cast<AODVRelRouteData *>(route->getProtocolData()) : NULL;
        bool isActive = routeData && routeData->isActive();
        if (isActive && !route->getGateway().isUnspecified()) {
            EV_INFO << "Active route found: " << route << endl;

            // Each time a route is used to forward a data packet, its Active Route
            // Lifetime field of the source, destination and the next hop on the
            // path to the destination is updated to be no less than the current
            // time plus ACTIVE_ROUTE_TIMEOUT.

            updateValidRouteLifeTime(destAddr, simTime() + activeRouteTimeout);
            updateValidRouteLifeTime(route->getGateway(), simTime() + activeRouteTimeout);

            return ACCEPT;
        }
        else if (sourceAddr.isUnspecified() || routingTable->isLocalAddress(sourceAddr)) {
            bool isInactive = routeData && !routeData->isActive();
            // A node disseminates a RREQ when it determines that it needs a route
            // to a destination and does not have one available.  This can happen if
            // the destination is previously unknown to the node, or if a previously
            // valid route to the destination expires or is marked as invalid.

            EV_INFO << (isInactive ? "Inactive" : "Missing") << " route for destination " << destAddr << endl;

            delayDatagram(datagram);

            if (!hasOngoingRouteDiscovery(destAddr)) {
                // When a new route to the same destination is required at a later time
                // (e.g., upon route loss), the TTL in the RREQ IP header is initially
                // set to the Hop Count plus TTL_INCREMENT.
                if (isInactive)
                    startRouteDiscovery(destAddr, route->getMetric() + ttlIncrement);
                else
                    startRouteDiscovery(destAddr);
            }
            else
                EV_DETAIL << "Route discovery is in progress, originator " << getSelfIPAddress() << " target " << destAddr << endl;

            return QUEUE;
        }
        else
            return ACCEPT;
    }
}

AODVRelRouting::AODVRelRouting()
{
    interfaceTable = NULL;
    host = NULL;
    routingTable = NULL;
    isOperational = false;
    networkProtocol = NULL;
    helloMsgTimer = NULL;
    expungeTimer = NULL;
    blacklistTimer = NULL;
    rrepAckTimer = NULL;
    jitterPar = NULL;
    nb = NULL;
}

bool AODVRelRouting::hasOngoingRouteDiscovery(const IPv4Address& target)
{
    return waitForRREPTimers.find(target) != waitForRREPTimers.end();
}

void AODVRelRouting::startRouteDiscovery(const IPv4Address& target, unsigned timeToLive)
{
    EV_INFO << "Starting route discovery with originator " << getSelfIPAddress() << " and destination " << target << endl;
    ASSERT(!hasOngoingRouteDiscovery(target));
    AODVRelRREQ *rreq = createRREQ(target);
    addressToRreqRetries[target] = 0;
    sendRREQ(rreq, IPv4Address::ALLONES_ADDRESS, timeToLive);
}

IPv4Address AODVRelRouting::getSelfIPAddress() const
{
    return routingTable->getRouterId();
}

void AODVRelRouting::delayDatagram(IPv4Datagram *datagram)
{
    EV_DETAIL << "Queuing datagram, source " << datagram->getSrcAddress() << ", destination " << datagram->getDestAddress() << endl;
    const IPv4Address& target = datagram->getDestAddress();
    targetAddressToDelayedPackets.insert(std::pair<IPv4Address, IPv4Datagram *>(target, datagram));
}

void AODVRelRouting::sendRREQ(AODVRelRREQ *rreq, const IPv4Address& destAddr, unsigned int timeToLive)
{
    // In an expanding ring search, the originating node initially uses a TTL =
    // TTL_START in the RREQ packet IP header and sets the timeout for
    // receiving a RREP to RING_TRAVERSAL_TIME milliseconds.
    // RING_TRAVERSAL_TIME is calculated as described in section 10.  The
    // TTL_VALUE used in calculating RING_TRAVERSAL_TIME is set equal to the
    // value of the TTL field in the IP header.  If the RREQ times out
    // without a corresponding RREP, the originator broadcasts the RREQ
    // again with the TTL incremented by TTL_INCREMENT.  This continues
    // until the TTL set in the RREQ reaches TTL_THRESHOLD, beyond which a
    // TTL = NET_DIAMETER is used for each attempt.

    if (rreqCount >= rreqRatelimit) {
        EV_WARN << "A node should not originate more than RREQ_RATELIMIT RREQ messages per second. Canceling sending RREQ" << endl;
        delete rreq;
        return;
    }

    std::map<IPv4Address, WaitForRREP *>::iterator rrepTimer = waitForRREPTimers.find(rreq->getDestAddr());
    WaitForRREP *rrepTimerMsg = NULL;
    if (rrepTimer != waitForRREPTimers.end()) {
        rrepTimerMsg = rrepTimer->second;
        unsigned int lastTTL = rrepTimerMsg->getLastTTL();
        rrepTimerMsg->setDestAddr(rreq->getDestAddr());

        // The Hop Count stored in an invalid routing table entry indicates the
        // last known hop count to that destination in the routing table.  When
        // a new route to the same destination is required at a later time
        // (e.g., upon route loss), the TTL in the RREQ IP header is initially
        // set to the Hop Count plus TTL_INCREMENT.  Thereafter, following each
        // timeout the TTL is incremented by TTL_INCREMENT until TTL =
        // TTL_THRESHOLD is reached.  Beyond this TTL = NET_DIAMETER is used.
        // Once TTL = NET_DIAMETER, the timeout for waiting for the RREP is set
        // to NET_TRAVERSAL_TIME, as specified in section 6.3.

        if (timeToLive != 0) {
            rrepTimerMsg->setLastTTL(timeToLive);
            rrepTimerMsg->setFromInvalidEntry(true);
            cancelEvent(rrepTimerMsg);
        }
        else if (lastTTL + ttlIncrement < ttlThreshold) {
            ASSERT(!rrepTimerMsg->isScheduled());
            timeToLive = lastTTL + ttlIncrement;
            rrepTimerMsg->setLastTTL(lastTTL + ttlIncrement);
        }
        else {
            ASSERT(!rrepTimerMsg->isScheduled());
            timeToLive = netDiameter;
            rrepTimerMsg->setLastTTL(netDiameter);
        }
    }
    else {
        rrepTimerMsg = new WaitForRREP();
        waitForRREPTimers[rreq->getDestAddr()] = rrepTimerMsg;
        ASSERT(hasOngoingRouteDiscovery(rreq->getDestAddr()));

        timeToLive = ttlStart;
        rrepTimerMsg->setLastTTL(ttlStart);
        rrepTimerMsg->setFromInvalidEntry(false);
        rrepTimerMsg->setDestAddr(rreq->getDestAddr());
    }

    // Each time, the timeout for receiving a RREP is RING_TRAVERSAL_TIME.
    simtime_t ringTraversalTime = 2.0 * nodeTraversalTime * (timeToLive + timeoutBuffer);
    scheduleAt(simTime() + ringTraversalTime, rrepTimerMsg);

    EV_INFO << "Sending a Route Request with target " << rreq->getDestAddr() << " and TTL= " << timeToLive << endl;
    sendAODVRelPacket(rreq, destAddr, timeToLive, jitterPar->doubleValue());
    rreqCount++;
}

void AODVRelRouting::sendRREP(AODVRelRREP *rrep, const IPv4Address& destAddr, unsigned int timeToLive)
{
    EV_INFO << "Sending Route Reply to " << destAddr << endl;

    // When any node transmits a RREP, the precursor list for the
    // corresponding destination node is updated by adding to it
    // the next hop node to which the RREP is forwarded.

    IPv4Route *destRoute = routingTable->findBestMatchingRoute(destAddr);
    const IPv4Address& nextHop = destRoute->getGateway();
    AODVRelRouteData *destRouteData = check_and_cast<AODVRelRouteData *>(destRoute->getProtocolData());
    destRouteData->addPrecursor(nextHop);

    // The node we received the Route Request for is our neighbor,
    // it is probably an unidirectional link
    if (destRoute->getMetric() == 1) {
        // It is possible that a RREP transmission may fail, especially if the
        // RREQ transmission triggering the RREP occurs over a unidirectional
        // link.

        rrep->setAckRequiredFlag(true);

        // when a node detects that its transmission of a RREP message has failed,
        // it remembers the next-hop of the failed RREP in a "blacklist" set.

        failedNextHop = nextHop;

        if (rrepAckTimer->isScheduled())
            cancelEvent(rrepAckTimer);

        scheduleAt(simTime() + nextHopWait, rrepAckTimer);
    }
    sendAODVRelPacket(rrep, nextHop, timeToLive, 0);
}

AODVRelRREQ *AODVRelRouting::createRREQ(const IPv4Address& destAddr)
{
    AODVRelRREQ *rreqPacket = new AODVRelRREQ("AODVRel-RREQ");

    rreqPacket->setGratuitousRREPFlag(askGratuitousRREP);
    IPv4Route *lastKnownRoute = routingTable->findBestMatchingRoute(destAddr);

    rreqPacket->setPacketType(RREQ);

    // The Originator Sequence Number in the RREQ message is the
    // node's own sequence number, which is incremented prior to
    // insertion in a RREQ.
    sequenceNum++;

    rreqPacket->setOriginatorSeqNum(sequenceNum);

    if (lastKnownRoute && lastKnownRoute->getSource() == this) {
        // The Destination Sequence Number field in the RREQ message is the last
        // known destination sequence number for this destination and is copied
        // from the Destination Sequence Number field in the routing table.

        AODVRelRouteData *routeData = check_and_cast<AODVRelRouteData *>(lastKnownRoute->getProtocolData());
        if (routeData && routeData->hasValidDestNum()) {
            rreqPacket->setDestSeqNum(routeData->getDestSeqNum());
            rreqPacket->setUnknownSeqNumFlag(false);
        }
        else
            rreqPacket->setUnknownSeqNumFlag(true);
    }
    else
        rreqPacket->setUnknownSeqNumFlag(true); // If no sequence number is known, the unknown sequence number flag MUST be set.

    rreqPacket->setOriginatorAddr(getSelfIPAddress());
    rreqPacket->setDestAddr(destAddr);

    // The RREQ ID field is incremented by one from the last RREQ ID used
    // by the current node. Each node maintains only one RREQ ID.
    rreqId++;
    rreqPacket->setRreqId(rreqId);

    // The Hop Count field is set to zero.
    rreqPacket->setHopCount(0);

    if(!DEBUG){
        // changes 1.1
        std::string state = "create";
        setAdditionalPacketArguments(rreqPacket, state);
    }

    // Before broadcasting the RREQ, the originating node buffers the RREQ
    // ID and the Originator IP address (its own address) of the RREQ for
    // PATH_DISCOVERY_TIME.
    // In this way, when the node receives the packet again from its neighbors,
    // it will not reprocess and re-forward the packet.

    RREQIdentifier rreqIdentifier(getSelfIPAddress(), rreqId);
    rreqsArrivalTime[rreqIdentifier] = simTime();

    return rreqPacket;
}

AODVRelRREP *AODVRelRouting::createRREP(AODVRelRREQ *rreq, IPv4Route *destRoute, IPv4Route *originatorRoute, const IPv4Address& lastHopAddr)
{
    AODVRelRREP *rrep = new AODVRelRREP("AODVRel-RREP");
    rrep->setPacketType(RREP);

    // When generating a RREP message, a node copies the Destination IP
    // IPv4Address and the Originator Sequence Number from the RREQ message into
    // the corresponding fields in the RREP message.

    rrep->setDestAddr(rreq->getDestAddr());
    rrep->setOriginatorSeqNum(rreq->getOriginatorSeqNum());

    // OriginatorAddr = The IP address of the node which originated the RREQ
    // for which the route is supplied.
    rrep->setOriginatorAddr(rreq->getOriginatorAddr());

    // changes 1.1

    if(!DEBUG){
        // use this function to pass additional arguments to packets or modify the existing ones.
        std::string state = "create";
        setAdditionalPacketArguments(rrep, state);

        // copy route reliability from rreq to rrep
        rrep->setRouteReliability(rreq->getRouteReliability());
    }

    // Processing is slightly different, depending on whether the node is
    // itself the requested destination (see section 6.6.1), or instead
    // if it is an intermediate node with an fresh enough route to the destination
    // (see section 6.6.2).

    if (rreq->getDestAddr() == getSelfIPAddress()) {    // node is itself the requested destination
        // If the generating node is the destination itself, it MUST increment
        // its own sequence number by one if the sequence number in the RREQ
        // packet is equal to that incremented value.

        if (!rreq->getUnknownSeqNumFlag() && sequenceNum + 1 == rreq->getDestSeqNum())
            sequenceNum++;

        // The destination node places its (perhaps newly incremented)
        // sequence number into the Destination Sequence Number field of
        // the RREP,
        rrep->setDestSeqNum(sequenceNum);

        // and enters the value zero in the Hop Count field
        // of the RREP.
        rrep->setHopCount(0);

        // The destination node copies the value MY_ROUTE_TIMEOUT
        // into the Lifetime field of the RREP.
        rrep->setLifeTime(myRouteTimeout);
    }
    else {    // intermediate node
              // it copies its known sequence number for the destination into
              // the Destination Sequence Number field in the RREP message.
        AODVRelRouteData *destRouteData = check_and_cast<AODVRelRouteData *>(destRoute->getProtocolData());
        AODVRelRouteData *originatorRouteData = check_and_cast<AODVRelRouteData *>(originatorRoute->getProtocolData());
        rrep->setDestSeqNum(destRouteData->getDestSeqNum());

        // The intermediate node updates the forward route entry by placing the
        // last hop node (from which it received the RREQ, as indicated by the
        // source IP address field in the IP header) into the precursor list for
        // the forward route entry -- i.e., the entry for the Destination IP
        // IPv4Address.
        destRouteData->addPrecursor(lastHopAddr);

        // The intermediate node also updates its route table entry
        // for the node originating the RREQ by placing the next hop towards the
        // destination in the precursor list for the reverse route entry --
        // i.e., the entry for the Originator IP IPv4Address field of the RREQ
        // message data.

        originatorRouteData->addPrecursor(destRoute->getGateway());

        // The intermediate node places its distance in hops from the
        // destination (indicated by the hop count in the routing table)
        // Hop Count field in the RREP.

        rrep->setHopCount(destRoute->getMetric());

        // The Lifetime field of the RREP is calculated by subtracting the
        // current time from the expiration time in its route table entry.

        rrep->setLifeTime(destRouteData->getLifeTime() - simTime());

        // changes 1.1

        if(!DEBUG){
            // handle for intermediate rrep by copying destination route's reliability into rrep
            rrep->setRouteReliability(destRouteData->getRouteReliability()*rreq->getRouteReliability());
            rrep->setReverseRouteReliability(destRouteData->getRouteReliability());
        }



    }
    return rrep;
}

// changes 1.6
AODVRelRREP *AODVRelRouting::createGratuitousRREP(AODVRelRREQ *rreq, IPv4Route *originatorRoute, IPv4Route *destRoute)
{
    ASSERT(originatorRoute != NULL);
    AODVRelRREP *grrep = new AODVRelRREP("AODVRel-GRREP");
    AODVRelRouteData *routeData = check_and_cast<AODVRelRouteData *>(originatorRoute->getProtocolData());

    // changes 1.6
    AODVRelRouteData *destRouteData = check_and_cast<AODVRelRouteData *>(destRoute->getProtocolData());

    // Hop Count                        The Hop Count as indicated in the
    //                                  node's route table entry for the
    //                                  originator
    //
    // Destination IP IPv4Address           The IP address of the node that
    //                                  originated the RREQ
    //
    // Destination Sequence Number      The Originator Sequence Number from
    //                                  the RREQ
    //
    // Originator IP IPv4Address            The IP address of the Destination
    //                                  node in the RREQ
    //
    // Lifetime                         The remaining lifetime of the route
    //                                  towards the originator of the RREQ,
    //                                  as known by the intermediate node.

    grrep->setPacketType(RREP);
    grrep->setHopCount(originatorRoute->getMetric());
    grrep->setDestAddr(rreq->getOriginatorAddr());
    grrep->setDestSeqNum(rreq->getOriginatorSeqNum());
    grrep->setOriginatorAddr(rreq->getDestAddr());
    grrep->setLifeTime(routeData->getLifeTime());

    // changes 1.1

    if(!DEBUG){
        // use this function to pass additional arguments to packets or modify the existing ones.
        std::string state = "create";
        setAdditionalPacketArguments(grrep, state);

        // handle for gratuitous rrep by copying destination route's reliability into rrep
        grrep->setRouteReliability(routeData->getRouteReliability()*destRouteData->getRouteReliability()); // multiply destination route reliability
        grrep->setReverseRouteReliability(routeData->getRouteReliability());
    }

    return grrep;
}

void AODVRelRouting::handleRREP(AODVRelRREP *rrep, const IPv4Address& sourceAddr)
{
    EV_INFO << "AODVRel Route Reply arrived with source addr: " << sourceAddr << " originator addr: " << rrep->getOriginatorAddr()
            << " destination addr: " << rrep->getDestAddr() << endl;

    if (rrep->getOriginatorAddr().isUnspecified()) {
        EV_INFO << "This Route Reply is a Hello Message" << endl;
        handleHelloMessage(rrep);
        delete rrep;
        return;
    }
    // When a node receives a RREP message, it searches (using longest-
    // prefix matching) for a route to the previous hop.

    // If needed, a route is created for the previous hop,
    // but without a valid sequence number (see section 6.2)

    IPv4Route *previousHopRoute = routingTable->findBestMatchingRoute(sourceAddr);


    // changes 1.1
    double linkReliability = 1.0;
    if(!DEBUG){
        linkReliability = prevHopReliability(rrep);
    }

    // changes 1.6
    if (!previousHopRoute || previousHopRoute->getSource() != this) {
        // create without valid sequence number
        previousHopRoute = createRoute(sourceAddr, sourceAddr, 1, false, rrep->getOriginatorSeqNum(), true, simTime() + activeRouteTimeout, linkReliability);
    }
    else
        updateRoutingTable(previousHopRoute, sourceAddr, 1, false, rrep->getOriginatorSeqNum(), true, simTime() + activeRouteTimeout, linkReliability);

    // Next, the node then increments the hop count value in the RREP by one,
    // to account for the new hop through the intermediate node
    unsigned int newHopCount = rrep->getHopCount() + 1;
    rrep->setHopCount(newHopCount);


    // changes 1.1
    double newRouteReliability = 1.0;
    double newReverseRouteReliability = 1.0;
    if(!DEBUG){
        std::string state = "handle";
        setAdditionalPacketArguments(rrep, state);
        newRouteReliability = rrep->getRouteReliability();
        newReverseRouteReliability = rrep->getReverseRouteReliability();
    }




    // Then the forward route for this destination is created if it does not
    // already exist.

    IPv4Route *destRoute = routingTable->findBestMatchingRoute(rrep->getDestAddr());
    AODVRelRouteData *destRouteData = NULL;
    simtime_t lifeTime = rrep->getLifeTime();
    unsigned int destSeqNum = rrep->getDestSeqNum();

    if (destRoute && destRoute->getSource() == this) {    // already exists
        destRouteData = check_and_cast<AODVRelRouteData *>(destRoute->getProtocolData());
        // Upon comparison, the existing entry is updated only in the following circumstances:

        // (i) the sequence number in the routing table is marked as
        //     invalid in route table entry.

        if (!destRouteData->hasValidDestNum()) {

            // changes 1.1
            if(!strcmp(this->routingType, "AODVReliability")){
                updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newRouteReliability);
            }
            else{
                updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newReverseRouteReliability);
            }

            // If the route table entry to the destination is created or updated,
            // then the following actions occur:
            //
            // -  the route is marked as active,
            //
            // -  the destination sequence number is marked as valid,
            //
            // -  the next hop in the route entry is assigned to be the node from
            //    which the RREP is received, which is indicated by the source IP
            //    address field in the IP header,
            //
            // -  the hop count is set to the value of the New Hop Count,
            //
            // -  the expiry time is set to the current time plus the value of the
            //    Lifetime in the RREP message,
            //
            // -  and the destination sequence number is the Destination Sequence
            //    Number in the RREP message.
        }
        // (ii) the Destination Sequence Number in the RREP is greater than
        //      the node's copy of the destination sequence number and the
        //      known value is valid, or
        else if (destSeqNum > destRouteData->getDestSeqNum()) {

            // changes 1.1
            if(!strcmp(this->routingType, "AODVReliability")){
                updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newRouteReliability);
            }
            else{
                updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newReverseRouteReliability);
            }

        }
        else {
            // (iii) the sequence numbers are the same, but the route is
            //       marked as inactive, or
            if (destSeqNum == destRouteData->getDestSeqNum() && !destRouteData->isActive()) {

                // changes 1.1
                if(!strcmp(this->routingType, "AODVReliability")){
                    updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newRouteReliability);
                }
                else{
                    updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newReverseRouteReliability);
                }

            }
            // (iv) the sequence numbers are the same, and the New Hop Count is
            //      smaller than the hop count in route table entry.
            else if (destSeqNum == destRouteData->getDestSeqNum()) {
                // changes 1.1

                if((!strcmp(this->routingType, "AODVReliability") || !strcmp(this->routingType, "AODVError")) && !DEBUG){
                    if(newRouteReliability > destRouteData->getRouteReliability() && !strcmp(this->routingType, "AODVReliability")){
                        updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newRouteReliability);
                    }
                    else if(newReverseRouteReliability > destRouteData->getRouteReliability()){
                        updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newReverseRouteReliability);
                    }
                }
                else{
                    if(newHopCount < (unsigned int)destRoute->getMetric()){
                        if(!strcmp(this->routingType, "AODVReliability")){
                            updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newRouteReliability);
                        }
                        else{
                            updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newReverseRouteReliability);
                        }
                    }
                }
            }
        }
    }
    else {    // create forward route for the destination: this path will be used by the originator to send data packets
        // changes 1.1

        if(!strcmp(this->routingType, "AODVReliability")){
            destRoute = createRoute(rrep->getDestAddr(), sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newRouteReliability);
        }
        else{
            destRoute = createRoute(rrep->getDestAddr(), sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newReverseRouteReliability);
        }

        destRouteData = check_and_cast<AODVRelRouteData *>(destRoute->getProtocolData());
    }

    // If the current node is not the node indicated by the Originator IP
    // IPv4Address in the RREP message AND a forward route has been created or
    // updated as described above, the node consults its route table entry
    // for the originating node to determine the next hop for the RREP
    // packet, and then forwards the RREP towards the originator using the
    // information in that route table entry.

    IPv4Route *originatorRoute = routingTable->findBestMatchingRoute(rrep->getOriginatorAddr());
    if (getSelfIPAddress() != rrep->getOriginatorAddr()) {
        // If a node forwards a RREP over a link that is likely to have errors or
        // be unidirectional, the node SHOULD set the 'A' flag to require that the
        // recipient of the RREP acknowledge receipt of the RREP by sending a RREP-ACK
        // message back (see section 6.8).

        if (originatorRoute && originatorRoute->getSource() == this) {
            AODVRelRouteData *originatorRouteData = check_and_cast<AODVRelRouteData *>(originatorRoute->getProtocolData());

            // Also, at each node the (reverse) route used to forward a
            // RREP has its lifetime changed to be the maximum of (existing-
            // lifetime, (current time + ACTIVE_ROUTE_TIMEOUT).

            simtime_t existingLifeTime = originatorRouteData->getLifeTime();
            originatorRouteData->setLifeTime(std::max(simTime() + activeRouteTimeout, existingLifeTime));

            if (simTime() > rebootTime + deletePeriod || rebootTime == 0) {
                // If a node forwards a RREP over a link that is likely to have errors
                // or be unidirectional, the node SHOULD set the 'A' flag to require that
                // the recipient of the RREP acknowledge receipt of the RREP by sending a
                // RREP-ACK message back (see section 6.8).

                if (rrep->getAckRequiredFlag()) {
                    AODVRelRREPACK *rrepACK = createRREPACK();
                    sendRREPACK(rrepACK, sourceAddr);
                    rrep->setAckRequiredFlag(false);
                }

                // When any node transmits a RREP, the precursor list for the
                // corresponding destination node is updated by adding to it
                // the next hop node to which the RREP is forwarded.

                destRouteData->addPrecursor(originatorRoute->getGateway());

                // Finally, the precursor list for the next hop towards the
                // destination is updated to contain the next hop towards the
                // source (originator).

                IPv4Route *nextHopToDestRoute = routingTable->findBestMatchingRoute(destRoute->getGateway());
                ASSERT(nextHopToDestRoute);
                AODVRelRouteData *nextHopToDestRouteData = check_and_cast<AODVRelRouteData *>(nextHopToDestRoute->getProtocolData());
                nextHopToDestRouteData->addPrecursor(originatorRoute->getGateway());

                AODVRelRREP *outgoingRREP = rrep->dup();
                forwardRREP(outgoingRREP, originatorRoute->getGateway(), 100);
            }
        }
        else
            EV_ERROR << "Reverse route doesn't exist. Dropping the RREP message" << endl;
    }
    else {
        if (hasOngoingRouteDiscovery(rrep->getDestAddr())) {
            EV_INFO << "The Route Reply has arrived for our Route Request to node " << rrep->getDestAddr() << endl;
            // changes 1.1

            if(!strcmp(this->routingType, "AODVReliability")){
                updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newRouteReliability);
            }
            else{
                updateRoutingTable(destRoute, sourceAddr, newHopCount, true, destSeqNum, true, simTime() + lifeTime, newReverseRouteReliability);
            }

            completeRouteDiscovery(rrep->getDestAddr());
        }
    }

    delete rrep;
}

void AODVRelRouting::updateRoutingTable(IPv4Route *route, const IPv4Address& nextHop, unsigned int hopCount, bool hasValidDestNum, unsigned int destSeqNum, bool isActive, simtime_t lifeTime, double routeReliability)
{
    EV_DETAIL << "Updating existing route: " << route << endl;

    route->setGateway(nextHop);
    route->setMetric(hopCount);

    AODVRelRouteData *routingData = check_and_cast<AODVRelRouteData *>(route->getProtocolData());
    ASSERT(routingData != NULL);

    routingData->setLifeTime(lifeTime);
    routingData->setDestSeqNum(destSeqNum);
    routingData->setIsActive(isActive);
    routingData->setHasValidDestNum(hasValidDestNum);

    // changes 1.1
    if(!DEBUG){
        routingData->setRouteReliability(routeReliability);
    }
    else{
        routingData->setRouteReliability(1.0);
    }

    EV_DETAIL << "Route updated: " << route << endl;

    scheduleExpungeRoutes();
}

void AODVRelRouting::sendAODVRelPacket(AODVRelControlPacket *packet, const IPv4Address& destAddr, unsigned int timeToLive, double delay)
{
    ASSERT(timeToLive != 0);

    IPv4ControlInfo *networkProtocolControlInfo = new IPv4ControlInfo();

    networkProtocolControlInfo->setTimeToLive(timeToLive);

    networkProtocolControlInfo->setProtocol(IP_PROT_MANET);
    networkProtocolControlInfo->setDestAddr(destAddr);
    networkProtocolControlInfo->setSrcAddr(getSelfIPAddress());

    // TODO: Implement: support for multiple interfaces
    InterfaceEntry *ifEntry = interfaceTable->getInterfaceByName("nic");
    networkProtocolControlInfo->setInterfaceId(ifEntry->getInterfaceId());

    UDPPacket *udpPacket = new UDPPacket(packet->getName());
    udpPacket->encapsulate(packet);
    udpPacket->setSourcePort(AODVRelUDPPort);
    udpPacket->setDestinationPort(AODVRelUDPPort);
    udpPacket->setControlInfo(dynamic_cast<cObject *>(networkProtocolControlInfo));

    if (destAddr.isLimitedBroadcastAddress())
        lastBroadcastTime = simTime();

    if (delay == 0)
        send(udpPacket, "ipOut");
    else
        sendDelayed(udpPacket, delay, "ipOut");
}

void AODVRelRouting::handleRREQ(AODVRelRREQ *rreq, const IPv4Address& sourceAddr, unsigned int timeToLive)
{
    EV_INFO << "AODVRel Route Request arrived with source addr: " << sourceAddr << " originator addr: " << rreq->getOriginatorAddr()
            << " destination addr: " << rreq->getDestAddr() << endl;

    // A node ignores all RREQs received from any node in its blacklist set.

    std::map<IPv4Address, simtime_t>::iterator blackListIt = blacklist.find(sourceAddr);
    if (blackListIt != blacklist.end()) {
        EV_INFO << "The sender node " << sourceAddr << " is in our blacklist. Ignoring the Route Request" << endl;
        delete rreq;
        return;
    }

    // When a node receives a RREQ, it first creates or updates a route to
    // the previous hop without a valid sequence number (see section 6.2).

    IPv4Route *previousHopRoute = routingTable->findBestMatchingRoute(sourceAddr);

    // changes 1.1
    double linkReliability = 1.0;
    if(!DEBUG){
        linkReliability = prevHopReliability(rreq);
    }

    if (!previousHopRoute || previousHopRoute->getSource() != this) {
        // create without valid sequence number
        previousHopRoute = createRoute(sourceAddr, sourceAddr, 1, false, rreq->getOriginatorSeqNum(), true, simTime() + activeRouteTimeout, linkReliability);
    }
    else
        updateRoutingTable(previousHopRoute, sourceAddr, 1, false, rreq->getOriginatorSeqNum(), true, simTime() + activeRouteTimeout, linkReliability);

    // then checks to determine whether it has received a RREQ with the same
    // Originator IP address and RREQ ID within at least the last PATH_DISCOVERY_TIME.
    // If such a RREQ has been received, the node silently discards the newly received RREQ.

    RREQIdentifier rreqIdentifier(rreq->getOriginatorAddr(), rreq->getRreqId());
    std::map<RREQIdentifier, simtime_t, RREQIdentifierCompare>::iterator checkRREQArrivalTime = rreqsArrivalTime.find(rreqIdentifier);
    if (checkRREQArrivalTime != rreqsArrivalTime.end() && simTime() - checkRREQArrivalTime->second <= pathDiscoveryTime) {
        EV_WARN << "The same packet has arrived within PATH_DISCOVERY_TIME= " << pathDiscoveryTime << ". Discarding it" << endl;
        delete rreq;
        return;
    }

    // update or create
    rreqsArrivalTime[rreqIdentifier] = simTime();

    // First, it first increments the hop count value in the RREQ by one, to
    // account for the new hop through the intermediate node.

    rreq->setHopCount(rreq->getHopCount() + 1);

    // changes 1.1
    if(!DEBUG){
        std::string state = "handle";
        EV_DEBUG << "###&&&$$$ oldMobility: " << rreq->getXPosition() << rreq->getYPosition() << rreq->getXVelocity() << rreq->getYVelocity() << endl;
        setAdditionalPacketArguments(rreq, state);
        EV_DEBUG << "###&&&$$$ newMobility: " << rreq->getXPosition() << rreq->getYPosition() << rreq->getXVelocity() << rreq->getYVelocity() << endl;
    }

    // Then the node searches for a reverse route to the Originator IP Address (see
    // section 6.2), using longest-prefix matching.

    IPv4Route *reverseRoute = routingTable->findBestMatchingRoute(rreq->getOriginatorAddr());

    // If need be, the route is created, or updated using the Originator Sequence Number from the
    // RREQ in its routing table.
    //
    // When the reverse route is created or updated, the following actions on
    // the route are also carried out:
    //
    //   1. the Originator Sequence Number from the RREQ is compared to the
    //      corresponding destination sequence number in the route table entry
    //      and copied if greater than the existing value there
    //
    //   2. the valid sequence number field is set to true;
    //
    //   3. the next hop in the routing table becomes the node from which the
    //      RREQ was received (it is obtained from the source IP address in
    //      the IP header and is often not equal to the Originator IP Address
    //      field in the RREQ message);
    //
    //   4. the hop count is copied from the Hop Count in the RREQ message;
    //
    //   Whenever a RREQ message is received, the Lifetime of the reverse
    //   route entry for the Originator IP address is set to be the maximum of
    //   (ExistingLifetime, MinimalLifetime), where
    //
    //   MinimalLifetime = (current time + 2*NET_TRAVERSAL_TIME - 2*HopCount*NODE_TRAVERSAL_TIME).

    unsigned int hopCount = rreq->getHopCount();
    simtime_t minimalLifeTime = simTime() + 2 * netTraversalTime - 2 * hopCount * nodeTraversalTime;
    simtime_t newLifeTime = std::max(simTime(), minimalLifeTime);
    int rreqSeqNum = rreq->getOriginatorSeqNum();
    if (!reverseRoute || reverseRoute->getSource() != this) {    // create
        // This reverse route will be needed if the node receives a RREP back to the
        // node that originated the RREQ (identified by the Originator IP Address).
        // changes 1.1
        reverseRoute = createRoute(rreq->getOriginatorAddr(), sourceAddr, hopCount, true, rreqSeqNum, true, newLifeTime, rreq->getRouteReliability());
    }
    else {
        AODVRelRouteData *routeData = check_and_cast<AODVRelRouteData *>(reverseRoute->getProtocolData());
        int routeSeqNum = routeData->getDestSeqNum();
        int newSeqNum = std::max(routeSeqNum, rreqSeqNum);
        int newHopCount = rreq->getHopCount();    // Note: already incremented by 1.
        int routeHopCount = reverseRoute->getMetric();
        // The route is only updated if the new sequence number is either
        //
        //   (i)       higher than the destination sequence number in the route
        //             table, or
        //
        //   (ii)      the sequence numbers are equal, but the hop count (of the
        //             new information) plus one, is smaller than the existing hop
        //             count in the routing table, or
        //
        //   (iii)     the sequence number is unknown.

        // changes 1.1
        if((!strcmp(this->routingType, "AODVReliability") || !strcmp(this->routingType, "AODVError")) && !DEBUG){
            // make decision based upon reliability
            double newRouteReliability = rreq->getRouteReliability();
            double oldRouteReliability = routeData->getRouteReliability();
            if (rreqSeqNum > routeSeqNum ||
                (rreqSeqNum == routeSeqNum && newRouteReliability > oldRouteReliability) ||
                rreq->getUnknownSeqNumFlag())
            {
                updateRoutingTable(reverseRoute, sourceAddr, hopCount, true, newSeqNum, true, newLifeTime, rreq->getRouteReliability());
            }
        }
        else{
            if (rreqSeqNum > routeSeqNum ||
                (rreqSeqNum == routeSeqNum && newHopCount < routeHopCount) ||
                rreq->getUnknownSeqNumFlag())
            {
                updateRoutingTable(reverseRoute, sourceAddr, hopCount, true, newSeqNum, true, newLifeTime, rreq->getRouteReliability());
            }
        }

    }

    // A node generates a RREP if either:
    //
    // (i)       it is itself the destination, or
    //
    // (ii)      it has an active route to the destination, the destination
    //           sequence number in the node's existing route table entry
    //           for the destination is valid and greater than or equal to
    //           the Destination Sequence Number of the RREQ (comparison
    //           using signed 32-bit arithmetic), and the "destination only"
    //           ('D') flag is NOT set.

    // After a node receives a RREQ and responds with a RREP, it discards
    // the RREQ.  If the RREQ has the 'G' flag set, and the intermediate
    // node returns a RREP to the originating node, it MUST also unicast a
    // gratuitous RREP to the destination node.

    IPv4Route *destRoute = routingTable->findBestMatchingRoute(rreq->getDestAddr());
    AODVRelRouteData *destRouteData = destRoute ? dynamic_cast<AODVRelRouteData *>(destRoute->getProtocolData()) : NULL;

    // check (i)
    if (rreq->getDestAddr() == getSelfIPAddress()) {
        EV_INFO << "I am the destination node for which the route was requested" << endl;

        // create RREP
        AODVRelRREP *rrep = createRREP(rreq, destRoute, reverseRoute, sourceAddr);

        // send to the originator
        sendRREP(rrep, rreq->getOriginatorAddr(), 255);

        delete rreq;
        return;    // discard RREQ, in this case, we do not forward it.
    }

    // check (ii)
    if (destRouteData && destRouteData->isActive() && destRouteData->hasValidDestNum() &&
        destRouteData->getDestSeqNum() >= rreq->getDestSeqNum())
    {
        EV_INFO << "I am an intermediate node who has information about a route to " << rreq->getDestAddr() << endl;

        if (destRoute->getGateway() == sourceAddr) {
            EV_WARN << "This RREP would make a loop. Dropping it" << endl;

            delete rreq;
            return;
        }

        // create RREP
        AODVRelRREP *rrep = createRREP(rreq, destRoute, reverseRoute, sourceAddr);

        // send to the originator
        sendRREP(rrep, rreq->getOriginatorAddr(), 255);

        if (rreq->getGratuitousRREPFlag()) {
            // The gratuitous RREP is then sent to the next hop along the path to
            // the destination node, just as if the destination node had already
            // issued a RREQ for the originating node and this RREP was produced in
            // response to that (fictitious) RREQ.

            IPv4Route *originatorRoute = routingTable->findBestMatchingRoute(rreq->getOriginatorAddr());
            AODVRelRREP *grrep = createGratuitousRREP(rreq, originatorRoute, destRoute);
            sendGRREP(grrep, rreq->getDestAddr(), 100);
        }

        delete rreq;
        return;    // discard RREQ, in this case, we also do not forward it.
    }
    // If a node does not generate a RREP (following the processing rules in
    // section 6.6), and if the incoming IP header has TTL larger than 1,
    // the node updates and broadcasts the RREQ to address 255.255.255.255
    // on each of its configured interfaces (see section 6.14).  To update
    // the RREQ, the TTL or hop limit field in the outgoing IP header is
    // decreased by one, and the Hop Count field in the RREQ message is
    // incremented by one, to account for the new hop through the
    // intermediate node. (!) Lastly, the Destination Sequence number for the
    // requested destination is set to the maximum of the corresponding
    // value received in the RREQ message, and the destination sequence
    // value currently maintained by the node for the requested destination.
    // However, the forwarding node MUST NOT modify its maintained value for
    // the destination sequence number, even if the value received in the
    // incoming RREQ is larger than the value currently maintained by the
    // forwarding node.

    if (timeToLive > 0 && (simTime() > rebootTime + deletePeriod || rebootTime == 0)) {
        if (destRouteData)
            rreq->setDestSeqNum(std::max(destRouteData->getDestSeqNum(), rreq->getDestSeqNum()));
        rreq->setUnknownSeqNumFlag(false);

        AODVRelRREQ *outgoingRREQ = rreq->dup();
        forwardRREQ(outgoingRREQ, timeToLive);
    }
    else
        EV_WARN << "Can't forward the RREQ because of its small (<= 1) TTL: " << timeToLive << " or the AODVRel reboot has not completed yet" << endl;

    delete rreq;
}

// changes 1.6
IPv4Route *AODVRelRouting::createRoute(const IPv4Address& destAddr, const IPv4Address& nextHop,
        unsigned int hopCount, bool hasValidDestNum, unsigned int destSeqNum,
        bool isActive, simtime_t lifeTime, double routeReliability)
{
    IPv4Route *newRoute = new IPv4Route();
    AODVRelRouteData *newProtocolData = new AODVRelRouteData();

    newProtocolData->setHasValidDestNum(hasValidDestNum);

    // active route
    newProtocolData->setIsActive(isActive);

    // A route towards a destination that has a routing table entry
    // that is marked as valid.  Only active routes can be used to
    // forward data packets.

    newProtocolData->setLifeTime(lifeTime);
    newProtocolData->setDestSeqNum(destSeqNum);

    // changes 1.1
    if(!DEBUG){
        newProtocolData->setRouteReliability(routeReliability);
    }
    else{
        newProtocolData->setRouteReliability(1.0);
    }

    InterfaceEntry *ifEntry = interfaceTable->getInterfaceByName("nic");    // TODO: IMPLEMENT: multiple interfaces
    if (ifEntry)
        newRoute->setInterface(ifEntry);

    newRoute->setDestination(destAddr);
    newRoute->setSourceType(IPv4Route::AODVRel);
    newRoute->setSource(this);
    newRoute->setProtocolData(newProtocolData);
    newRoute->setMetric(hopCount);
    newRoute->setGateway(nextHop);
    newRoute->setNetmask(IPv4Address::ALLONES_ADDRESS);    // TODO:

    EV_DETAIL << "Adding new route " << newRoute << endl;
    routingTable->addRoute(newRoute);
    scheduleExpungeRoutes();
    return newRoute;
}

void AODVRelRouting::receiveChangeNotification(int signalID, const cObject *obj)
{
    Enter_Method("receiveChangeNotification");
    if (signalID == NF_LINK_BREAK) {
        // changes 1.5
        emit(statLinkFailure, NULL);

        EV_DETAIL << "Received link break signal" << endl;
        // XXX: This is a hack for supporting both IdealMac and Ieee80211Mac.
        Ieee80211Frame *ieee80211Frame = dynamic_cast<Ieee80211Frame *>(const_cast<cObject *>(obj));
        IPv4Datagram *datagram = dynamic_cast<IPv4Datagram *> (ieee80211Frame == NULL ? const_cast<cObject *>(obj) : ieee80211Frame->getEncapsulatedPacket());
        if (datagram) {
            const IPv4Address& unreachableAddr = datagram->getDestAddress();
            if (true) {
                // A node initiates processing for a RERR message in three situations:
                //
                //   (i)     if it detects a link break for the next hop of an active
                //           route in its routing table while transmitting data (and
                //           route repair, if attempted, was unsuccessful), or

                // TODO: Implement: local repair

                IPv4Route *route = routingTable->findBestMatchingRoute(unreachableAddr);

                if (route && route->getSource() == this)
                    handleLinkBreakSendRERR(route->getGateway());
            }
        }

        // changes 1.6
//        else
//            throw cRuntimeError("Unknown packet type in NF_LINK_BREAK notification");
    }
}

void AODVRelRouting::handleLinkBreakSendRERR(const IPv4Address& unreachableAddr)
{
    // For case (i), the node first makes a list of unreachable destinations
    // consisting of the unreachable neighbor and any additional
    // destinations (or subnets, see section 7) in the local routing table
    // that use the unreachable neighbor as the next hop.

    // Just before transmitting the RERR, certain updates are made on the
    // routing table that may affect the destination sequence numbers for
    // the unreachable destinations.  For each one of these destinations,
    // the corresponding routing table entry is updated as follows:
    //
    // 1. The destination sequence number of this routing entry, if it
    //    exists and is valid, is incremented for cases (i) and (ii) above,
    //    and copied from the incoming RERR in case (iii) above.
    //
    // 2. The entry is invalidated by marking the route entry as invalid
    //
    // 3. The Lifetime field is updated to current time plus DELETE_PERIOD.
    //    Before this time, the entry SHOULD NOT be deleted.

    IPv4Route *unreachableRoute = routingTable->findBestMatchingRoute(unreachableAddr);

    if (!unreachableRoute || unreachableRoute->getSource() != this)
        return;

    std::vector<UnreachableNode> unreachableNodes;
    AODVRelRouteData *unreachableRouteData = check_and_cast<AODVRelRouteData *>(unreachableRoute->getProtocolData());
    UnreachableNode node;
    node.addr = unreachableAddr;
    node.seqNum = unreachableRouteData->getDestSeqNum();
    unreachableNodes.push_back(node);

    // For case (i), the node first makes a list of unreachable destinations
    // consisting of the unreachable neighbor and any additional destinations
    // (or subnets, see section 7) in the local routing table that use the
    // unreachable neighbor as the next hop.

    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IPv4Route *route = routingTable->getRoute(i);

        if (route->getGateway() == unreachableAddr) {
            AODVRelRouteData *routeData = check_and_cast<AODVRelRouteData *>(route->getProtocolData());

            if (routeData->hasValidDestNum())
                routeData->setDestSeqNum(routeData->getDestSeqNum() + 1);

            EV_DETAIL << "Marking route to " << route->getDestination() << " as inactive" << endl;

            routeData->setIsActive(false);
            routeData->setLifeTime(simTime() + deletePeriod);
            scheduleExpungeRoutes();

            UnreachableNode node;
            node.addr = route->getDestination();
            node.seqNum = routeData->getDestSeqNum();
            unreachableNodes.push_back(node);
        }
    }

    // The neighboring node(s) that should receive the RERR are all those
    // that belong to a precursor list of at least one of the unreachable
    // destination(s) in the newly created RERR.  In case there is only one
    // unique neighbor that needs to receive the RERR, the RERR SHOULD be
    // unicast toward that neighbor.  Otherwise the RERR is typically sent
    // to the local broadcast address (Destination IP == 255.255.255.255,
    // TTL == 1) with the unreachable destinations, and their corresponding
    // destination sequence numbers, included in the packet.

    if (rerrCount >= rerrRatelimit) {
        EV_WARN << "A node should not generate more than RERR_RATELIMIT RERR messages per second. Canceling sending RERR" << endl;
        return;
    }

    AODVRelRERR *rerr = createRERR(unreachableNodes);
    rerrCount++;

    // broadcast
    EV_INFO << "Broadcasting Route Error message with TTL=1" << endl;
    sendAODVRelPacket(rerr, IPv4Address::ALLONES_ADDRESS, 1, jitterPar->doubleValue());
}

AODVRelRERR *AODVRelRouting::createRERR(const std::vector<UnreachableNode>& unreachableNodes)
{
    AODVRelRERR *rerr = new AODVRelRERR("AODVRel-RERR");
    unsigned int destCount = unreachableNodes.size();

    rerr->setPacketType(RERR);
    rerr->setDestCount(destCount);
    rerr->setUnreachableNodesArraySize(destCount);

    for (unsigned int i = 0; i < destCount; i++) {
        UnreachableNode node;
        node.addr = unreachableNodes[i].addr;
        node.seqNum = unreachableNodes[i].seqNum;
        rerr->setUnreachableNodes(i, node);
    }
    return rerr;
}

void AODVRelRouting::handleRERR(AODVRelRERR *rerr, const IPv4Address& sourceAddr)
{
    EV_INFO << "AODVRel Route Error arrived with source addr: " << sourceAddr << endl;

    // A node initiates processing for a RERR message in three situations:
    // (iii)   if it receives a RERR from a neighbor for one or more
    //         active routes.
    unsigned int unreachableArraySize = rerr->getUnreachableNodesArraySize();
    std::vector<UnreachableNode> unreachableNeighbors;

    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IPv4Route *route = routingTable->getRoute(i);
        AODVRelRouteData *routeData = route ? dynamic_cast<AODVRelRouteData *>(route->getProtocolData()) : NULL;

        if (!routeData)
            continue;

        // For case (iii), the list should consist of those destinations in the RERR
        // for which there exists a corresponding entry in the local routing
        // table that has the transmitter of the received RERR as the next hop.

        if (route->getGateway() == sourceAddr) {
            for (unsigned int j = 0; j < unreachableArraySize; j++) {
                if (route->getDestination() == rerr->getUnreachableNodes(j).addr) {
                    // 1. The destination sequence number of this routing entry, if it
                    // exists and is valid, is incremented for cases (i) and (ii) above,
                    // ! and copied from the incoming RERR in case (iii) above.

                    routeData->setDestSeqNum(rerr->getUnreachableNodes(j).seqNum);
                    routeData->setIsActive(false);    // it means invalid, see 3. AODVRel Terminology p.3. in RFC 3561
                    routeData->setLifeTime(simTime() + deletePeriod);

                    // The RERR should contain those destinations that are part of
                    // the created list of unreachable destinations and have a non-empty
                    // precursor list.

                    if (routeData->getPrecursorList().size() > 0) {
                        UnreachableNode node;
                        node.addr = route->getDestination();
                        node.seqNum = routeData->getDestSeqNum();
                        unreachableNeighbors.push_back(node);
                    }
                    scheduleExpungeRoutes();
                }
            }
        }
    }

    if (rerrCount >= rerrRatelimit) {
        EV_WARN << "A node should not generate more than RERR_RATELIMIT RERR messages per second. Canceling sending RERR" << endl;
        delete rerr;
        return;
    }

    if (unreachableNeighbors.size() > 0 && (simTime() > rebootTime + deletePeriod || rebootTime == 0)) {
        EV_INFO << "Sending RERR to inform our neighbors about link breaks." << endl;
        AODVRelRERR *newRERR = createRERR(unreachableNeighbors);
        sendAODVRelPacket(newRERR, IPv4Address::ALLONES_ADDRESS, 1, 0);
        rerrCount++;
    }
    delete rerr;
}

bool AODVRelRouting::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();
    if (dynamic_cast<NodeStartOperation *>(operation)) {
        if (stage == NodeStartOperation::STAGE_APPLICATION_LAYER) {
            isOperational = true;
            rebootTime = simTime();

            if (useHelloMessages)
                scheduleAt(simTime() + helloInterval - periodicJitter->doubleValue(), helloMsgTimer);

            scheduleAt(simTime() + 1, counterTimer);
        }
    }
    else if (dynamic_cast<NodeShutdownOperation *>(operation)) {
        if (stage == NodeShutdownOperation::STAGE_APPLICATION_LAYER) {
            isOperational = false;
            clearState();
        }
    }
    else if (dynamic_cast<NodeCrashOperation *>(operation)) {
        if (stage == NodeCrashOperation::STAGE_CRASH) {
            isOperational = false;
            clearState();
        }
    }
    else
        throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());

    return true;
}

void AODVRelRouting::clearState()
{
    rerrCount = rreqCount = rreqId = sequenceNum = 0;
    addressToRreqRetries.clear();
    for (std::map<IPv4Address, WaitForRREP *>::iterator it = waitForRREPTimers.begin(); it != waitForRREPTimers.end(); ++it)
        cancelAndDelete(it->second);

    // FIXME: Drop the queued datagrams.
    //for (std::multimap<IPv4Address, IPv4Datagram *>::iterator it = targetAddressToDelayedPackets.begin(); it != targetAddressToDelayedPackets.end(); it++)
    //    networkProtocol->dropQueuedDatagram(const_cast<const IPv4Datagram *>(it->second));

    targetAddressToDelayedPackets.clear();

    waitForRREPTimers.clear();
    rreqsArrivalTime.clear();

    if (useHelloMessages)
        cancelEvent(helloMsgTimer);

    cancelEvent(expungeTimer);
    cancelEvent(counterTimer);
    cancelEvent(blacklistTimer);
    cancelEvent(rrepAckTimer);
}

void AODVRelRouting::handleWaitForRREP(WaitForRREP *rrepTimer)
{
    EV_INFO << "We didn't get any Route Reply within RREP timeout" << endl;
    IPv4Address destAddr = rrepTimer->getDestAddr();

    ASSERT(addressToRreqRetries.find(destAddr) != addressToRreqRetries.end());
    if (addressToRreqRetries[destAddr] == rreqRetries) {
        EV_WARN << "Re-discovery attempts for node " << destAddr << " reached RREQ_RETRIES= " << rreqRetries << " limit. Stop sending RREQ." << endl;
        return;
    }

    AODVRelRREQ *rreq = createRREQ(destAddr);

    // the node MAY try again to discover a route by broadcasting another
    // RREQ, up to a maximum of RREQ_RETRIES times at the maximum TTL value.
    if (rrepTimer->getLastTTL() == netDiameter) // netDiameter is the maximum TTL value
        addressToRreqRetries[destAddr]++;

    sendRREQ(rreq, IPv4Address::ALLONES_ADDRESS, 0);
}

void AODVRelRouting::forwardRREP(AODVRelRREP *rrep, const IPv4Address& destAddr, unsigned int timeToLive)
{
    EV_INFO << "Forwarding the Route Reply to the node " << rrep->getOriginatorAddr() << " which originated the Route Request" << endl;

    // RFC 5148:
    // When a node forwards a message, it SHOULD be jittered by delaying it
    // by a random duration.  This delay SHOULD be generated uniformly in an
    // interval between zero and MAXJITTER.
    sendAODVRelPacket(rrep, destAddr, 100, jitterPar->doubleValue());
}

void AODVRelRouting::forwardRREQ(AODVRelRREQ *rreq, unsigned int timeToLive)
{
    EV_INFO << "Forwarding the Route Request message with TTL= " << timeToLive << endl;
    sendAODVRelPacket(rreq, IPv4Address::ALLONES_ADDRESS, timeToLive, jitterPar->doubleValue());
}

void AODVRelRouting::completeRouteDiscovery(const IPv4Address& target)
{
    EV_DETAIL << "Completing route discovery, originator " << getSelfIPAddress() << ", target " << target << endl;
    ASSERT(hasOngoingRouteDiscovery(target));

    std::multimap<IPv4Address, IPv4Datagram *>::iterator lt = targetAddressToDelayedPackets.lower_bound(target);
    std::multimap<IPv4Address, IPv4Datagram *>::iterator ut = targetAddressToDelayedPackets.upper_bound(target);

    // reinject the delayed datagrams
    for (std::multimap<IPv4Address, IPv4Datagram *>::iterator it = lt; it != ut; it++) {
        IPv4Datagram *datagram = it->second;
        EV_DETAIL << "Sending queued datagram: source " << datagram->getSrcAddress() << ", destination " << datagram->getDestAddress() << endl;
        networkProtocol->reinjectQueuedDatagram(const_cast<const IPv4Datagram *>(datagram));
    }

    // clear the multimap
    targetAddressToDelayedPackets.erase(lt, ut);

    // we have a route for the destination, thus we must cancel the WaitForRREPTimer events
    std::map<IPv4Address, WaitForRREP *>::iterator waitRREPIter = waitForRREPTimers.find(target);
    ASSERT(waitRREPIter != waitForRREPTimers.end());
    cancelAndDelete(waitRREPIter->second);
    waitForRREPTimers.erase(waitRREPIter);
}

void AODVRelRouting::sendGRREP(AODVRelRREP *grrep, const IPv4Address& destAddr, unsigned int timeToLive)
{
    EV_INFO << "Sending gratuitous Route Reply to " << destAddr << endl;

    IPv4Route *destRoute = routingTable->findBestMatchingRoute(destAddr);
    const IPv4Address& nextHop = destRoute->getGateway();

    sendAODVRelPacket(grrep, nextHop, timeToLive, 0);
}

AODVRelRREP *AODVRelRouting::createHelloMessage()
{
    // called a Hello message, with the RREP
    // message fields set as follows:
    //
    //    Destination IP IPv4Address         The node's IP address.
    //
    //    Destination Sequence Number    The node's latest sequence number.
    //
    //    Hop Count                      0
    //
    //    Lifetime                       ALLOWED_HELLO_LOSS *HELLO_INTERVAL

    AODVRelRREP *helloMessage = new AODVRelRREP("AODVRel-HelloMsg");
    helloMessage->setPacketType(RREP);
    helloMessage->setDestAddr(getSelfIPAddress());
    helloMessage->setDestSeqNum(sequenceNum);
    helloMessage->setHopCount(0);
    helloMessage->setLifeTime(allowedHelloLoss * helloInterval);

    return helloMessage;
}

void AODVRelRouting::sendHelloMessagesIfNeeded()
{
    ASSERT(useHelloMessages);
    // Every HELLO_INTERVAL milliseconds, the node checks whether it has
    // sent a broadcast (e.g., a RREQ or an appropriate layer 2 message)
    // within the last HELLO_INTERVAL.  If it has not, it MAY broadcast
    // a RREP with TTL = 1

    // A node SHOULD only use hello messages if it is part of an
    // active route.
    bool hasActiveRoute = false;

    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IPv4Route *route = routingTable->getRoute(i);
        if (route->getSource() == this) {
            AODVRelRouteData *routeData = check_and_cast<AODVRelRouteData *>(route->getProtocolData());
            if (routeData->isActive()) {
                hasActiveRoute = true;
                break;
            }
        }
    }

    if (hasActiveRoute && (lastBroadcastTime == 0 || simTime() - lastBroadcastTime > helloInterval)) {
        EV_INFO << "It is hello time, broadcasting Hello Messages with TTL=1" << endl;
        AODVRelRREP *helloMessage = createHelloMessage();
        sendAODVRelPacket(helloMessage, IPv4Address::ALLONES_ADDRESS, 1, 0);
    }

    scheduleAt(simTime() + helloInterval - periodicJitter->doubleValue(), helloMsgTimer);
}

void AODVRelRouting::handleHelloMessage(AODVRelRREP *helloMessage)
{
    const IPv4Address& helloOriginatorAddr = helloMessage->getDestAddr();
    IPv4Route *routeHelloOriginator = routingTable->findBestMatchingRoute(helloOriginatorAddr);

    // Whenever a node receives a Hello message from a neighbor, the node
    // SHOULD make sure that it has an active route to the neighbor, and
    // create one if necessary.  If a route already exists, then the
    // Lifetime for the route should be increased, if necessary, to be at
    // least ALLOWED_HELLO_LOSS * HELLO_INTERVAL.  The route to the
    // neighbor, if it exists, MUST subsequently contain the latest
    // Destination Sequence Number from the Hello message.  The current node
    // can now begin using this route to forward data packets.  Routes that
    // are created by hello messages and not used by any other active routes
    // will have empty precursor lists and would not trigger a RERR message
    // if the neighbor moves away and a neighbor timeout occurs.

    unsigned int latestDestSeqNum = helloMessage->getDestSeqNum();
    simtime_t newLifeTime = simTime() + allowedHelloLoss * helloInterval;

    // changes 1.1
    double linkReliability = 1.0;
    if(!DEBUG){
        linkReliability = prevHopReliability(helloMessage);
    }

    // changes 1.6

    if (!routeHelloOriginator || routeHelloOriginator->getSource() != this)
        createRoute(helloOriginatorAddr, helloOriginatorAddr, 1, true, latestDestSeqNum, true, newLifeTime, linkReliability);
    else {
        AODVRelRouteData *routeData = check_and_cast<AODVRelRouteData *>(routeHelloOriginator->getProtocolData());
        simtime_t lifeTime = routeData->getLifeTime();
        updateRoutingTable(routeHelloOriginator, helloOriginatorAddr, 1, true, latestDestSeqNum, true, std::max(lifeTime, newLifeTime), linkReliability);
    }

    // TODO: This feature has not implemented yet.
    // A node MAY determine connectivity by listening for packets from its
    // set of neighbors.  If, within the past DELETE_PERIOD, it has received
    // a Hello message from a neighbor, and then for that neighbor does not
    // receive any packets (Hello messages or otherwise) for more than
    // ALLOWED_HELLO_LOSS * HELLO_INTERVAL milliseconds, the node SHOULD
    // assume that the link to this neighbor is currently lost.  When this
    // happens, the node SHOULD proceed as in Section 6.11.
}

void AODVRelRouting::expungeRoutes()
{
    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IPv4Route *route = routingTable->getRoute(i);
        if (route->getSource() == this) {
            AODVRelRouteData *routeData = check_and_cast<AODVRelRouteData *>(route->getProtocolData());
            ASSERT(routeData != NULL);
            if (routeData->getLifeTime() <= simTime()) {
                if (routeData->isActive()) {
                    EV_DETAIL << "Route to " << route->getDestination() << " expired and set to inactive. It will be deleted after DELETE_PERIOD time" << endl;
                    // An expired routing table entry SHOULD NOT be expunged before
                    // (current_time + DELETE_PERIOD) (see section 6.11).  Otherwise, the
                    // soft state corresponding to the route (e.g., last known hop count)
                    // will be lost.
                    routeData->setIsActive(false);
                    routeData->setLifeTime(simTime() + deletePeriod);
                }
                else {
                    // Any routing table entry waiting for a RREP SHOULD NOT be expunged
                    // before (current_time + 2 * NET_TRAVERSAL_TIME).
                    if (hasOngoingRouteDiscovery(route->getDestination())) {
                        EV_DETAIL << "Route to " << route->getDestination() << " expired and is inactive, but we are waiting for a RREP to this destination, so we extend its lifetime with 2 * NET_TRAVERSAL_TIME" << endl;
                        routeData->setLifeTime(simTime() + 2 * netTraversalTime);
                    }
                    else {
                        EV_DETAIL << "Route to " << route->getDestination() << " expired and is inactive and we are not expecting any RREP to this destination, so we delete this route" << endl;
                        routingTable->deleteRoute(route);
                    }
                }
            }
        }
    }
    scheduleExpungeRoutes();
}

void AODVRelRouting::scheduleExpungeRoutes()
{
    simtime_t nextExpungeTime = SimTime::getMaxTime();
    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        IPv4Route *route = routingTable->getRoute(i);

        if (route->getSource() == this) {
            AODVRelRouteData *routeData = check_and_cast<AODVRelRouteData *>(route->getProtocolData());
            ASSERT(routeData != NULL);

            if (routeData->getLifeTime() < nextExpungeTime)
                nextExpungeTime = routeData->getLifeTime();
        }
    }
    if (nextExpungeTime == SimTime::getMaxTime()) {
        if (expungeTimer->isScheduled())
            cancelEvent(expungeTimer);
    }
    else {
        if (!expungeTimer->isScheduled())
            scheduleAt(nextExpungeTime, expungeTimer);
        else {
            if (expungeTimer->getArrivalTime() != nextExpungeTime) {
                cancelEvent(expungeTimer);
                scheduleAt(nextExpungeTime, expungeTimer);
            }
        }
    }
}

INetfilter::IHook::Result AODVRelRouting::datagramForwardHook(IPv4Datagram *datagram, const InterfaceEntry *inputInterfaceEntry, const InterfaceEntry *& outputInterfaceEntry, IPv4Address& nextHopAddress)
{
    // TODO: Implement: Actions After Reboot
    // If the node receives a data packet for some other destination, it SHOULD
    // broadcast a RERR as described in subsection 6.11 and MUST reset the waiting
    // timer to expire after current time plus DELETE_PERIOD.

    Enter_Method("datagramForwardHook");
    const IPv4Address& destAddr = datagram->getDestAddress();
    const IPv4Address& sourceAddr = datagram->getSrcAddress();
    IPv4Route *ipSource = routingTable->findBestMatchingRoute(sourceAddr);

    if (destAddr.isLimitedBroadcastAddress() || routingTable->isLocalAddress(destAddr) || destAddr.isMulticast()) {
        if (routingTable->isLocalAddress(destAddr) && ipSource && ipSource->getSource() == this)
            updateValidRouteLifeTime(ipSource->getGateway(), simTime() + activeRouteTimeout);

        return ACCEPT;
    }

    // TODO: IMPLEMENT: check if the datagram is a data packet or we take control packets as data packets

    IPv4Route *routeDest = routingTable->findBestMatchingRoute(destAddr);
    AODVRelRouteData *routeDestData = routeDest ? dynamic_cast<AODVRelRouteData *>(routeDest->getProtocolData()) : NULL;

    // Each time a route is used to forward a data packet, its Active Route
    // Lifetime field of the source, destination and the next hop on the
    // path to the destination is updated to be no less than the current
    // time plus ACTIVE_ROUTE_TIMEOUT

    updateValidRouteLifeTime(sourceAddr, simTime() + activeRouteTimeout);
    updateValidRouteLifeTime(destAddr, simTime() + activeRouteTimeout);

    if (routeDest && routeDest->getSource() == this)
        updateValidRouteLifeTime(routeDest->getGateway(), simTime() + activeRouteTimeout);

    // Since the route between each originator and destination pair is expected
    // to be symmetric, the Active Route Lifetime for the previous hop, along the
    // reverse path back to the IP source, is also updated to be no less than the
    // current time plus ACTIVE_ROUTE_TIMEOUT.

    if (ipSource && ipSource->getSource() == this)
        updateValidRouteLifeTime(ipSource->getGateway(), simTime() + activeRouteTimeout);

    EV_INFO << "We can't forward datagram because we have no active route for " << destAddr << endl;
    if (routeDest && routeDestData && !routeDestData->isActive()) {    // exists but is not active
        // A node initiates processing for a RERR message in three situations:
        // (ii)      if it gets a data packet destined to a node for which it
        //           does not have an active route and is not repairing (if
        //           using local repair)

        // TODO: check if it is not repairing (if using local repair)

        // 1. The destination sequence number of this routing entry, if it
        // exists and is valid, is incremented for cases (i) and (ii) above,
        // and copied from the incoming RERR in case (iii) above.

        if (routeDestData->hasValidDestNum())
            routeDestData->setDestSeqNum(routeDestData->getDestSeqNum() + 1);

        // 2. The entry is invalidated by marking the route entry as invalid <- it is invalid

        // 3. The Lifetime field is updated to current time plus DELETE_PERIOD.
        //    Before this time, the entry SHOULD NOT be deleted.
        routeDestData->setLifeTime(simTime() + deletePeriod);

        sendRERRWhenNoRouteToForward(destAddr);
    }
    else if (!routeDest || routeDest->getSource() != this) // doesn't exist at all
        sendRERRWhenNoRouteToForward(destAddr);

    return ACCEPT;
}

void AODVRelRouting::sendRERRWhenNoRouteToForward(const IPv4Address& unreachableAddr)
{
    if (rerrCount >= rerrRatelimit) {
        EV_WARN << "A node should not generate more than RERR_RATELIMIT RERR messages per second. Canceling sending RERR" << endl;
        return;
    }
    std::vector<UnreachableNode> unreachableNodes;
    UnreachableNode node;
    node.addr = unreachableAddr;

    IPv4Route *unreachableRoute = routingTable->findBestMatchingRoute(unreachableAddr);
    AODVRelRouteData *unreachableRouteData = unreachableRoute ? dynamic_cast<AODVRelRouteData *>(unreachableRoute->getProtocolData()) : NULL;

    if (unreachableRouteData && unreachableRouteData->hasValidDestNum())
        node.seqNum = unreachableRouteData->getDestSeqNum();
    else
        node.seqNum = 0;

    unreachableNodes.push_back(node);
    AODVRelRERR *rerr = createRERR(unreachableNodes);

    rerrCount++;
    EV_INFO << "Broadcasting Route Error message with TTL=1" << endl;
    sendAODVRelPacket(rerr, IPv4Address::ALLONES_ADDRESS, 1, jitterPar->doubleValue());    // TODO: unicast if there exists a route to the source
}

void AODVRelRouting::cancelRouteDiscovery(const IPv4Address& destAddr)
{
    ASSERT(hasOngoingRouteDiscovery(destAddr));
    std::multimap<IPv4Address, IPv4Datagram *>::iterator lt = targetAddressToDelayedPackets.lower_bound(destAddr);
    std::multimap<IPv4Address, IPv4Datagram *>::iterator ut = targetAddressToDelayedPackets.upper_bound(destAddr);
    for (std::multimap<IPv4Address, IPv4Datagram *>::iterator it = lt; it != ut; it++)
        networkProtocol->dropQueuedDatagram(const_cast<const IPv4Datagram *>(it->second));

    targetAddressToDelayedPackets.erase(lt, ut);
}

bool AODVRelRouting::updateValidRouteLifeTime(const IPv4Address& destAddr, simtime_t lifetime)
{
    IPv4Route *route = routingTable->findBestMatchingRoute(destAddr);
    if (route && route->getSource() == this) {
        AODVRelRouteData *routeData = check_and_cast<AODVRelRouteData *>(route->getProtocolData());
        if (routeData->isActive()) {
            simtime_t newLifeTime = std::max(routeData->getLifeTime(), lifetime);
            EV_DETAIL << "Updating " << route << " lifetime to " << newLifeTime << endl;
            routeData->setLifeTime(newLifeTime);
            return true;
        }
    }
    return false;
}

AODVRelRREPACK *AODVRelRouting::createRREPACK()
{
    AODVRelRREPACK *rrepACK = new AODVRelRREPACK("AODVRel-RREPACK");
    rrepACK->setPacketType(RREPACK);
    return rrepACK;
}

void AODVRelRouting::sendRREPACK(AODVRelRREPACK *rrepACK, const IPv4Address& destAddr)
{
    EV_INFO << "Sending Route Reply ACK to " << destAddr << endl;
    sendAODVRelPacket(rrepACK, destAddr, 100, 0);
}

void AODVRelRouting::handleRREPACK(AODVRelRREPACK *rrepACK, const IPv4Address& neighborAddr)
{
    // Note that the RREP-ACK packet does not contain any information about
    // which RREP it is acknowledging.  The time at which the RREP-ACK is
    // received will likely come just after the time when the RREP was sent
    // with the 'A' bit.
    ASSERT(rrepAckTimer->isScheduled());
    EV_INFO << "RREP-ACK arrived from " << neighborAddr << endl;

    IPv4Route *route = routingTable->findBestMatchingRoute(neighborAddr);
    if (route && route->getSource() == this) {
        EV_DETAIL << "Marking route " << route << " as active" << endl;
        AODVRelRouteData *routeData = check_and_cast<AODVRelRouteData *>(route->getProtocolData());
        routeData->setIsActive(true);
        cancelEvent(rrepAckTimer);
    }
}

void AODVRelRouting::handleRREPACKTimer()
{
    // when a node detects that its transmission of a RREP message has failed,
    // it remembers the next-hop of the failed RREP in a "blacklist" set.

    EV_INFO << "RREP-ACK didn't arrived within timeout. Adding " << failedNextHop << " to the blacklist" << endl;

    blacklist[failedNextHop] = simTime() + blacklistTimeout;    // lifetime

    if (!blacklistTimer->isScheduled())
        scheduleAt(simTime() + blacklistTimeout, blacklistTimer);
}

void AODVRelRouting::handleBlackListTimer()
{
    simtime_t nextTime = SimTime::getMaxTime();

    for (std::map<IPv4Address, simtime_t>::iterator it = blacklist.begin(); it != blacklist.end(); ) {
        std::map<IPv4Address, simtime_t>::iterator current = it++;

        // Nodes are removed from the blacklist set after a BLACKLIST_TIMEOUT period
        if (current->second <= simTime()) {
            EV_DETAIL << "Blacklist lifetime has expired for " << current->first << " removing it from the blacklisted addresses" << endl;
            blacklist.erase(current);
        }
        else if (nextTime > current->second)
            nextTime = current->second;
    }

    if (nextTime != SimTime::getMaxTime())
        scheduleAt(nextTime, blacklistTimer);
}

AODVRelRouting::~AODVRelRouting()
{
    clearState();
    delete helloMsgTimer;
    delete expungeTimer;
    delete counterTimer;
    delete rrepAckTimer;
    delete blacklistTimer;

    nb = NotificationBoardAccess().getIfExists(this);
    if (nb)
        nb->unsubscribe(this, NF_LINK_BREAK);
}

