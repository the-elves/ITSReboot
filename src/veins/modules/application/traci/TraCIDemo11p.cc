//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "IPv4Datagram.h"
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;
#include <iostream>
#include <ARP.h>
#include "Ieee802Ctrl_m.h"
#include "ARPPacket_m.h"

const simsignalwrap_t TraCIDemo11p::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

Define_Module(TraCIDemo11p);

void TraCIDemo11p::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobility = TraCIMobilityAccess().get(getParentModule());
		traci = mobility->getCommandInterface();
		traciVehicle = mobility->getVehicleCommandInterface();
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		sentMessage = false;
		lastDroveAt = simTime();
		findHost()->subscribe(parkingStateChangedSignal, this);
		isParking = false;
		sendWhileParking = par("sendWhileParking").boolValue();
	}
}

void TraCIDemo11p::onBeacon(WaveShortMessage* wsm) {
}

void TraCIDemo11p::onData(WaveShortMessage* wsm) {
//	findHost()->getDisplayString().updateWith("r=16,green");
//	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));
//	if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
//	if (!sentMessage) sendMessage(wsm->getWsmData());
//-------------------------------------------------------
    ////    memcpy(upperDatagram, copy->getWsmData(), (copy->getBitLength()-88)/8);
    //    IPv4Datagram *datagram = new IPv4Datagram( copy->getWsmData ());

    if(wsm == NULL)
            error("wsm error");
    cPacket *incomingMessage= check_and_cast<cPacket*>(wsm->getObject("mymsg"));
    IPv4Datagram *upperDatagram = dynamic_cast<IPv4Datagram*>(incomingMessage);
    if (upperDatagram == NULL){
        ARPPacket *upperARPPacket;
        upperARPPacket = dynamic_cast<ARPPacket*>(wsm->getObject("mymsg"))->dup();
        upperARPPacket->removeControlInfo();
        assert(upperARPPacket);
        cMsgPar *par = dynamic_cast<cMsgPar*> (wsm->getParList().get("ctrlinfo") );
        assert(par);
        assert(par->getObjectValue());
        Ieee802Ctrl *ctrlinfo =  (Ieee802Ctrl *)(par->getObjectValue());
        assert(ctrlinfo);
        // ctrlinfo = dynamic_cast<Ieee802Ctrl*>(attachedObjects->get(0));
        upperARPPacket->setControlInfo(ctrlinfo->dup());
        send(upperARPPacket,upperLayerOut);
        return;
    }
    assert(upperDatagram);
    send(incomingMessage->dup(),upperLayerOut);

}

void TraCIDemo11p::handleUpperMsg(cMessage *msg){
    EV<<"Forwarding message to lower layer";
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    cPacket *upperDatagram = dynamic_cast<IPv4Datagram*>(msg);
    WaveShortMessage* wsm;
    Ieee802Ctrl *ctrolinfo;
    ctrolinfo = NULL;
    if(upperDatagram == NULL){
        ARPPacket *upperARP;
        upperARP = dynamic_cast<ARPPacket*>(msg);
        assert(upperARP);
        ctrolinfo = (dynamic_cast<Ieee802Ctrl *>(upperARP->getControlInfo()) )->dup();
        assert(ctrolinfo);
        msg->setName("mymsg");
        wsm = prepareWSM("data", upperARP->getBitLength(), channel, dataPriority, -1,2);
        cMsgPar *par = new cMsgPar("ctrlinfo");
        par->setObjectValue((cOwnedObject*)ctrolinfo);
        wsm->addPar(par);
        wsm->addObject(msg);
    }
    else{
        msg->setName("mymsg");
        wsm = prepareWSM("data", upperDatagram->getBitLength(), channel, dataPriority, -1,2);
        wsm->addObject(msg);
    }

    sendWSM(wsm->dup());

//    std::cout<<"ppp "<<((IPv4Datagram*)wsm->getWsmData())->getDestAddress().str()<<endl;
//    std::cout<<"ppp "<<((IPv4Datagram*)upperDatagram)->getDestAddress().str()<<endl;

}

void TraCIDemo11p::sendMessage(std::string blockedRoadId) {
	sentMessage = true;

	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}
void TraCIDemo11p::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj) {
	Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
	else if (signalID == parkingStateChangedSignal) {
		handleParkingUpdate(obj);
	}
}
void TraCIDemo11p::handleParkingUpdate(cObject* obj) {
	isParking = mobility->getParkingState();
	if (sendWhileParking == false) {
		if (isParking == true) {
			(FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
		}
		else {
			Coord pos = mobility->getCurrentPosition();
			(FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
		}
	}
}
void TraCIDemo11p::handlePositionUpdate(cObject* obj) {
	BaseWaveApplLayer::handlePositionUpdate(obj);

	// stopped for for at least 10s?
	if (mobility->getSpeed() < 1) {
		if (simTime() - lastDroveAt >= 10) {
			findHost()->getDisplayString().updateWith("r=16,red");
			if (!sentMessage) sendMessage(mobility->getRoadId());
		}
	}
	else {
		lastDroveAt = simTime();
	}
}
void TraCIDemo11p::sendWSM(WaveShortMessage* wsm) {
	if (isParking && !sendWhileParking) return;
	sendDelayedDown(wsm,individualOffset);
}
