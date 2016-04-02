//
// Copyright (C) 2008 Christoph Sommer <christoph.sommer@informatik.uni-erlangen.de>
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

package org.car2x.veins.nodes;

import inet.applications.udpapp.UDPBasicApp;
import inet.base.NotificationBoard;
import inet.networklayer.autorouting.ipv4.HostAutoConfigurator;
import inet.networklayer.autorouting.ipv4.IPv4NodeConfigurator;
import inet.networklayer.common.InterfaceTable;
import inet.networklayer.ipv4.IGMPv2;
import inet.networklayer.ipv4.IPv4;
import inet.networklayer.ipv4.RoutingTable;
import inet.networklayer.routing.aodv.AODVRouting;
import inet.nodes.inet.NetworkLayer;
import inet.transport.IUDP;
import inet.transport.udp.UDP;
import org.car2x.veins.base.modules.*;
import org.car2x.veins.modules.application.traci.TraCIDemo11p;
import org.car2x.veins.modules.nic.Nic80211p;

module Car
{
    parameters:
        string applType; //type of the application layer
        string nicType = default("Nic80211p"); // type of network interface card
        string veinsmobilityType; //type of the mobility module
        networkLayer.configurator.networkConfiguratorModule = "";
        @display("bgb=480,448");
        @node;
    gates:
        input veinsradioIn; // gate for sendDirect
    submodules:

//         iPv4NodeConfigurator: IPv4NodeConfigurator {
//            @display("p=410,398");
//        }

        hostAutoConfigurator: HostAutoConfigurator {
            parameters:
                @display("p=422,403");
                interfaces = "nic";
        }
        veinsmobility: <veinsmobilityType> like org.car2x.veins.base.modules.IMobility {
            parameters:
                @display("p=422,329;i=block/cogwheel");
        }
        appl: TraCIDemo11p {
            @display("p=73,321");
        }
        networkLayer: NetworkLayer {
            @display("p=72,232");
        }



        routingTable: RoutingTable {
            @display("p=422,254");
        }




        aodvRouting: AODVRouting {
            @display("p=216,231;b=49,43");
        }

        interfaceTable: InterfaceTable {
            @display("p=422,185");
        }

        nic: <nicType> like org.car2x.veins.modules.nic.INic80211p {
            parameters:
                @display("p=72,395");
        }

        igmPv2: IGMPv2 {
            @display("p=136,168");
        }
        notificationBoard: NotificationBoard {
            @display("p=422,107");
        }




        udpBasicApp: UDPBasicApp {
            parameters:
                @display("p=76,33");
                destAddresses = "10.0.0.45";
                destPort = 5555;
                messageLength = 100B;
                sendInterval = 1s;
        }
        udp: UDP {
            @display("p=72,120");
        }
    connections allowunconnected:
        veinsradioIn --> nic.radioIn;
        appl.lowerControlOut --> nic.upperControlIn;
        appl.lowerLayerOut --> nic.upperLayerIn;
        nic.upperControlOut --> appl.lowerControlIn;
        nic.upperLayerOut --> appl.lowerLayerIn;
        networkLayer.transportOut++ --> aodvRouting.ipIn;
        aodvRouting.ipOut --> networkLayer.transportIn++;
        networkLayer.ifOut++ --> appl.upperLayerIn;
        appl.upperLayerOut --> networkLayer.ifIn++;
        igmPv2.routerOut --> networkLayer.igmpIn;
        networkLayer.igmpOut --> igmPv2.routerIn;
        udpBasicApp.udpOut --> udp.appIn++;
        udp.appOut++ --> udpBasicApp.udpIn;
        udp.ipOut --> networkLayer.transportIn++;
        networkLayer.transportOut++ --> udp.ipIn;
}
