/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Tim Schubert <ns-3-leo@timschubert.net>
 */

#include <iostream>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/leo-module.h"
#include "ns3/network-module.h"
#include "ns3/aodv-module.h"
#include "ns3/udp-server.h"

using namespace ns3;

// Delay tracking
map<int, double> TxTimes;
map<int, double> delay;

// SNR tracking
std::map<int, double> snrValues;
int snrCount = 0;
NodeContainer g_satellites;
NodeContainer g_users;
Ptr<PropagationLossModel> g_propLoss = nullptr;

static void EchoTxRx (std::string context, const Ptr< const Packet > packet, const TcpHeader &header, const Ptr< const TcpSocketBase > socket)
{
    // TODO: Calculate end-to-end delay
    // Hint1: Parse the packet (you may refer context.find())
    double time = Simulator::Now ().GetSeconds();
    uint64_t uid = packet->GetUid();
    if (context.find("/Tx") != std::string::npos) {
        // std::cout << "This is transmitting\n";
        TxTimes[uid] = time;
    }
    else if (context.find("/Rx") != std::string::npos) { 
        // std::cout << "This is receiving\n";
        if (TxTimes.find(uid) != TxTimes.end()) {
            delay[uid] = time - TxTimes[uid];
        }

        // Calculatin SNR
        uint32_t nodeId = socket->GetNode()->GetId();

        if (nodeId >= g_satellites.GetN()) {
            Ptr<Node> rxNode = socket->GetNode();
            Ptr<MobilityModel> rxMob = rxNode->GetObject<MobilityModel>();

            double maxRxPower = -1000.0;
            for (uint32_t i = 0; i < g_satellites.GetN(); i ++) {
                Ptr<Node> satNode = g_satellites.Get(i);
                Ptr<MobilityModel> satMob = satNode->GetObject<MobilityModel>();
                Ptr<MockNetDevice> satDev = DynamicCast<MockNetDevice>(satNode->GetDevice(0));

                double txPower = satDev->GetTxPower();
                double rxPower = g_propLoss->CalcRxPower(txPower, satMob, rxMob);

                if (rxPower > maxRxPower) maxRxPower = rxPower;
            }

            const double NOISE_DB = -90.0;
            double snr_dB = maxRxPower - NOISE_DB;

            snrValues[snrCount++] = snr_dB;

            // std::cout << "RxPower: " << maxRxPower << " dBm, SNR: " << snr_dB << " dB\n";
        }
    }
    // std::cout << Simulator::Now () << ":" << context << ":" << uid << ":" << socket->GetNode () << ":" << header.GetSequenceNumber () << std::endl;

}

void connect ()
{
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Tx", MakeCallback (&EchoTxRx));
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Rx", MakeCallback (&EchoTxRx));
}

void initial_position (const NodeContainer &satellites, int sz)
{
    for(int i = 0; i < min((int)satellites.GetN(), sz); i++){
        // Get satellite position
        Vector pos = satellites.Get(i)->GetObject<MobilityModel>()->GetPosition();
        // Convert position to latitude & longtitude
        double r = sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z);
        double lat = asin(pos.z / r) * 180.0 / M_PI;
        double longit = atan2(pos.y, pos.x) * 180 / M_PI;
        cout << "Satellite " << i << " latitude = " << lat << ", longtitude = " << longit << endl;
    }
}

void PrintSatellitesPosition() {
    Vector pos = g_satellites.Get(0)->GetObject<MobilityModel>()->GetPosition();
    double dist = sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z);
    std::cout << "[DEBUG] Time=" << Simulator::Now().GetSeconds() << "s, Sat0 distance from Earth center=" << dist/1000 << " km\n"; 
}

NS_LOG_COMPONENT_DEFINE ("LeoBulkSendTracingExample");

int main (int argc, char *argv[])
{

    CommandLine cmd;
    std::string orbitFile;
    std::string traceFile;
    LeoLatLong source (6.06692, 73.0213);
    LeoLatLong destination (7.06692, 74.0213);
    std::string islRate = "2Gbps";
    std::string constellation = "TelesatGateway";
    uint16_t port = 9;
    uint32_t latGws = 20;
    uint32_t lonGws = 20;
    double duration = 100;
    bool islEnabled = false;
    bool pcap = false;
    uint64_t ttlThresh = 0;
    std::string routingProto = "aodv";

    cmd.AddValue("orbitFile", "CSV file with orbit parameters", orbitFile);
    cmd.AddValue("traceFile", "CSV file to store mobility trace in", traceFile);
    cmd.AddValue("precision", "ns3::LeoCircularOrbitMobilityModel::Precision");
    cmd.AddValue("duration", "Duration of the simulation in seconds", duration);
    cmd.AddValue("source", "Traffic source", source);
    cmd.AddValue("destination", "Traffic destination", destination);
    cmd.AddValue("islRate", "ns3::MockNetDevice::DataRate");
    cmd.AddValue("constellation", "LEO constellation link settings name", constellation);
    cmd.AddValue("routing", "Routing protocol", routingProto);
    cmd.AddValue("islEnabled", "Enable inter-satellite links", islEnabled);
    cmd.AddValue("latGws", "Latitudal rows of gateways", latGws);
    cmd.AddValue("lonGws", "Longitudinal rows of gateways", lonGws);
    cmd.AddValue("ttlThresh", "TTL threshold", ttlThresh);
    cmd.AddValue("destOnly", "ns3::aodv::RoutingProtocol::DestinationOnly");
    cmd.AddValue("routeTimeout", "ns3::aodv::RoutingProtocol::ActiveRouteTimeout");
    cmd.AddValue("pcap", "Enable packet capture", pcap);
    cmd.Parse (argc, argv);

    std::streambuf *coutbuf = std::cout.rdbuf();
    // redirect cout if traceFile
    std::ofstream out;
    out.open (traceFile);
    if (out.is_open ())
    {
        std::cout.rdbuf(out.rdbuf());
    }

    LeoOrbitNodeHelper orbit;
    NodeContainer satellites;
    if (!orbitFile.empty())
    {
        satellites = orbit.Install (orbitFile);
    }
    else
    {
        satellites = orbit.Install ({ LeoOrbit (1200, 20, 5, 5) });
    }

    LeoGndNodeHelper ground;
    NodeContainer users = ground.Install (source, destination);

    g_satellites = satellites;
    g_users = users;

    LeoChannelHelper utCh;
    utCh.SetConstellation (constellation);
    utCh.SetGndDeviceAttribute("DataRate", StringValue("8kbps"));
    NetDeviceContainer utNet = utCh.Install (satellites, users);

    // Get Propagation Loss Model
    if(utNet.GetN() > 0) {
        Ptr<Channel> channel = utNet.Get(0)->GetChannel();
        if (channel) {
            Ptr<MockChannel> mockChannel = DynamicCast<MockChannel>(channel);
            if (mockChannel) {
                g_propLoss = mockChannel->GetPropagationLoss();
                std::cerr << "PropagationLossModel obtained successfully" << std::endl;
            }
        }
    }
    initial_position(satellites, 5);

    InternetStackHelper stack;
    AodvHelper aodv;
    aodv.Set ("EnableHello", BooleanValue (false));
    //aodv.Set ("HelloInterval", TimeValue (Seconds (10)));
    if (ttlThresh != 0)
    {
        aodv.Set ("TtlThreshold", UintegerValue (ttlThresh));
        aodv.Set ("NetDiameter", UintegerValue (2*ttlThresh));
    }
    stack.SetRoutingHelper (aodv);

    // Install internet stack on nodes
    stack.Install (satellites);
    stack.Install (users);

    Ipv4AddressHelper ipv4;

    ipv4.SetBase ("10.1.0.0", "255.255.0.0");
    ipv4.Assign (utNet);

    if (islEnabled)
    {
        std::cerr << "ISL enabled" << std::endl;
        IslHelper islCh;
        NetDeviceContainer islNet = islCh.Install (satellites);
        ipv4.SetBase ("10.2.0.0", "255.255.0.0");
        ipv4.Assign (islNet);
    }

    Ipv4Address remote = users.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
    BulkSendHelper sender ("ns3::TcpSocketFactory",
            InetSocketAddress (remote, port));
    // Set the amount of data to send in bytes.  Zero is unlimited.
    sender.SetAttribute ("MaxBytes", UintegerValue (1024));
    sender.SetAttribute ("SendSize", UintegerValue (512));
    ApplicationContainer sourceApps = sender.Install (users.Get (0));
    sourceApps.Start (Seconds (0.0));

    //
    // Create a PacketSinkApplication and install it on node 1
    //
    PacketSinkHelper sink ("ns3::TcpSocketFactory",
            InetSocketAddress (Ipv4Address::GetAny (), port));
    ApplicationContainer sinkApps = sink.Install (users.Get (1));
    sinkApps.Start (Seconds (0.0));

    // Fix segmentation fault
    Simulator::Schedule(Seconds(1e-7), &connect);
    for (int t = 0; t <= 100; t += 10) Simulator::Schedule(Seconds(t), &PrintSatellitesPosition);

    //
    // Set up tracing if enabled
    //
    if (pcap)
    {
        AsciiTraceHelper ascii;
        utCh.EnableAsciiAll (ascii.CreateFileStream ("tcp-bulk-send.tr"));
        utCh.EnablePcapAll ("tcp-bulk-send", false);
    }

    std::cerr << "LOCAL =" << users.Get (0)->GetId () << std::endl;
    std::cerr << "REMOTE=" << users.Get (1)->GetId () << ",addr=" << Ipv4Address::ConvertFrom (remote) << std::endl;

    NS_LOG_INFO ("Run Simulation.");
    Simulator::Stop (Seconds (duration));
    Simulator::Run ();
    Simulator::Destroy ();
    NS_LOG_INFO ("Done.");

    Ptr<PacketSink> sink1 = DynamicCast<PacketSink> (sinkApps.Get (0));
    std::cout << users.Get (0)->GetId () << ":" << users.Get (1)->GetId () << ": " << sink1->GetTotalRx () << std::endl;
    // 25:26: 1024
    // TODO: Output End-to-end Delay
    double avg_delay = 0;
    double nums = 0;
    for(auto &[seq, t]: delay){
        avg_delay += delay[seq];
        nums += 1;
    }
    cout << "Packet average end-to-end delay is " << (avg_delay / nums) << "s" << endl;

    std::cout << "\n========== SNR Statistics ==========\n";
    double avg_snr = 0.0;
    for (auto &[id, snr]: snrValues) {
        avg_snr += snr;
        cout << "Packet " << id << " SNR: " << snr << " dB\n";
    }
    if (snrCount > 0) cout << "Average Snr: " << avg_snr / snrCount << "dB\n";

    out.close ();
    std::cout.rdbuf(coutbuf);

    return 0;
}
