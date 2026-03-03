/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * uas-vanet-basic.cc
 *
 * A basic NS-3.35 simulation of Flying Vehicle Communication (UAS-based VANET).
 *
 * Scenario Overview:
 * -----------------
 *   - 2 UAV (Unmanned Aerial Vehicle) nodes flying at altitude (Z=100m)
 *   - 3 Ground Vehicle nodes on the ground (Z=0m)
 *   - All nodes communicate using IEEE 802.11p (WAVE/DSRC) — the vehicular standard
 *   - AODV (Ad-hoc On-demand Distance Vector) routing for multi-hop packet delivery
 *   - A simple UDP echo application: a ground vehicle sends packets to a UAV
 *   - FlowMonitor collects performance metrics (delay, throughput, packet loss)
 *
 * How to run (from your ns-3.35 root directory):
 *   1. Copy this file to:  ns-3.35/scratch/uas-vanet-basic.cc
 *      (or ns-3.35/contrib/your-module/examples/ if using a contrib module)
 *   2. Build:              ./waf build
 *   3. Run:                ./waf --run uas-vanet-basic
 *
 * Key lesson — Propagation Model Matters!
 *   The default YansWifiChannelHelper::Default() uses LogDistance with exponent=3,
 *   which attenuates signals too much for air-to-ground (141m → -91 dBm, below
 *   receiver sensitivity). We use FriisPropagationLossModel at 5.9 GHz instead,
 *   giving -71 dBm at the same distance — well above the -82 dBm threshold.
 *
 * Expected output:
 *   - Console log showing UDP echo packets sent/received
 *   - FlowMonitor XML report saved to "uas-vanet-flowmon.xml"
 *   - NetAnim XML file saved to "uas-vanet-animation.xml" (open with NetAnim)
 */

// ============================================================================
// STEP 0: Include necessary NS-3 module headers
// ============================================================================

// Core NS-3 modules
#include "ns3/core-module.h"          // Simulator, CommandLine, logging macros, etc.
#include "ns3/network-module.h"       // Node, NodeContainer, NetDeviceContainer, Packet
#include "ns3/internet-module.h"      // InternetStackHelper, Ipv4AddressHelper, AODV
#include "ns3/mobility-module.h"      // MobilityHelper, ConstantVelocityMobilityModel, etc.
#include "ns3/applications-module.h"  // UdpEchoServer/Client helpers
#include "ns3/flow-monitor-module.h"  // FlowMonitorHelper for performance analysis
#include "ns3/netanim-module.h"       // AnimationInterface for visualization
#include "ns3/aodv-module.h"

// WAVE / 802.11p specific modules (vehicular communication)
#include "ns3/wave-module.h"          // Wifi80211pHelper, NqosWaveMacHelper
#include "ns3/wifi-module.h"          // YansWifiPhyHelper, YansWifiChannelHelper
// Note: In ns-3.35, the 'wave' module provides 802.11p support for VANET.
// The Wifi80211pHelper creates a WifiNetDevice with OcbWifiMac (Outside the
// Context of a BSS), which means no association/authentication is needed —
// perfect for fast vehicle-to-vehicle communication.

// Standard library
#include <iostream>

// Use the ns3 namespace so we don't have to prefix everything with "ns3::"
using namespace ns3;

// Enable a logging component for this file (use NS_LOG_* macros below)
NS_LOG_COMPONENT_DEFINE ("UasVanetBasic");

int
main (int argc, char *argv[])
{
  // ============================================================================
  // STEP 1: Simulation Parameters (configurable via command line)
  // ============================================================================
  //
  // These define the basic scenario. You can override them from the command line:
  //   ./waf --run "uas-vanet-basic --numUav=4 --simTime=30"

  uint32_t numUav     = 2;      // Number of UAV (flying) nodes
  uint32_t numVehicle = 3;      // Number of ground vehicle nodes
  double   simTime    = 20.0;   // Total simulation time in seconds
  bool     verbose    = true;   // Enable application-level logging

  // CommandLine parser: allows overriding parameters from the terminal
  CommandLine cmd;
  cmd.AddValue ("numUav",     "Number of UAV nodes",           numUav);
  cmd.AddValue ("numVehicle", "Number of ground vehicle nodes", numVehicle);
  cmd.AddValue ("simTime",    "Simulation time (seconds)",     simTime);
  cmd.AddValue ("verbose",    "Enable UDP echo logging",       verbose);
  cmd.Parse (argc, argv);

  // If verbose mode is on, enable the built-in logging of UDP echo apps.
  // This will print messages like "Sent 1024 bytes to ..." and "Received 1024 bytes from ..."
  if (verbose)
    {
      LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
      LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

  // ============================================================================
  // STEP 2: Create Nodes
  // ============================================================================
  //
  // In NS-3, a "Node" is the basic computing element — like a blank computer.
  // We create two groups: UAVs (flying) and Vehicles (ground).
  // We also combine them into one container for shared operations (WiFi, IP).

  NodeContainer uavNodes;
  uavNodes.Create (numUav);        // Create UAV nodes

  NodeContainer vehicleNodes;
  vehicleNodes.Create (numVehicle); // Create ground vehicle nodes

  // Combine all nodes into one container (needed for installing WiFi on all)
  NodeContainer allNodes;
  allNodes.Add (uavNodes);
  allNodes.Add (vehicleNodes);

  std::cout << "=== UAS-VANET Simulation ===" << std::endl;
  std::cout << "UAV nodes:     " << numUav << std::endl;
  std::cout << "Vehicle nodes: " << numVehicle << std::endl;
  std::cout << "Total nodes:   " << allNodes.GetN () << std::endl;

  // ============================================================================
  // STEP 3: Set Up Mobility Models (Position & Movement)
  // ============================================================================
  //
  // This is critical for a UAS-VANET simulation:
  //   - UAVs fly at altitude (e.g., Z = 100m) with constant velocity
  //   - Ground vehicles move on the ground (Z = 0m)
  //
  // We use ListPositionAllocator to set exact initial positions, then
  // ConstantVelocityMobilityModel so each node moves at a fixed speed/direction.
  //
  // In a more advanced simulation, you could use:
  //   - GaussMarkovMobilityModel for UAV random flight patterns
  //   - Ns2MobilityHelper to replay SUMO traces for realistic vehicle movement
  //   - WaypointMobilityModel for scripted UAV flight paths

  // --- UAV Mobility ---
  MobilityHelper uavMobility;

  // Set initial positions for UAVs using a list allocator
  Ptr<ListPositionAllocator> uavPositionAlloc = CreateObject<ListPositionAllocator> ();
  // UAV 0: starts at (0, 0, 100) — hovering at 100m altitude
  uavPositionAlloc->Add (Vector (0.0, 0.0, 100.0));
  // UAV 1: starts at (200, 0, 100) — 200m away, same altitude
  uavPositionAlloc->Add (Vector (200.0, 0.0, 100.0));
  // (If you increase numUav, add more positions here or use a different allocator)

  uavMobility.SetPositionAllocator (uavPositionAlloc);
  // ConstantVelocityMobilityModel: each node moves at a fixed velocity vector.
  // We set the velocity after installation (the helper only sets initial position).
  uavMobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  uavMobility.Install (uavNodes);

  // Now set UAV velocities (must be done AFTER Install)
  // UAV 0 flies eastward at 20 m/s
  uavNodes.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (
      Vector (20.0, 0.0, 0.0));
  // UAV 1 flies westward at 15 m/s
  uavNodes.Get (1)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (
      Vector (-15.0, 0.0, 0.0));

  // --- Ground Vehicle Mobility ---
  MobilityHelper vehicleMobility;

  Ptr<ListPositionAllocator> vehiclePositionAlloc = CreateObject<ListPositionAllocator> ();
  // Vehicles on the ground (Z=0), spread along the X axis
  vehiclePositionAlloc->Add (Vector (50.0,  0.0, 0.0));   // Vehicle 0
  vehiclePositionAlloc->Add (Vector (100.0, 0.0, 0.0));   // Vehicle 1
  vehiclePositionAlloc->Add (Vector (150.0, 0.0, 0.0));   // Vehicle 2

  vehicleMobility.SetPositionAllocator (vehiclePositionAlloc);
  // Vehicles also use ConstantVelocityMobilityModel
  vehicleMobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  vehicleMobility.Install (vehicleNodes);

  // Set vehicle velocities: all moving eastward at 10 m/s (like cars on a highway)
  for (uint32_t i = 0; i < vehicleNodes.GetN (); ++i)
    {
      vehicleNodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (
          Vector (10.0, 0.0, 0.0));
    }

  std::cout << "Mobility models installed." << std::endl;

  // ============================================================================
  // STEP 4: Set Up 802.11p (WAVE/DSRC) Wireless Communication
  // ============================================================================
  //
  // IEEE 802.11p is the wireless standard for vehicular communication:
  //   - Operates in the 5.9 GHz band with 10 MHz channel bandwidth
  //   - Uses OFDM (same as 802.11a but with 10 MHz channels)
  //   - No association/authentication needed (OCB mode = Outside BSS Context)
  //   - Designed for fast, low-latency vehicle-to-vehicle (V2V) communication
  //
  // In NS-3, we use three helpers to set this up:
  //   1. YansWifiChannelHelper  — creates the wireless channel (propagation model)
  //   2. YansWifiPhyHelper      — configures the physical layer (frequency, power)
  //   3. NqosWaveMacHelper      — configures the MAC layer (no QoS, OCB mode)
  //   4. Wifi80211pHelper       — ties MAC and PHY together for 802.11p

  // --- PHY Layer (Physical) ---
  //
  // IMPORTANT: We do NOT use YansWifiChannelHelper::Default() here!
  // The default uses LogDistancePropagationLossModel with exponent=3 and
  // reference loss=46.67 dB (calibrated for 5.15 GHz WiFi). This causes
  // too much signal attenuation for our air-to-ground scenario:
  //   At 141m (3D distance UAV↔Vehicle): path loss ≈ 111 dB
  //   Received power = 20 dBm - 111 dB = -91 dBm  ← BELOW receiver sensitivity!
  //
  // Instead, we use FriisPropagationLossModel (free-space) at 5.9 GHz,
  // which is more appropriate for UAV-to-ground with clear line-of-sight:
  //   At 141m: path loss ≈ 91 dB
  //   Received power = 20 dBm - 91 dB = -71 dBm   ← ABOVE receiver sensitivity!
  //
  // For even more realism, you can chain a NakagamiPropagationLossModel
  // after Friis to add fast fading effects (see commented code below).

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel",
                                  "Frequency", DoubleValue (5.9e9));  // 5.9 GHz for 802.11p
  // Optional: add Nakagami fading for more realistic air-to-ground channel
  // Uncomment the next line to enable stochastic fading:
  // wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");

  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Increase transmit power for UAV-to-ground communication range
  // 802.11p allows higher power than standard WiFi (up to 33 dBm / 2W in US)
  wifiPhy.Set ("TxPowerStart", DoubleValue (20.0));  // Min transmit power (dBm)
  wifiPhy.Set ("TxPowerEnd",   DoubleValue (20.0));  // Max transmit power (dBm)

  // --- MAC Layer ---
  // NqosWaveMacHelper::Default() creates an OcbWifiMac (no BSS, no association)
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();

  // --- Combine into 802.11p Helper ---
  // Wifi80211pHelper::Default() internally sets the PHY standard to
  // 802.11 with 10 MHz channel width (the vehicular standard)
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  // Optional: set a fixed data rate (OfdmRate6MbpsBW10MHz is common for VANET)
  // If not set, the default rate manager will be used
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",    StringValue ("OfdmRate6MbpsBW10MHz"),
                                      "ControlMode", StringValue ("OfdmRate6MbpsBW10MHz"),
                                      "NonUnicastMode", StringValue ("OfdmRate6MbpsBW10MHz"));

  // Install 802.11p on ALL nodes (both UAVs and ground vehicles)
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, allNodes);

  std::cout << "802.11p wireless devices installed on all nodes." << std::endl;

  // ============================================================================
  // STEP 5: Install Internet Stack with AODV Routing
  // ============================================================================
  //
  // Now we need to give each node an IP networking stack (like installing an OS
  // with TCP/IP support). We also choose AODV as the routing protocol because:
  //   - It's designed for mobile ad-hoc networks (MANETs)
  //   - It discovers routes "on demand" — only when a node wants to send data
  //   - It adapts to topology changes (important when UAVs and vehicles move)
  //
  // Alternative routing protocols you could try:
  //   - OlsrHelper (OLSR): proactive, maintains routes continuously
  //   - DsdvHelper (DSDV): proactive, table-driven
  //   - DsrHelper  (DSR):  reactive, source routing

  AodvHelper aodv;
  // Optional: you can tune AODV parameters, for example:
  // aodv.Set ("AllowedHelloLoss", UintegerValue (2));
  // aodv.Set ("HelloInterval", TimeValue (Seconds (1)));

  InternetStackHelper internet;
  internet.SetRoutingHelper (aodv);  // Use AODV as the routing protocol
  internet.Install (allNodes);       // Install IP stack on all nodes

  // Assign IP addresses to all wireless devices
  // All nodes will be in the 10.1.1.0/24 subnet
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign (devices);

  // Print assigned IP addresses for reference
  std::cout << "\nIP Address Assignments:" << std::endl;
  for (uint32_t i = 0; i < allNodes.GetN (); ++i)
    {
      std::string nodeType = (i < numUav) ? "UAV" : "Vehicle";
      uint32_t localIdx = (i < numUav) ? i : (i - numUav);
      std::cout << "  " << nodeType << " " << localIdx
                << " -> " << interfaces.GetAddress (i) << std::endl;
    }

  // ============================================================================
  // STEP 6: Set Up Applications (UDP Echo)
  // ============================================================================
  //
  // We create a simple client-server application:
  //   - Server: UAV 0 (node 0) — listens for incoming UDP packets
  //   - Client: Vehicle 1 (node numUav+1) — sends UDP packets to the UAV server
  //
  // The UDP Echo protocol: client sends a packet, server echoes it back.
  // This lets us verify two-way communication between air and ground.
  //
  // For more advanced traffic, you could use:
  //   - OnOffHelper: generates traffic at a specified rate (CBR or bursty)
  //   - BulkSendHelper: TCP bulk data transfer
  //   - Custom applications for BSM (Basic Safety Messages)

  uint16_t port = 9;  // UDP port number

  // --- Server on UAV 0 ---
  UdpEchoServerHelper echoServer (port);
  ApplicationContainer serverApps = echoServer.Install (uavNodes.Get (0));
  serverApps.Start (Seconds (1.0));     // Server starts listening at t=1s
  serverApps.Stop (Seconds (simTime));  // Server stops at end of simulation

  // --- Client on Vehicle 1 ---
  // The client sends packets to UAV 0's IP address
  UdpEchoClientHelper echoClient (interfaces.GetAddress (0), port);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (10));          // Send 10 packets
  echoClient.SetAttribute ("Interval",   TimeValue (Seconds (1.0)));   // 1 packet per second
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));        // 1024 bytes each

  // Install client on Vehicle 1 (index = numUav + 1 in allNodes)
  ApplicationContainer clientApps = echoClient.Install (vehicleNodes.Get (1));
  clientApps.Start (Seconds (2.0));     // Client starts sending at t=2s
  clientApps.Stop (Seconds (simTime));  // Client stops at end of simulation

  std::cout << "\nApplication setup:" << std::endl;
  std::cout << "  Server: UAV 0     (" << interfaces.GetAddress (0) << ":" << port << ")" << std::endl;
  std::cout << "  Client: Vehicle 1 (" << interfaces.GetAddress (numUav + 1) << ")" << std::endl;
  std::cout << "  Sending " << 10 << " packets of 1024 bytes, 1 per second\n" << std::endl;

  // ============================================================================
  // STEP 7: Install FlowMonitor (Performance Metrics)
  // ============================================================================
  //
  // FlowMonitor tracks per-flow statistics including:
  //   - Packet delivery ratio (PDR)
  //   - End-to-end delay
  //   - Jitter (delay variation)
  //   - Throughput
  //   - Lost packets
  //
  // Results are saved to an XML file that you can parse for analysis.

  FlowMonitorHelper flowmonHelper;
  Ptr<FlowMonitor> flowMonitor = flowmonHelper.InstallAll ();

  // ============================================================================
  // STEP 8: Enable NetAnim Visualization (Optional)
  // ============================================================================
  //
  // NetAnim is a GUI tool that comes with NS-3. It reads an XML file and
  // shows an animation of nodes moving and packets being transmitted.
  //
  // To view: open NetAnim (in ns-3.35/netanim/) and load the XML file.
  //
  // We also set node descriptions and colors to distinguish UAVs from vehicles.

  AnimationInterface anim ("uas-vanet-animation.xml");

  // Set node descriptions and sizes for visualization
  for (uint32_t i = 0; i < uavNodes.GetN (); ++i)
    {
      anim.UpdateNodeDescription (uavNodes.Get (i), "UAV-" + std::to_string (i));
      anim.UpdateNodeColor (uavNodes.Get (i), 255, 0, 0);  // Red for UAVs
      anim.UpdateNodeSize (uavNodes.Get (i)->GetId (), 5.0, 5.0);
    }
  for (uint32_t i = 0; i < vehicleNodes.GetN (); ++i)
    {
      anim.UpdateNodeDescription (vehicleNodes.Get (i), "VEH-" + std::to_string (i));
      anim.UpdateNodeColor (vehicleNodes.Get (i), 0, 0, 255);  // Blue for vehicles
      anim.UpdateNodeSize (vehicleNodes.Get (i)->GetId (), 3.0, 3.0);
    }

  // ============================================================================
  // STEP 9: Run the Simulation
  // ============================================================================

  std::cout << "Starting simulation for " << simTime << " seconds..." << std::endl;

  Simulator::Stop (Seconds (simTime));  // Set simulation end time
  Simulator::Run ();                    // Run the event-driven simulation

  // ============================================================================
  // STEP 10: Collect and Print Results
  // ============================================================================

  // Save detailed FlowMonitor results to XML
  flowMonitor->SerializeToXmlFile ("uas-vanet-flowmon.xml", true, true);

  // Print a summary of flow statistics to the console
  std::cout << "\n=== Simulation Complete ===" << std::endl;
  std::cout << "FlowMonitor results saved to: uas-vanet-flowmon.xml" << std::endl;
  std::cout << "NetAnim animation saved to:   uas-vanet-animation.xml" << std::endl;

  // Parse FlowMonitor stats for a quick console summary
  Ptr<Ipv4FlowClassifier> classifier =
      DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats ();

  std::cout << "\n--- Per-Flow Statistics ---" << std::endl;
  for (auto it = stats.begin (); it != stats.end (); ++it)
    {
      Ipv4FlowClassifier::FiveTuple ft = classifier->FindFlow (it->first);
      std::cout << "Flow " << it->first << " ("
                << ft.sourceAddress << " -> " << ft.destinationAddress << ")" << std::endl;
      std::cout << "  Tx Packets:   " << it->second.txPackets << std::endl;
      std::cout << "  Rx Packets:   " << it->second.rxPackets << std::endl;
      std::cout << "  Lost Packets: " << it->second.lostPackets << std::endl;
      if (it->second.rxPackets > 0)
        {
          // Average delay in milliseconds
          double avgDelay = it->second.delaySum.GetMilliSeconds ()
                            / (double) it->second.rxPackets;
          std::cout << "  Avg Delay:    " << avgDelay << " ms" << std::endl;
          // Throughput in kbps (guard against zero time interval)
          double duration = it->second.timeLastRxPacket.GetSeconds ()
                            - it->second.timeFirstTxPacket.GetSeconds ();
          if (duration > 0)
            {
              double throughput = it->second.rxBytes * 8.0 / duration / 1000.0;
              std::cout << "  Throughput:   " << throughput << " kbps" << std::endl;
            }
          // Packet Delivery Ratio
          double pdr = (double) it->second.rxPackets / (double) it->second.txPackets * 100.0;
          std::cout << "  PDR:          " << pdr << " %" << std::endl;
        }
      std::cout << std::endl;
    }

  // Clean up the simulator (free all allocated objects)
  Simulator::Destroy ();

  return 0;
}