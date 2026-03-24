/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * uav-to-leo.cc
 *
 * Step 2: Establish a DIRECT communication link between the main UAV
 * (cluster head) and a LEO satellite for large-volume data transmission.
 *
 * Architecture:
 *
 *   [UAV Cluster Head]  ── LEO channel (uplink) ──>  [LEO Satellite]
 *        (sender)                                       (receiver)
 *        GND device                                     SAT device
 *        BulkSend                                       PacketSink
 *
 * This is a single-hop scenario: the UAV sends data directly to whichever
 * LEO satellite is visible (within the elevation-angle cutoff).
 *
 * Key design decisions:
 *   - The UAV is treated as a "mobile ground station" by the ns-3-leo module.
 *     LeoMockChannel only permits GND <-> SAT transmission.
 *   - LeoPropagationLossModel handles FSPL, atmospheric loss, elevation-angle
 *     cutoff, and link margin — parameterized via SetConstellation().
 *   - TCP BulkSendApplication for reliable large-volume data transfer.
 *   - AODV with extended ActiveRouteTimeout for dynamic route discovery.
 *   - PacketSink is installed on ALL satellites so that whichever satellite
 *     enters the visibility window can receive data.
 *   - At startup, the simulation scans all satellites and automatically
 *     selects the closest one as the initial target.
 *
 * Reference files: calculate-delay.cc, uas-vanet.cc
 * Module:          ns-3-leo (LeoChannelHelper, LeoPropagationLossModel,
 *                  LeoMockChannel, LeoOrbitNodeHelper)
 *
 * Usage examples:
 *   ./waf --run "uav-to-leo"
 *   ./waf --run "uav-to-leo --duration=600 --maxBytes=0"
 *   ./waf --run "uav-to-leo --uavLat=24.8 --uavLon=121.0 --uavAlt=500"
 *   ./waf --run "uav-to-leo --constellation=TelesatGateway --pcap=true"
 */

#include <iostream>
#include <cmath>
#include <map>
#include <fstream>
#include <vector>
#include <algorithm>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/leo-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"

using namespace ns3;
using namespace std;

// ============================================================================
// Constants
// ============================================================================
static const double EARTH_RADIUS = 6.37101e6;  // meters (same as LEO_PROP_EARTH_RAD)

// ============================================================================
// Global data structures for end-to-end delay measurement
// ============================================================================
// Same approach as calculate-delay.cc: match Tx and Rx by packet UID.

map<uint64_t, double> TxTimes;  ///< Tx timestamp keyed by packet UID
map<uint64_t, double> delay;    ///< End-to-end delay keyed by packet UID
uint64_t g_traceCallCount = 0;  ///< Debug counter: how many times EchoTxRx was called

// ============================================================================
// Trace callback: TCP Tx / Rx — identical to calculate-delay.cc
// ============================================================================
static void
EchoTxRx (std::string context,
          const Ptr<const Packet> packet,
          const TcpHeader &header,
          const Ptr<const TcpSocketBase> socket)
{
    double time = Simulator::Now ().GetSeconds ();
    uint64_t uid = packet->GetUid ();
    g_traceCallCount++;

    if (context.find ("/Tx") != std::string::npos)
    {
        TxTimes[uid] = time;
    }
    else if (context.find ("/Rx") != std::string::npos)
    {
        if (TxTimes.find (uid) != TxTimes.end ())
        {
            delay[uid] = time - TxTimes[uid];
        }
    }

    // std::cout << Simulator::Now () << ":" << context << ":" << uid
    //           << ":" << socket->GetNode ()
    //           << ":" << header.GetSequenceNumber () << std::endl;
}

// ============================================================================
// Deferred trace connection — identical to calculate-delay.cc
// ============================================================================
void
connect ()
{
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Tx",
                     MakeCallback (&EchoTxRx));
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Rx",
                     MakeCallback (&EchoTxRx));
    // Debug: confirm this function actually ran
    std::cerr << "[TRACE] connect() executed at t="
              << Simulator::Now ().GetSeconds () << "s" << std::endl;
}

// ============================================================================
// Coordinate conversion utilities
// ============================================================================

/**
 * \brief Geodetic (lat, lon, alt) -> ECEF.  Spherical Earth model.
 *        Uses same radius constant as LeoPropagationLossModel.
 */
static Vector
GeoToEcef (double latDeg, double lonDeg, double altM)
{
    double latRad = latDeg * M_PI / 180.0;
    double lonRad = lonDeg * M_PI / 180.0;
    double R = EARTH_RADIUS + altM;
    return Vector (R * cos (latRad) * cos (lonRad),
                   R * cos (latRad) * sin (lonRad),
                   R * sin (latRad));
}

/**
 * \brief ECEF -> geodetic (lat deg, lon deg, alt km) for display.
 */
static void
EcefToGeo (const Vector &ecef, double &latDeg, double &lonDeg, double &altKm)
{
    double r = sqrt (ecef.x * ecef.x + ecef.y * ecef.y + ecef.z * ecef.z);
    latDeg = asin (ecef.z / r) * 180.0 / M_PI;
    lonDeg = atan2 (ecef.y, ecef.x) * 180.0 / M_PI;
    altKm  = (r - EARTH_RADIUS) / 1000.0;
}

/**
 * \brief Euclidean distance between two ECEF positions (meters).
 */
static double
EcefDistance (const Vector &a, const Vector &b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = a.z - b.z;
    return sqrt (dx * dx + dy * dy + dz * dz);
}

/**
 * \brief Compute elevation angle (degrees) from a ground point to a satellite.
 *
 * The elevation angle is the angle above the local horizon at the ground
 * point.  The LEO module's propagation loss model drops packets when this
 * angle is below the configured cutoff (e.g., 40 deg for TelesatUser).
 *
 * \param gndEcef  Ground point ECEF position
 * \param satEcef  Satellite ECEF position
 * \return         Elevation angle in degrees (0 = horizon, 90 = zenith)
 */
static double
ComputeElevationAngle (const Vector &gndEcef, const Vector &satEcef)
{
    // Vector from ground to satellite
    double dx = satEcef.x - gndEcef.x;
    double dy = satEcef.y - gndEcef.y;
    double dz = satEcef.z - gndEcef.z;
    double slantRange = sqrt (dx * dx + dy * dy + dz * dz);

    if (slantRange < 1.0) return 90.0;  // coincident points

    // Ground point unit normal (points radially outward from Earth center)
    double gndR = sqrt (gndEcef.x * gndEcef.x +
                        gndEcef.y * gndEcef.y +
                        gndEcef.z * gndEcef.z);
    double nx = gndEcef.x / gndR;
    double ny = gndEcef.y / gndR;
    double nz = gndEcef.z / gndR;

    // Dot product of ground-to-sat vector with the surface normal
    double dot = (dx * nx + dy * ny + dz * nz) / slantRange;

    // Elevation = 90 - zenith angle = 90 - acos(dot)
    // Equivalently: elevation = asin(dot)
    double elevRad = asin (std::max (-1.0, std::min (1.0, dot)));
    return elevRad * 180.0 / M_PI;
}

// ============================================================================
// Find the closest satellite to the UAV
// ============================================================================
/**
 * \brief Scans all satellites and returns the index of the one closest to
 *        the UAV at the current simulation time.  Also prints the top-N
 *        closest satellites with their distances and elevation angles.
 *
 * \param satellites  NodeContainer of all satellites
 * \param uavNode     The UAV node
 * \param topN        Number of top candidates to print
 * \return            Index of the closest satellite
 */
static uint32_t
FindClosestSatellite (const NodeContainer &satellites,
                      Ptr<Node> uavNode,
                      int topN = 10)
{
    Vector uavPos = uavNode->GetObject<MobilityModel> ()->GetPosition ();
    uint32_t numSats = satellites.GetN ();

    // Build a list of (distance, index) pairs
    std::vector<std::pair<double, uint32_t>> distList;
    for (uint32_t i = 0; i < numSats; i++)
    {
        Vector satPos = satellites.Get (i)->GetObject<MobilityModel> ()->GetPosition ();
        double dist = EcefDistance (uavPos, satPos);
        distList.push_back (std::make_pair (dist, i));
    }

    // Sort by distance (ascending)
    std::sort (distList.begin (), distList.end ());

    // Print the top-N closest satellites
    std::cerr << "\n=== Closest satellites to UAV at t="
              << Simulator::Now ().GetSeconds () << "s ===" << std::endl;
    int printed = 0;
    for (auto &[dist, idx] : distList)
    {
        if (printed >= topN) break;
        Vector satPos = satellites.Get (idx)->GetObject<MobilityModel> ()->GetPosition ();
        double lat, lon, altKm;
        EcefToGeo (satPos, lat, lon, altKm);
        double elev = ComputeElevationAngle (uavPos, satPos);
        std::cerr << "  Sat[" << idx << "] dist=" << dist / 1000.0
                  << " km, elev=" << elev << " deg"
                  << ", lat=" << lat << " lon=" << lon
                  << ((elev >= 40.0) ? "  ** VISIBLE (>=40 deg) **" :
                     ((elev >= 20.0) ? "  * VISIBLE (>=20 deg) *" : ""))
                  << std::endl;
        printed++;
    }

    return distList[0].second;  // index of the closest satellite
}

// ============================================================================
// Void wrapper for FindClosestSatellite (Simulator::Schedule needs void return)
// ============================================================================
static void
PrintClosestSatellites (const NodeContainer &satellites,
                        Ptr<Node> uavNode,
                        int topN)
{
    FindClosestSatellite (satellites, uavNode, topN);
}

// ============================================================================
// Periodic monitoring helpers
// ============================================================================

void
PrintSatellitePositions (const NodeContainer &satellites, int maxPrint)
{
    std::cerr << "=== Satellite positions at t="
              << Simulator::Now ().GetSeconds () << "s ===" << std::endl;
    for (int i = 0; i < min ((int) satellites.GetN (), maxPrint); i++)
    {
        Vector pos = satellites.Get (i)->GetObject<MobilityModel> ()->GetPosition ();
        double lat, lon, altKm;
        EcefToGeo (pos, lat, lon, altKm);
        std::cerr << "  Sat[" << i << "] lat=" << lat
                  << " lon=" << lon
                  << " alt=" << altKm << " km" << std::endl;
    }
}

void
PrintUavPosition (Ptr<Node> uavNode)
{
    Vector pos = uavNode->GetObject<MobilityModel> ()->GetPosition ();
    double lat, lon, altKm;
    EcefToGeo (pos, lat, lon, altKm);
    std::cerr << "  UAV   lat=" << lat
              << " lon=" << lon
              << " alt=" << altKm * 1000.0 << " m" << std::endl;
}

void
PrintUavSatDistance (Ptr<Node> uavNode, Ptr<Node> satNode, uint32_t satIdx)
{
    Vector uavPos = uavNode->GetObject<MobilityModel> ()->GetPosition ();
    Vector satPos = satNode->GetObject<MobilityModel> ()->GetPosition ();
    double dist = EcefDistance (uavPos, satPos);
    double elev = ComputeElevationAngle (uavPos, satPos);
    std::cerr << "  t=" << Simulator::Now ().GetSeconds ()
              << "s  UAV <-> Sat[" << satIdx << "] dist="
              << dist / 1000.0 << " km, elev=" << elev << " deg"
              << ((elev >= 40.0) ? " [VISIBLE]" : " [below cutoff]")
              << std::endl;
}

// ============================================================================
NS_LOG_COMPONENT_DEFINE ("UavToLeoExample");
// ============================================================================

int
main (int argc, char *argv[])
{
    // ========================================================================
    // 1. Command-line parameters
    // ========================================================================

    std::string orbitFile;
    std::string traceFile;

    // UAV position — default: Hsinchu, Taiwan
    double uavLatDeg  = 24.80;
    double uavLonDeg  = 120.97;
    double uavAltM    = 300.0;

    // Constellation preset
    std::string constellation = "TelesatUser";

    // Target satellite — -1 means "auto-select closest"
    int32_t targetSatIndex = -1;

    // Application
    uint16_t port     = 9;
    uint32_t maxBytes = 100 * 1024 * 1024;   // 10 MB
    uint32_t sendSize = 10 * 1024;
    double   duration = 300.0;

    // Routing
    uint64_t ttlThresh   = 0;
    double   routeTimeout = 300.0;

    // Debug
    bool pcap          = false;

    // ========================================================================
    // 2. Parse command line
    // ========================================================================

    CommandLine cmd;
    cmd.AddValue ("orbitFile",       "CSV file with orbit parameters",         orbitFile);
    cmd.AddValue ("traceFile",       "CSV file to redirect stdout to",         traceFile);
    cmd.AddValue ("precision",       "ns3::LeoCircularOrbitMobilityModel::Precision");
    cmd.AddValue ("duration",        "Simulation duration (seconds)",          duration);
    cmd.AddValue ("uavLat",          "UAV latitude  (degrees N)",              uavLatDeg);
    cmd.AddValue ("uavLon",          "UAV longitude (degrees E)",              uavLonDeg);
    cmd.AddValue ("uavAlt",          "UAV altitude  (meters ASL)",             uavAltM);
    cmd.AddValue ("constellation",   "LEO constellation preset name",          constellation);
    cmd.AddValue ("targetSatIndex",  "Satellite index (-1 = auto-closest)",    targetSatIndex);
    cmd.AddValue ("maxBytes",        "Total bytes to send (0 = unlimited)",    maxBytes);
    cmd.AddValue ("sendSize",        "TCP segment size (bytes)",               sendSize);
    cmd.AddValue ("ttlThresh",       "AODV TTL threshold",                     ttlThresh);
    cmd.AddValue ("routeTimeout",    "AODV ActiveRouteTimeout (seconds)",      routeTimeout);
    cmd.AddValue ("destOnly",        "ns3::aodv::RoutingProtocol::DestinationOnly");
    cmd.AddValue ("pcap",            "Enable PCAP packet capture",             pcap);
    cmd.Parse (argc, argv);

    // ========================================================================
    // 3. Redirect stdout to trace file (optional)
    // ========================================================================

    std::streambuf *coutbuf = std::cout.rdbuf ();
    std::ofstream out;
    if (!traceFile.empty ())
    {
        out.open (traceFile);
        if (out.is_open ())
        {
            std::cout.rdbuf (out.rdbuf ());
        }
    }

    // ========================================================================
    // 4. Create LEO satellite constellation
    // ========================================================================
    //
    // LeoOrbit (height_km, inclination_deg, sats_per_plane, num_planes)

    LeoOrbitNodeHelper orbit;
    NodeContainer satellites;

    if (!orbitFile.empty ())
    {
        satellites = orbit.Install (orbitFile);
    }
    else
    {
        // Telesat-like: 1200 km, 53 deg incl, 22 sats/plane, 12 planes = 264 sats
        satellites = orbit.Install ({LeoOrbit (1200, 53, 22, 12)});
    }

    uint32_t numSats = satellites.GetN ();
    std::cerr << "Created " << numSats << " LEO satellites" << std::endl;

    // ========================================================================
    // 5. Create the main UAV node (cluster head)
    // ========================================================================
    //
    // From the LEO module's perspective, the UAV is a ground-station node.
    // LeoMockChannel classifies it as GND type and only allows GND<->SAT
    // communication — exactly the uplink we need.
    //
    // Position MUST be in ECEF because LeoPropagationLossModel computes
    // slant range and elevation angle from ECEF coordinates.

    NodeContainer uavNodes;
    uavNodes.Create (1);
    Ptr<Node> mainUav = uavNodes.Get (0);

    // Install mobility: ConstantPositionMobilityModel (hovering UAV)
    //
    // For a flight path, replace with WaypointMobilityModel:
    //   Ptr<WaypointMobilityModel> mob = CreateObject<WaypointMobilityModel>();
    //   mob->AddWaypoint(Waypoint(Seconds(0),   GeoToEcef(24.80, 120.97, 300)));
    //   mob->AddWaypoint(Waypoint(Seconds(60),  GeoToEcef(24.85, 121.00, 300)));
    //   mainUav->AggregateObject(mob);

    MobilityHelper uavMobility;
    Ptr<ListPositionAllocator> uavPosAlloc = CreateObject<ListPositionAllocator> ();
    Vector uavEcef = GeoToEcef (uavLatDeg, uavLonDeg, uavAltM);
    uavPosAlloc->Add (uavEcef);
    uavMobility.SetPositionAllocator (uavPosAlloc);
    uavMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    uavMobility.Install (uavNodes);

    std::cerr << "UAV ECEF: (" << uavEcef.x << ", "
              << uavEcef.y << ", " << uavEcef.z << ")" << std::endl;
    PrintUavPosition (mainUav);

    // ========================================================================
    // 6. Auto-select the closest satellite to the UAV
    // ========================================================================
    //
    // The previous version hard-coded targetSatIndex=0, which happened to be
    // a satellite starting at (lat=53, lon=0) — over 9000 km from Taiwan.
    // With 264 satellites spread across 12 orbital planes, there are usually
    // several satellites much closer to any given ground position.
    //
    // This scan finds the satellite with the smallest Euclidean distance to
    // the UAV and uses it as the BulkSend target.  It also prints the top-10
    // closest satellites with their elevation angles so you can verify
    // visibility.

    uint32_t autoClosest = FindClosestSatellite (satellites, mainUav, 10);

    if (targetSatIndex < 0)
    {
        // Auto-select
        targetSatIndex = (int32_t) autoClosest;
        std::cerr << "\nAuto-selected target: Sat[" << targetSatIndex << "]" << std::endl;
    }
    else if ((uint32_t) targetSatIndex >= numSats)
    {
        std::cerr << "WARNING: targetSatIndex=" << targetSatIndex
                  << " out of range, using auto-closest Sat[" << autoClosest << "]"
                  << std::endl;
        targetSatIndex = (int32_t) autoClosest;
    }

    Ptr<Node> targetSat = satellites.Get ((uint32_t) targetSatIndex);

    // ========================================================================
    // 7. Set up UAV-to-LEO channel (link model)
    // ========================================================================
    //
    // LeoChannelHelper creates:
    //   - LeoMockChannel:          GND <-> SAT channel (no GND<->GND)
    //   - LeoPropagationLossModel: path loss + elevation-angle cutoff
    //   - LeoMockNetDevice:        typed devices (GND or SAT)
    //
    // Constellation presets:
    //   TelesatUser:    13.5 GHz, elev 40 deg, ~316 Mbps
    //   TelesatGateway: 28.5 GHz, elev 20 deg, ~9.8 Gbps (needs 3.5m dish)
    //   StarlinkUser:   12.0 GHz, elev 40 deg, ~316 Mbps
    //   StarlinkGateway:29.5 GHz, elev 20 deg, ~9.8 Gbps

    LeoChannelHelper utCh;
    utCh.SetConstellation (constellation);

    // Install channel between ALL satellites and the UAV node.
    // LeoMockChannel internally decides per-packet whether the destination
    // satellite is within the elevation-angle window.
    NetDeviceContainer utNet = utCh.Install (satellites, uavNodes);

    std::cerr << "LEO channel: constellation=" << constellation
              << ", " << utNet.GetN () << " devices" << std::endl;

    // ========================================================================
    // 8. Install Internet stack with AODV routing
    // ========================================================================
    //
    // AODV handles dynamic route discovery as satellites orbit overhead.
    //   - EnableHello=false: HELLO broadcasts unreliable over satellite.
    //   - ActiveRouteTimeout=300s: matches ~5 min LEO pass duration.

    InternetStackHelper stack;
    AodvHelper aodv;
    aodv.Set ("EnableHello", BooleanValue (false));
    aodv.Set ("ActiveRouteTimeout", TimeValue (Seconds (routeTimeout)));

    if (ttlThresh != 0)
    {
        aodv.Set ("TtlThreshold", UintegerValue (ttlThresh));
        aodv.Set ("NetDiameter",  UintegerValue (2 * ttlThresh));
    }

    stack.SetRoutingHelper (aodv);
    stack.Install (satellites);
    stack.Install (uavNodes);

    // ========================================================================
    // 9. Assign IP addresses
    // ========================================================================

    Ipv4AddressHelper ipv4;
    ipv4.SetBase ("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer utIf = ipv4.Assign (utNet);

    // ========================================================================
    // 10. Resolve the target satellite's IP address
    // ========================================================================

    Ipv4Address targetAddr = targetSat->GetObject<Ipv4> ()
                                 ->GetAddress (1, 0).GetLocal ();

    std::cerr << "UAV  node ID = " << mainUav->GetId () << std::endl;
    std::cerr << "Target Sat[" << targetSatIndex
              << "] node ID = " << targetSat->GetId ()
              << ", IP = " << targetAddr << std::endl;

    // Verify the initial elevation angle to the target
    {
        Vector satPos = targetSat->GetObject<MobilityModel> ()->GetPosition ();
        double dist = EcefDistance (uavEcef, satPos);
        double elev = ComputeElevationAngle (uavEcef, satPos);
        std::cerr << "Initial link: dist=" << dist / 1000.0
                  << " km, elev=" << elev << " deg";
        if (elev >= 40.0)
            std::cerr << " [VISIBLE with TelesatUser]" << std::endl;
        else if (elev >= 20.0)
            std::cerr << " [VISIBLE with TelesatGateway only]" << std::endl;
        else
            std::cerr << " [NOT VISIBLE — connection may fail!]" << std::endl;
    }

    // ========================================================================
    // 11. Install TCP BulkSendApplication on the UAV (sender)
    // ========================================================================

    BulkSendHelper sender ("ns3::TcpSocketFactory",
                           InetSocketAddress (targetAddr, port));
    sender.SetAttribute ("MaxBytes", UintegerValue (maxBytes));
    sender.SetAttribute ("SendSize", UintegerValue (sendSize));

    ApplicationContainer sourceApps = sender.Install (mainUav);
    // CRITICAL: Start at t=0.0 so the TCP socket exists when Config::Connect
    // runs at t=1e-7.  Starting at t=1.0 means the socket doesn't exist yet
    // → UAV Tx is never traced → no delay data.  (Same as calculate-delay.cc)
    sourceApps.Start (Seconds (0.0));

    // ========================================================================
    // 12. Install PacketSink on ALL satellites
    // ========================================================================
    //
    // KEY FIX: Instead of installing PacketSink on only one satellite,
    // we install it on ALL satellites.  This way:
    //   - If AODV routes to the target satellite and it's visible, data flows.
    //   - If the target satellite moves out of range during the simulation,
    //     and another satellite enters range, AODV can potentially re-route
    //     (though BulkSend's TCP connection is bound to the target IP).
    //
    // For the primary data flow, the BulkSend connects to targetAddr.
    // PacketSink on other satellites is a safety net for future extensions
    // (e.g., anycast-style routing or multiple TCP connections).

    ApplicationContainer sinkApps;
    PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                                 InetSocketAddress (Ipv4Address::GetAny (), port));
    for (uint32_t i = 0; i < numSats; i++)
    {
        ApplicationContainer app = sinkHelper.Install (satellites.Get (i));
        app.Start (Seconds (0.0));
        if (i == (uint32_t) targetSatIndex)
        {
            sinkApps.Add (app);  // Track the target's sink for results
        }
    }

    // ========================================================================
    // 13. Connect TCP trace sources for delay measurement
    // ========================================================================

    Simulator::Schedule (Seconds (1e-7), &connect);

    // ========================================================================
    // 14. Schedule periodic monitoring
    // ========================================================================

    // Print distance + elevation to target satellite every 30s
    for (int t = 0; t <= (int) duration; t += 30)
    {
        Simulator::Schedule (Seconds (t), &PrintUavSatDistance,
                             mainUav, targetSat, (uint32_t) targetSatIndex);
    }

    // Re-scan for closest satellite every 60s (informational)
    for (int t = 60; t <= (int) duration; t += 60)
    {
        Simulator::Schedule (Seconds (t), &PrintClosestSatellites,
                             satellites, mainUav, 5);
    }

    // ========================================================================
    // 15. Enable PCAP tracing (optional)
    // ========================================================================

    if (pcap)
    {
        AsciiTraceHelper ascii;
        utCh.EnableAsciiAll (ascii.CreateFileStream ("uav-to-leo.tr"));
        utCh.EnablePcapAll ("uav-to-leo", false);
    }

    // ========================================================================
    // 16. Run simulation
    // ========================================================================

    std::cerr << "\n=== Starting simulation ==="
              << "\n  Duration:      " << duration << "s"
              << "\n  MaxBytes:      " << maxBytes
              << "\n  Constellation: " << constellation
              << "\n  Target:        Sat[" << targetSatIndex << "] IP=" << targetAddr
              << "\n  UAV position:  (" << uavLatDeg << "N, " << uavLonDeg << "E, "
              << uavAltM << "m)"
              << "\n=========================" << std::endl;

    NS_LOG_INFO ("Run Simulation.");
    Simulator::Stop (Seconds (duration));
    Simulator::Run ();
    Simulator::Destroy ();
    NS_LOG_INFO ("Done.");

    // ========================================================================
    // 17. Output results
    // ========================================================================

    Ptr<PacketSink> pktSink = DynamicCast<PacketSink> (sinkApps.Get (0));
    uint64_t totalRx = pktSink->GetTotalRx ();

    std::cout << "\n========== UAV-to-LEO Simulation Results ==========" << std::endl;
    std::cout << "UAV node:       " << mainUav->GetId () << std::endl;
    std::cout << "Target Sat[" << targetSatIndex << "]: node "
              << targetSat->GetId () << " (IP " << targetAddr << ")" << std::endl;
    std::cout << "Constellation:  " << constellation << std::endl;
    std::cout << "Duration:       " << duration << " s" << std::endl;
    std::cout << "Bytes requested:" << maxBytes << std::endl;
    std::cout << "Bytes received: " << totalRx << std::endl;

    // Throughput
    if (duration > 0)
    {
        double throughputKbps = (totalRx * 8.0) / (duration * 1e3);
        double throughputMbps = throughputKbps / 1e3;
        std::cout << "Avg throughput: " << throughputKbps << " kbps ("
                  << throughputMbps << " Mbps)" << std::endl;
    }

    // Debug diagnostics to cerr (always visible on terminal)
    std::cerr << "\n[DEBUG] EchoTxRx was called " << g_traceCallCount << " times" << std::endl;
    std::cerr << "[DEBUG] TxTimes entries: " << TxTimes.size () << std::endl;
    std::cerr << "[DEBUG] delay entries:   " << delay.size () << std::endl;

    // End-to-end delay — same output pattern as calculate-delay.cc
    if (!delay.empty ())
    {
        double totalDelay = 0.0, minDelay = 1e9, maxDelay = 0.0;
        double nums = 0;

        for (auto &[uid, d] : delay)
        {
            totalDelay += d;
            nums += 1;
            if (d < minDelay) minDelay = d;
            if (d > maxDelay) maxDelay = d;
        }
        double avgDelay = totalDelay / nums;

        std::cout << "Packets measured:  " << (int) nums << std::endl;
        std::cout << "Avg delay:         " << avgDelay * 1000.0 << " ms" << std::endl;
        std::cout << "Min delay:         " << minDelay * 1000.0 << " ms" << std::endl;
        std::cout << "Max delay:         " << maxDelay * 1000.0 << " ms" << std::endl;
        cout << "Packet average end-to-end delay is " << avgDelay << "s" << endl;
    }
    else
    {
        std::cout << "\nWARNING: No delay measurements collected." << std::endl;
        std::cout << "Check the elevation angles printed above." << std::endl;
        std::cout << "If all say '[below cutoff]', try:" << std::endl;
        std::cout << "  --constellation=TelesatGateway  (20 deg cutoff)" << std::endl;
        std::cout << "  --duration=600                  (longer window)" << std::endl;
    }
    std::cout << "=====================================================" << std::endl;

    // Restore stdout
    if (out.is_open ())
    {
        out.close ();
        std::cout.rdbuf (coutbuf);
    }

    return 0;
}