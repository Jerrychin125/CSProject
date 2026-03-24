/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * uav-to-leo.cc
 *
 * Step 2: Direct UAV (cluster head) → LEO satellite data transmission.
 *
 * [UPDATE-2] 新增：使用衛星通訊頻段（Ka / Ku band）計算 link budget，
 *   透過 Shannon capacity 公式由 SNR 推導理論 data rate，
 *   並用 SetConstellation() + public API 覆寫 DataRate 將計算結果設定到 LEO channel。
 *   取代原本直接使用 preset 固定 data rate 的做法。
 */

#include <iostream>
#include <cmath>
#include <map>
#include <fstream>
#include <iomanip>
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
// [UPDATE-2] Satellite frequency band parameters
// ============================================================================
// 衛星通訊常見頻段參數。每組包含完整的 link budget 所需數值。
// 來源：del Portillo et al., MIT (leo-telesat-constants.h, leo-starlink-constants.h)
//
// 使用方式：透過 --band 命令列參數選擇頻段，程式會自動計算 FSPL、SNR、Shannon data rate。

struct SatBandParams
{
    std::string name;          // 頻段名稱
    double freqGHz;            // 載波頻率 (GHz)
    double bandwidthGHz;       // 通道頻寬 (GHz)
    double eirpDbm;            // 等效全向輻射功率 (dBm)
    double rxGainDbi;          // 接收天線增益 (dBi)
    double rxLossDb;           // 接收端損耗 (dB)
    double atmLossDb;          // 大氣損耗 (dB)
    double linkMarginDb;       // 鏈路餘量 (dB)
    double systemTempK;        // 系統噪聲溫度 (K)
    double elevAngleDeg;       // 最小仰角門檻 (deg)
};

// 預設頻段表
static const SatBandParams BAND_KU_USER = {
    "Ku-User",                 // Telesat user uplink
    13.5,                      // freq: Ku-band 13.5 GHz
    0.25,                      // BW: 250 MHz
    64.6,                      // EIRP: 64.6 dBm
    38.3,                      // Rx gain: 38.3 dBi
    0.0,                       // Rx loss: 0 dB
    0.41,                      // Atm loss: 0.41 dB
    0.76,                      // Link margin: 0.76 dB
    350.1,                     // System temp: 350.1 K
    40.0                       // Min elevation: 40 deg
};

static const SatBandParams BAND_KA_GATEWAY = {
    "Ka-Gateway",              // Telesat gateway uplink
    28.5,                      // freq: Ka-band 28.5 GHz
    2.1,                       // BW: 2100 MHz
    105.9,                     // EIRP: 105.9 dBm
    31.8,                      // Rx gain: 31.8 dBi
    0.0,                       // Rx loss: 0 dB
    4.8,                       // Atm loss: 4.8 dB
    0.36,                      // Link margin: 0.36 dB
    868.4,                     // System temp: 868.4 K
    20.0                       // Min elevation: 20 deg
};

static const SatBandParams BAND_KA_USER = {
    "Ka-User",                 // Ka-band user terminal (hypothetical UAV terminal)
    20.0,                      // freq: Ka-band downlink 20 GHz (可作 uplink 估算)
    0.5,                       // BW: 500 MHz
    70.0,                      // EIRP: 70 dBm (UAV 中型天線)
    35.0,                      // Rx gain: 35 dBi
    0.0,                       // Rx loss: 0 dB
    2.0,                       // Atm loss: 2.0 dB
    1.0,                       // Link margin: 1.0 dB
    300.0,                     // System temp: 300 K
    30.0                       // Min elevation: 30 deg
};

static const SatBandParams BAND_S = {
    "S-band",                  // S-band (低頻、低 data rate、高穿透力)
    2.2,                       // freq: 2.2 GHz
    0.02,                      // BW: 20 MHz
    50.0,                      // EIRP: 50 dBm
    25.0,                      // Rx gain: 25 dBi
    0.0,                       // Rx loss: 0 dB
    0.1,                       // Atm loss: 0.1 dB
    0.5,                       // Link margin: 0.5 dB
    290.0,                     // System temp: 290 K
    20.0                       // Min elevation: 20 deg
};

// ============================================================================
// [UPDATE-2] Link budget calculation functions
// ============================================================================

/**
 * \brief 計算自由空間路徑損耗 (FSPL)
 *
 * 公式: FSPL(dB) = 20*log10(d) + 20*log10(f) + 20*log10(4*pi/c)
 *   d = 斜距 (m), f = 頻率 (Hz), c = 光速 (m/s)
 *
 * 等價於: FSPL(dB) = 32.45 + 20*log10(f_MHz) + 20*log10(d_km)
 *
 * \param distKm    UAV 到衛星的斜距 (km)
 * \param freqGHz   載波頻率 (GHz)
 * \return          FSPL (dB)
 */
static double
CalcFSPL (double distKm, double freqGHz)
{
    // FSPL = 32.45 + 20*log10(f_MHz) + 20*log10(d_km)
    double freqMHz = freqGHz * 1000.0;
    return 32.45 + 20.0 * log10 (freqMHz) + 20.0 * log10 (distKm);
}

/**
 * \brief 計算接收端 SNR
 *
 * Link budget:
 *   Rx Power (dBm) = EIRP - FSPL - atmLoss + rxGain - rxLoss - linkMargin
 *   Noise (dBm)    = 10*log10(k*T*B) 轉成 dBm
 *                   = -228.6 (dBW/K/Hz) + 10*log10(T) + 10*log10(B_Hz) + 30
 *   SNR (dB)       = Rx Power - Noise
 *
 * \param band      頻段參數
 * \param distKm    斜距 (km)
 * \return          SNR (dB)
 */
static double
CalcSNR (const SatBandParams &band, double distKm)
{
    double fspl = CalcFSPL (distKm, band.freqGHz);

    // Received power (dBm)
    double rxPowerDbm = band.eirpDbm
                        - fspl
                        - band.atmLossDb
                        + band.rxGainDbi
                        - band.rxLossDb
                        - band.linkMarginDb;

    // Noise power (dBm)
    //   N = k * T * B  (Watts)
    //   k = 1.38e-23 J/K (Boltzmann)
    //   In dBm: N_dBm = -228.6 + 10*log10(T_K) + 10*log10(B_Hz) + 30
    double bwHz = band.bandwidthGHz * 1e9;
    double noiseDbm = -228.6 + 10.0 * log10 (band.systemTempK)
                             + 10.0 * log10 (bwHz)
                             + 30.0;  // dBW → dBm

    return rxPowerDbm - noiseDbm;
}

/**
 * \brief 由 SNR 計算 Shannon capacity (理論最大 data rate)
 *
 * 公式: C = B * log2(1 + SNR_linear)
 *
 * \param band      頻段參數 (取 bandwidthGHz)
 * \param snrDb     SNR (dB)
 * \return          Shannon capacity (Mbps)
 */
static double
CalcShannonCapacity (const SatBandParams &band, double snrDb)
{
    double snrLinear = pow (10.0, snrDb / 10.0);
    double bwHz = band.bandwidthGHz * 1e9;
    double capacityBps = bwHz * log2 (1.0 + snrLinear);
    return capacityBps / 1e6;  // → Mbps
}

/**
 * \brief 完整 link budget 計算並印出結果
 *
 * 以 UAV 與衛星之間的斜距和所選頻段，計算 FSPL → SNR → Shannon data rate。
 * 結果印到 cerr 供終端檢視。
 *
 * \param band      頻段參數
 * \param distKm    斜距 (km)
 * \param elevDeg   仰角 (deg)
 */
static void
PrintLinkBudget (const SatBandParams &band, double distKm, double elevDeg)
{
    double fspl = CalcFSPL (distKm, band.freqGHz);
    double snrDb = CalcSNR (band, distKm);
    double capacityMbps = CalcShannonCapacity (band, snrDb);

    std::cerr << "\n[UPDATE-2] === Link Budget (" << band.name << ") ===" << std::endl;
    std::cerr << "  Frequency:       " << band.freqGHz << " GHz" << std::endl;
    std::cerr << "  Bandwidth:       " << band.bandwidthGHz * 1000.0 << " MHz" << std::endl;
    std::cerr << "  EIRP:            " << band.eirpDbm << " dBm" << std::endl;
    std::cerr << "  Slant range:     " << distKm << " km" << std::endl;
    std::cerr << "  Elevation:       " << elevDeg << " deg" << std::endl;
    std::cerr << "  FSPL:            " << fspl << " dB" << std::endl;
    std::cerr << "  Atm loss:        " << band.atmLossDb << " dB" << std::endl;
    std::cerr << "  Rx gain:         " << band.rxGainDbi << " dBi" << std::endl;
    std::cerr << "  Link margin:     " << band.linkMarginDb << " dB" << std::endl;
    std::cerr << "  SNR:             " << snrDb << " dB" << std::endl;
    std::cerr << "  Shannon C:       " << capacityMbps << " Mbps" << std::endl;
    std::cerr << "  ============================================" << std::endl;
}

// ============================================================================
// Global data structures for end-to-end delay measurement
// ============================================================================
// Same approach as calculate-delay.cc: match Tx and Rx by packet UID.

map<uint64_t, double> TxTimes;
map<uint64_t, double> delay;
uint64_t g_traceCallCount = 0;

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

    // Per-packet log (commented out to avoid flooding terminal)
    // std::cout << Simulator::Now () << ":" << context << ":" << uid
    //           << ":" << socket->GetNode ()
    //           << ":" << header.GetSequenceNumber () << std::endl;
}

void
connect ()
{
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Tx",
                     MakeCallback (&EchoTxRx));
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Rx",
                     MakeCallback (&EchoTxRx));
    std::cerr << "[TRACE] connect() executed at t="
              << Simulator::Now ().GetSeconds () << "s" << std::endl;
}

// ============================================================================
// Coordinate conversion utilities (unchanged)
// ============================================================================

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

static void
EcefToGeo (const Vector &ecef, double &latDeg, double &lonDeg, double &altKm)
{
    double r = sqrt (ecef.x * ecef.x + ecef.y * ecef.y + ecef.z * ecef.z);
    latDeg = asin (ecef.z / r) * 180.0 / M_PI;
    lonDeg = atan2 (ecef.y, ecef.x) * 180.0 / M_PI;
    altKm  = (r - EARTH_RADIUS) / 1000.0;
}

static double
EcefDistance (const Vector &a, const Vector &b)
{
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return sqrt (dx * dx + dy * dy + dz * dz);
}

static double
ComputeElevationAngle (const Vector &gndEcef, const Vector &satEcef)
{
    double dx = satEcef.x - gndEcef.x;
    double dy = satEcef.y - gndEcef.y;
    double dz = satEcef.z - gndEcef.z;
    double slantRange = sqrt (dx * dx + dy * dy + dz * dz);
    if (slantRange < 1.0) return 90.0;
    double gndR = sqrt (gndEcef.x * gndEcef.x + gndEcef.y * gndEcef.y + gndEcef.z * gndEcef.z);
    double nx = gndEcef.x / gndR, ny = gndEcef.y / gndR, nz = gndEcef.z / gndR;
    double dot = (dx * nx + dy * ny + dz * nz) / slantRange;
    return asin (std::max (-1.0, std::min (1.0, dot))) * 180.0 / M_PI;
}

// ============================================================================
// Satellite selection (unchanged)
// ============================================================================

static uint32_t
FindClosestSatellite (const NodeContainer &satellites, Ptr<Node> uavNode, int topN = 10)
{
    Vector uavPos = uavNode->GetObject<MobilityModel> ()->GetPosition ();
    std::vector<std::pair<double, uint32_t>> distList;
    for (uint32_t i = 0; i < satellites.GetN (); i++)
    {
        Vector satPos = satellites.Get (i)->GetObject<MobilityModel> ()->GetPosition ();
        distList.push_back ({EcefDistance (uavPos, satPos), i});
    }
    std::sort (distList.begin (), distList.end ());
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
                  << " km, elev=" << elev << " deg, lat=" << lat << " lon=" << lon
                  << ((elev >= 40.0) ? "  ** VISIBLE (>=40) **" :
                     ((elev >= 20.0) ? "  * VISIBLE (>=20) *" : ""))
                  << std::endl;
        printed++;
    }
    return distList[0].second;
}

static void
PrintClosestSatellites (const NodeContainer &satellites, Ptr<Node> uavNode, int topN)
{
    FindClosestSatellite (satellites, uavNode, topN);
}

// ============================================================================
// Periodic monitoring helpers (unchanged)
// ============================================================================

void
PrintUavPosition (Ptr<Node> uavNode)
{
    Vector pos = uavNode->GetObject<MobilityModel> ()->GetPosition ();
    double lat, lon, altKm;
    EcefToGeo (pos, lat, lon, altKm);
    std::cerr << "  UAV   lat=" << lat << " lon=" << lon
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

    double uavLatDeg  = 24.80;
    double uavLonDeg  = 120.97;
    double uavAltM    = 300.0;

    // [UPDATE-2] 新增 --band 參數，取代原本的 --constellation
    // 可選: "Ku-User", "Ka-Gateway", "Ka-User", "S-band"
    std::string bandName = "Ku-User";

    int32_t targetSatIndex = -1;

    uint16_t port     = 9;
    uint32_t maxBytes = 10 * 1024 * 1024;
    uint32_t sendSize = 1024;
    double   duration = 300.0;

    uint64_t ttlThresh   = 0;
    double   routeTimeout = 300.0;

    bool pcap = false;

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
    cmd.AddValue ("band",            "Sat band: Ku-User|Ka-Gateway|Ka-User|S-band", bandName);
    cmd.AddValue ("targetSatIndex",  "Satellite index (-1 = auto-closest)",    targetSatIndex);
    cmd.AddValue ("maxBytes",        "Total bytes to send (0 = unlimited)",    maxBytes);
    cmd.AddValue ("sendSize",        "TCP segment size (bytes)",               sendSize);
    cmd.AddValue ("ttlThresh",       "AODV TTL threshold",                     ttlThresh);
    cmd.AddValue ("routeTimeout",    "AODV ActiveRouteTimeout (seconds)",      routeTimeout);
    cmd.AddValue ("destOnly",        "ns3::aodv::RoutingProtocol::DestinationOnly");
    cmd.AddValue ("pcap",            "Enable PCAP packet capture",             pcap);
    cmd.Parse (argc, argv);

    // ========================================================================
    // [UPDATE-2] 3. 選擇頻段參數
    // ========================================================================

    SatBandParams band;
    if (bandName == "Ku-User")          band = BAND_KU_USER;
    else if (bandName == "Ka-Gateway")  band = BAND_KA_GATEWAY;
    else if (bandName == "Ka-User")     band = BAND_KA_USER;
    else if (bandName == "S-band")      band = BAND_S;
    else
    {
        std::cerr << "ERROR: unknown band '" << bandName
                  << "'. Using Ku-User." << std::endl;
        band = BAND_KU_USER;
    }
    std::cerr << "[UPDATE-2] Selected band: " << band.name
              << " (" << band.freqGHz << " GHz, BW="
              << band.bandwidthGHz * 1000.0 << " MHz)" << std::endl;

    // ========================================================================
    // 4. Redirect stdout (optional)
    // ========================================================================

    std::streambuf *coutbuf = std::cout.rdbuf ();
    std::ofstream out;
    if (!traceFile.empty ())
    {
        out.open (traceFile);
        if (out.is_open ()) std::cout.rdbuf (out.rdbuf ());
    }

    // ========================================================================
    // 5. Create LEO satellite constellation
    // ========================================================================

    LeoOrbitNodeHelper orbit;
    NodeContainer satellites;
    if (!orbitFile.empty ())
        satellites = orbit.Install (orbitFile);
    else
        satellites = orbit.Install ({LeoOrbit (1200, 53, 22, 12)});

    uint32_t numSats = satellites.GetN ();
    std::cerr << "Created " << numSats << " LEO satellites" << std::endl;

    // ========================================================================
    // 6. Create the main UAV node
    // ========================================================================

    NodeContainer uavNodes;
    uavNodes.Create (1);
    Ptr<Node> mainUav = uavNodes.Get (0);

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
    // 7. Auto-select closest satellite
    // ========================================================================

    uint32_t autoClosest = FindClosestSatellite (satellites, mainUav, 10);
    if (targetSatIndex < 0)
    {
        targetSatIndex = (int32_t) autoClosest;
        std::cerr << "\nAuto-selected target: Sat[" << targetSatIndex << "]" << std::endl;
    }
    else if ((uint32_t) targetSatIndex >= numSats)
    {
        std::cerr << "WARNING: targetSatIndex out of range, using Sat["
                  << autoClosest << "]" << std::endl;
        targetSatIndex = (int32_t) autoClosest;
    }
    Ptr<Node> targetSat = satellites.Get ((uint32_t) targetSatIndex);

    // ========================================================================
    // [UPDATE-2] 8. 計算 link budget 並設定 LEO channel
    // ========================================================================
    //
    // 策略：
    //   1. 先用 SetConstellation() 載入一組 preset 作為 propagation loss 的基礎
    //      （ElevationAngle, FSPL, AtmosphericLoss, LinkMargin 等參數由 preset 決定）
    //   2. 用 UAV 到 target satellite 的實際斜距計算 FSPL → SNR → Shannon data rate
    //   3. 用 public API (SetGndDeviceAttribute / SetSatDeviceAttribute) 覆寫 DataRate
    //      為計算得到的 Shannon capacity
    //
    // 為什麼不直接呼叫 SetConstellationAttributes()？
    //   因為它是 private method，只有 SetConstellation() 內部可以呼叫。
    //   但 Device 的屬性 (TxPower, RxGain, DataRate 等) 可以透過 public API 覆寫。
    //
    // 注意：propagation loss model 的參數（ElevationAngle, FSPL 等）使用 preset 的值，
    // 不會完全匹配我們從實際距離算出的 FSPL。這是可接受的近似：
    // preset 的 FSPL 決定「是否能收到封包」（link feasibility），
    // 而我們計算的 Shannon rate 決定「收到封包時的傳輸速率」。

    Vector satPos = targetSat->GetObject<MobilityModel> ()->GetPosition ();
    double initDistKm = EcefDistance (uavEcef, satPos) / 1000.0;
    double initElevDeg = ComputeElevationAngle (uavEcef, satPos);

    // 計算 link budget
    double fsplDb = CalcFSPL (initDistKm, band.freqGHz);
    double snrDb  = CalcSNR (band, initDistKm);
    double shannonMbps = CalcShannonCapacity (band, snrDb);

    // 印出完整 link budget
    PrintLinkBudget (band, initDistKm, initElevDeg);

    // 把 Shannon capacity 轉成 ns-3 data rate 字串 (e.g. "456.7Mbps")
    std::ostringstream dataRateStr;
    dataRateStr << std::fixed << std::setprecision(1) << shannonMbps << "Mbps";

    std::cerr << "[UPDATE-2] Computed data rate: " << dataRateStr.str () << std::endl;

    // [UPDATE-2] Step 1: 用 SetConstellation() 載入 preset（設定 propagation loss 參數）
    // 根據所選頻段映射到最接近的 preset
    LeoChannelHelper utCh;
    if (band.freqGHz > 20.0)
        utCh.SetConstellation ("TelesatGateway");   // Ka-band → TelesatGateway preset
    else
        utCh.SetConstellation ("TelesatUser");       // Ku/S-band → TelesatUser preset

    // [UPDATE-2] Step 2: 用 public API 覆寫 DataRate 為 Shannon 計算值
    utCh.SetGndDeviceAttribute ("DataRate", StringValue (dataRateStr.str ()));
    utCh.SetSatDeviceAttribute ("DataRate", StringValue (dataRateStr.str ()));

    // [UPDATE-2] Step 3: 覆寫 TxPower 和 RxGain 以匹配所選頻段
    utCh.SetGndDeviceAttribute ("TxPower", DoubleValue (band.eirpDbm));
    utCh.SetSatDeviceAttribute ("TxPower", DoubleValue (band.eirpDbm));
    utCh.SetGndDeviceAttribute ("RxGain",  DoubleValue (band.rxGainDbi));
    utCh.SetSatDeviceAttribute ("RxGain",  DoubleValue (band.rxGainDbi));

    NetDeviceContainer utNet = utCh.Install (satellites, uavNodes);
    std::cerr << "LEO channel installed: " << utNet.GetN () << " devices, "
              << "band=" << band.name << ", rate=" << dataRateStr.str () << std::endl;

    // ========================================================================
    // 9. Install Internet stack with AODV
    // ========================================================================

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
    // 10. Assign IP addresses
    // ========================================================================

    Ipv4AddressHelper ipv4;
    ipv4.SetBase ("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer utIf = ipv4.Assign (utNet);

    // ========================================================================
    // 11. Resolve target satellite IP
    // ========================================================================

    Ipv4Address targetAddr = targetSat->GetObject<Ipv4> ()
                                 ->GetAddress (1, 0).GetLocal ();
    std::cerr << "UAV node ID = " << mainUav->GetId () << std::endl;
    std::cerr << "Target Sat[" << targetSatIndex << "] node ID = "
              << targetSat->GetId () << ", IP = " << targetAddr << std::endl;

    // ========================================================================
    // 12. Install BulkSend on UAV (sender)
    // ========================================================================

    BulkSendHelper sender ("ns3::TcpSocketFactory",
                           InetSocketAddress (targetAddr, port));
    sender.SetAttribute ("MaxBytes", UintegerValue (maxBytes));
    sender.SetAttribute ("SendSize", UintegerValue (sendSize));
    ApplicationContainer sourceApps = sender.Install (mainUav);
    sourceApps.Start (Seconds (0.0));

    // ========================================================================
    // 13. Install PacketSink on ALL satellites
    // ========================================================================

    ApplicationContainer sinkApps;
    PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",
                                 InetSocketAddress (Ipv4Address::GetAny (), port));
    for (uint32_t i = 0; i < numSats; i++)
    {
        ApplicationContainer app = sinkHelper.Install (satellites.Get (i));
        app.Start (Seconds (0.0));
        if (i == (uint32_t) targetSatIndex)
            sinkApps.Add (app);
    }

    // ========================================================================
    // 14. Connect traces
    // ========================================================================

    Simulator::Schedule (Seconds (1e-7), &connect);

    // ========================================================================
    // 15. Periodic monitoring
    // ========================================================================

    for (int t = 0; t <= (int) duration; t += 30)
        Simulator::Schedule (Seconds (t), &PrintUavSatDistance,
                             mainUav, targetSat, (uint32_t) targetSatIndex);
    for (int t = 60; t <= (int) duration; t += 60)
        Simulator::Schedule (Seconds (t), &PrintClosestSatellites,
                             satellites, mainUav, 5);

    // ========================================================================
    // 16. PCAP (optional)
    // ========================================================================

    if (pcap)
    {
        AsciiTraceHelper ascii;
        utCh.EnableAsciiAll (ascii.CreateFileStream ("uav-to-leo.tr"));
        utCh.EnablePcapAll ("uav-to-leo", false);
    }

    // ========================================================================
    // 17. Run simulation
    // ========================================================================

    std::cerr << "\n=== Starting simulation ==="
              << "\n  Duration:      " << duration << "s"
              << "\n  MaxBytes:      " << maxBytes
              << "\n  Band:          " << band.name
              << " (" << band.freqGHz << " GHz)"
              << "\n  Data rate:     " << dataRateStr.str ()
              << "\n  Target:        Sat[" << targetSatIndex << "] IP=" << targetAddr
              << "\n  UAV:           (" << uavLatDeg << "N, " << uavLonDeg << "E, "
              << uavAltM << "m)"
              << "\n=========================" << std::endl;

    NS_LOG_INFO ("Run Simulation.");
    Simulator::Stop (Seconds (duration));
    Simulator::Run ();
    Simulator::Destroy ();
    NS_LOG_INFO ("Done.");

    // ========================================================================
    // 18. Output results
    // ========================================================================

    Ptr<PacketSink> pktSink = DynamicCast<PacketSink> (sinkApps.Get (0));
    uint64_t totalRx = pktSink->GetTotalRx ();

    std::cout << "\n========== UAV-to-LEO Simulation Results ==========" << std::endl;
    std::cout << "UAV node:       " << mainUav->GetId () << std::endl;
    std::cout << "Target Sat[" << targetSatIndex << "]: node "
              << targetSat->GetId () << " (IP " << targetAddr << ")" << std::endl;

    // [UPDATE-2] 印出頻段與 link budget 結果
    std::cout << "Band:           " << band.name << " (" << band.freqGHz << " GHz)" << std::endl;
    std::cout << "Bandwidth:      " << band.bandwidthGHz * 1000.0 << " MHz" << std::endl;
    std::cout << "Init distance:  " << initDistKm << " km" << std::endl;
    std::cout << "Init elevation: " << initElevDeg << " deg" << std::endl;
    std::cout << "FSPL:           " << fsplDb << " dB" << std::endl;
    std::cout << "SNR:            " << snrDb << " dB" << std::endl;
    std::cout << "Shannon rate:   " << shannonMbps << " Mbps" << std::endl;
    std::cout << "NS-3 data rate: " << dataRateStr.str () << std::endl;

    std::cout << "Duration:       " << duration << " s" << std::endl;
    std::cout << "Bytes requested:" << maxBytes << std::endl;
    std::cout << "Bytes received: " << totalRx << std::endl;

    if (duration > 0)
    {
        double throughputKbps = (totalRx * 8.0) / (duration * 1e3);
        double throughputMbps = throughputKbps / 1e3;
        std::cout << "Avg throughput: " << throughputKbps << " kbps ("
                  << throughputMbps << " Mbps)" << std::endl;
    }

    // Debug diagnostics
    std::cerr << "\n[DEBUG] EchoTxRx called " << g_traceCallCount << " times" << std::endl;
    std::cerr << "[DEBUG] TxTimes entries: " << TxTimes.size () << std::endl;
    std::cerr << "[DEBUG] delay entries:   " << delay.size () << std::endl;

    if (!delay.empty ())
    {
        double totalDelay = 0.0, minDelay = 1e9, maxDelay = 0.0, nums = 0;
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
    }
    std::cout << "=====================================================" << std::endl;

    if (out.is_open ())
    {
        out.close ();
        std::cout.rdbuf (coutbuf);
    }
    return 0;
}