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
#include <string>
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
    double bfGainDb;           // [UPDATE-3] Beamforming gain (dB), 0 = no beamforming
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
    40.0,                      // Min elevation: 40 deg
    0.0                        // [UPDATE-3] BF gain: 0 dB (no beamforming)
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
    20.0,                      // Min elevation: 20 deg
    0.0                        // [UPDATE-3] BF gain: 0 dB
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
    30.0,                      // Min elevation: 30 deg
    0.0                        // [UPDATE-3] BF gain: 0 dB
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
    20.0,                      // Min elevation: 20 deg
    0.0                        // [UPDATE-3] BF gain: 0 dB
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
 *   Rx Power (dBm) = EIRP + bfGain - FSPL - atmLoss + rxGain - rxLoss - linkMargin
 *   Noise (dBm)    = -228.6 (dBW/K/Hz) + 10*log10(T) + 10*log10(B_Hz) + 30
 *   SNR (dB)       = Rx Power - Noise
 *
 * [UPDATE-3] bfGain 加在 EIRP 之後，等效提高發射功率。
 *   Beamforming 將天線陣列的能量集中在衛星方向，效果等同於增加 EIRP。
 *   bfGainDb = 10*log10(N)，N 為天線元素數（例如 4x4=16 → 12 dB）。
 *
 * \param band      頻段參數（含 bfGainDb）
 * \param distKm    斜距 (km)
 * \return          SNR (dB)
 */
static double
CalcSNR (const SatBandParams &band, double distKm)
{
    double fspl = CalcFSPL (distKm, band.freqGHz);

    // Received power (dBm)
    // [UPDATE-3] bfGainDb 加在這裡，等效提高發射端的 EIRP
    double rxPowerDbm = band.eirpDbm
                        + band.bfGainDb     // [UPDATE-3] Beamforming gain
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

    std::cerr << "\n  Link Budget: " << band.name
              << " | " << band.freqGHz << " GHz, BW=" << band.bandwidthGHz * 1000.0 << " MHz"
              << " | EIRP=" << band.eirpDbm << " dBm"
              << ((band.bfGainDb > 0) ? " + BF=" : "")
              << ((band.bfGainDb > 0) ? std::to_string((int)band.bfGainDb) + " dB" : "")
              << ", RxGain=" << band.rxGainDbi << " dBi"
              << "\n               dist=" << distKm << " km, elev=" << elevDeg << " deg"
              << " | FSPL=" << fspl << " dB, SNR=" << snrDb << " dB"
              << " | Shannon=" << capacityMbps << " Mbps" << std::endl;
}

// ============================================================================
// Global data structures for end-to-end delay measurement
// ============================================================================
// Same approach as calculate-delay.cc: match Tx and Rx by packet UID.

map<uint64_t, double> TxTimes;
map<uint64_t, double> delay;

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
FindClosestSatellite (const NodeContainer &satellites, Ptr<Node> uavNode, int topN = 3)
{
    Vector uavPos = uavNode->GetObject<MobilityModel> ()->GetPosition ();
    std::vector<std::pair<double, uint32_t>> distList;
    for (uint32_t i = 0; i < satellites.GetN (); i++)
    {
        Vector satPos = satellites.Get (i)->GetObject<MobilityModel> ()->GetPosition ();
        distList.push_back ({EcefDistance (uavPos, satPos), i});
    }
    std::sort (distList.begin (), distList.end ());
    std::cerr << "  Top-" << topN << " closest satellites:" << std::endl;
    int printed = 0;
    for (auto &[dist, idx] : distList)
    {
        if (printed >= topN) break;
        Vector satPos = satellites.Get (idx)->GetObject<MobilityModel> ()->GetPosition ();
        double elev = ComputeElevationAngle (uavPos, satPos);
        std::cerr << "    Sat[" << idx << "] dist="
                  << std::fixed << std::setprecision(0)
                  << dist / 1000.0 << " km, elev="
                  << std::setprecision(1) << elev << " deg" << std::endl;
        printed++;
    }
    return distList[0].second;
}

// ============================================================================
// [UPDATE-1] Adaptive data rate: periodically update DataRate based on SNR
// ============================================================================
// 每隔固定間隔重新計算 UAV↔target satellite 的即時斜距 → SNR → Shannon rate，
// 並透過 MockNetDevice::SetDataRate() 動態更新 UAV device 的 data rate。
//
// 這模擬了 adaptive modulation & coding (AMC) 的效果：
//   - 衛星靠近 (高仰角) → 低 FSPL → 高 SNR → 高 data rate
//   - 衛星遠離 (低仰角) → 高 FSPL → 低 SNR → 低 data rate
//   - 衛星低於仰角門檻  → link 斷開，data rate 設為最低值

/// [UPDATE-1] 記錄每次 adaptive rate 更新的結果，供模擬結束後統計
struct AdaptiveRateRecord
{
    double timeSec;
    double distKm;
    double elevDeg;
    double snrDb;
    double rateMbps;
};
std::vector<AdaptiveRateRecord> g_rateLog;

static void
UpdateAdaptiveRate (Ptr<Node> uavNode,
                    Ptr<Node> satNode,
                    NetDeviceContainer utNet,
                    SatBandParams band,
                    uint32_t satIdx)
{
    Vector uavPos = uavNode->GetObject<MobilityModel> ()->GetPosition ();
    Vector satPos = satNode->GetObject<MobilityModel> ()->GetPosition ();
    double distKm = EcefDistance (uavPos, satPos) / 1000.0;
    double elevDeg = ComputeElevationAngle (uavPos, satPos);

    // 計算即時 SNR 和 Shannon rate
    double snrDb = CalcSNR (band, distKm);
    double rateMbps = CalcShannonCapacity (band, snrDb);

    // 如果仰角低於門檻，link 不可用，設最低 rate
    if (elevDeg < band.elevAngleDeg)
    {
        rateMbps = 0.001;  // 近乎 0，但避免除以零
    }

    // 轉成 DataRate 字串
    std::ostringstream rateStr;
    rateStr << std::fixed << std::setprecision(1) << rateMbps << "Mbps";

    // 更新 UAV 的 GND device (utNet 的最後一個 device 是 UAV 的)
    // LeoChannelHelper::Install(satellites, uavNodes) 的順序是：
    //   先裝所有 satellite devices，最後裝 uavNodes 的 devices
    // 所以 UAV 的 device index = utNet.GetN() - 1
    uint32_t uavDevIdx = utNet.GetN () - 1;
    Ptr<MockNetDevice> uavDev = DynamicCast<MockNetDevice> (utNet.Get (uavDevIdx));
    if (uavDev)
    {
        uavDev->SetDataRate (DataRate (rateStr.str ()));
    }

    // 也更新 target satellite 的 device
    Ptr<MockNetDevice> satDev = DynamicCast<MockNetDevice> (utNet.Get (satIdx));
    if (satDev)
    {
        satDev->SetDataRate (DataRate (rateStr.str ()));
    }

    // 記錄到 g_rateLog（模擬結束後一次性輸出表格）
    g_rateLog.push_back ({Simulator::Now ().GetSeconds (), distKm, elevDeg, snrDb, rateMbps});
}

// ============================================================================
// Helper: print UAV position (used once at startup)
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

    // [UPDATE-1] Adaptive rate update interval (seconds)
    double   rateInterval = 10.0;

    bool pcap = false;

    // [UPDATE-3] Beamforming gain (dB), overrides band default
    // 0 = no beamforming, 6 = 2x2 array, 12 = 4x4 array, 18 = 8x8 array
    double bfGainDb = 0.0;

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
    cmd.AddValue ("rateInterval",    "Adaptive rate update interval (seconds)", rateInterval);
    cmd.AddValue ("bfGain",          "Beamforming gain in dB (0/6/12/18)",     bfGainDb);
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

    // [UPDATE-3] 套用命令列指定的 beamforming gain（覆寫 band 預設值）
    if (bfGainDb > 0.0)
    {
        band.bfGainDb = bfGainDb;
    }

    std::cerr << "Band: " << band.name << " (" << band.freqGHz << " GHz, BW="
              << band.bandwidthGHz * 1000.0 << " MHz, elev cutoff="
              << band.elevAngleDeg << " deg)"
              << std::endl;
    // [UPDATE-3] 顯示 beamforming 設定
    if (band.bfGainDb > 0.0)
    {
        std::cerr << "[UPDATE-3] Beamforming gain: " << band.bfGainDb << " dB" << std::endl;
    }

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

    PrintUavPosition (mainUav);

    // ========================================================================
    // 7. Auto-select closest satellite
    // ========================================================================

    uint32_t autoClosest = FindClosestSatellite (satellites, mainUav, 3);
    if (targetSatIndex < 0)
    {
        targetSatIndex = (int32_t) autoClosest;
    }
    else if ((uint32_t) targetSatIndex >= numSats)
    {
        targetSatIndex = (int32_t) autoClosest;
    }
    Ptr<Node> targetSat = satellites.Get ((uint32_t) targetSatIndex);
    std::cerr << "  Selected: Sat[" << targetSatIndex << "]" << std::endl;

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
    // 15. Periodic monitoring — adaptive rate updates only
    // ========================================================================
    // PrintUavSatDistance and PrintClosestSatellites removed to reduce clutter.
    // All distance/elev/SNR/rate info is in the Adaptive Rate Log table.

    // [UPDATE-1] 定期更新 adaptive data rate
    for (double t = rateInterval; t <= duration; t += rateInterval)
    {
        Simulator::Schedule (Seconds (t), &UpdateAdaptiveRate,
                             mainUav, targetSat, utNet, band,
                             (uint32_t) targetSatIndex);
    }

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

    std::cerr << "\n=== Simulation: " << duration << "s, "
              << band.name << ", rate=" << dataRateStr.str ()
              << ", Sat[" << targetSatIndex << "], "
              << "adaptive interval=" << rateInterval << "s ===" << std::endl;

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
    std::cout << "UAV node " << mainUav->GetId ()
              << " -> Sat[" << targetSatIndex << "] node "
              << targetSat->GetId () << " (IP " << targetAddr << ")" << std::endl;
    std::cout << "Band:           " << band.name
              << " (" << band.freqGHz << " GHz, BW=" << band.bandwidthGHz * 1000.0 << " MHz)" << std::endl;

    // [UPDATE-3] 顯示 beamforming gain
    if (band.bfGainDb > 0.0)
    {
        std::cout << "Beamforming:    +" << band.bfGainDb << " dB  [UPDATE-3]" << std::endl;
    }
    else
    {
        std::cout << "Beamforming:    none (0 dB)" << std::endl;
    }

    // *** Highlight: this is the parameter that decides LINK DOWN ***
    // It comes from SatBandParams::elevAngleDeg, which is set per frequency band.
    // - Ku-User:    40 deg  (from Telesat user link spec)
    // - Ka-Gateway: 20 deg  (from Telesat gateway link spec)
    // - Ka-User:    30 deg  (estimated for UAV Ka terminal)
    // - S-band:     20 deg  (typical for S-band links)
    // The LEO module's LeoPropagationLossModel also enforces this via its
    // "ElevationAngle" attribute (set by SetConstellation), which drops packets
    // when the satellite is below this angle from the ground node's horizon.
    std::cout << "Elev cutoff:    " << band.elevAngleDeg
              << " deg  <-- decides LINK DOWN (from " << band.name << " band spec)" << std::endl;

    std::cout << "Init link:      dist=" << std::fixed << std::setprecision(1)
              << initDistKm << " km, elev=" << initElevDeg
              << " deg, FSPL=" << fsplDb << " dB, SNR=" << snrDb
              << " dB, rate=" << shannonMbps << " Mbps" << std::endl;
    std::cout << "Duration:       " << duration << " s" << std::endl;
    std::cout << "Bytes:          " << totalRx << " / " << maxBytes << " received" << std::endl;

    if (duration > 0)
    {
        double throughputMbps = (totalRx * 8.0) / (duration * 1e6);
        std::cout << "Avg throughput: " << throughputMbps << " Mbps" << std::endl;
    }

    if (!delay.empty ())
    {
        double totalDelay = 0.0, minDelay = 1e9, maxDelay = 0.0, nums = 0;
        for (auto &[uid, d] : delay)
        {
            totalDelay += d; nums += 1;
            if (d < minDelay) minDelay = d;
            if (d > maxDelay) maxDelay = d;
        }
        std::cout << "Delay:          avg=" << (totalDelay / nums) * 1000.0
                  << " ms, min=" << minDelay * 1000.0
                  << " ms, max=" << maxDelay * 1000.0
                  << " ms (" << (int) nums << " pkts)" << std::endl;
    }

    // [UPDATE-1] Adaptive Rate Log — single compact table
    if (!g_rateLog.empty ())
    {
        std::cout << "\n--- Adaptive Rate Log (interval=" << rateInterval << "s) ---" << std::endl;
        std::cout << "  Time   Dist(km)   Elev    SNR    Rate(Mbps)  Status" << std::endl;
        for (auto &r : g_rateLog)
        {
            bool down = (r.elevDeg < band.elevAngleDeg);
            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(6) << r.timeSec << "s"
                      << std::setw(10) << r.distKm
                      << std::setw(8) << r.elevDeg << "°"
                      << std::setw(7) << r.snrDb << " dB"
                      << std::setw(11) << r.rateMbps
                      << "  " << (down ? "[DOWN <" : "[OK   >=")
                      << band.elevAngleDeg << "°]"
                      << std::endl;
        }
    }
    std::cout << "=====================================================" << std::endl;

    if (out.is_open ())
    {
        out.close ();
        std::cout.rdbuf (coutbuf);
    }
    return 0;
}
