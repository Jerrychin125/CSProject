/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * uav-to-leo.cc
 *
 * Single-hop UAV (cluster head) → LEO satellite uplink simulator.
 * See uav_leo_guide.md for architecture, build, and usage instructions.
 */

#include <iostream>
#include <cmath>
#include <map>
#include <fstream>
#include <filesystem>
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

static const double EARTH_RADIUS = 6.37101e6;

// ============================================================================
// Frequency band presets
// ============================================================================

struct SatBandParams
{
    std::string name;
    double freqGHz;
    double bandwidthGHz;
    double eirpDbm;
    double rxGainDbi;
    double rxLossDb;
    double atmLossDb;
    double linkMarginDb;
    double systemTempK;
    double elevAngleDeg;    ///< Minimum elevation; link DOWN below this
    int    nAnt;            ///< Antenna element count (CLI may override)
    int    nRF;             ///< RF chain count for hybrid beamforming
};

static const SatBandParams BAND_KU_USER = {
    "Ku-User", 13.5, 0.25, 64.6, 38.3, 0.0, 0.41, 0.76, 350.1, 40.0, 1, 1
};

static const SatBandParams BAND_KA_GATEWAY = {
    "Ka-Gateway", 28.5, 2.1, 105.9, 31.8, 0.0, 4.8, 0.36, 868.4, 20.0, 1, 1
};

static const SatBandParams BAND_KA_USER = {
    "Ka-User", 20.0, 0.5, 70.0, 35.0, 0.0, 2.0, 1.0, 300.0, 30.0, 1, 1
};

static const SatBandParams BAND_S = {
    "S-band", 2.2, 0.02, 50.0, 25.0, 0.0, 0.1, 0.5, 290.0, 20.0, 1, 1
};

// ============================================================================
// Link budget calculation
// ============================================================================

/**
 * \param distKm   Slant range (km)
 * \param freqGHz  Carrier frequency (GHz)
 * \return         Free-space path loss (dB)
 */
static double
CalcFSPL (double distKm, double freqGHz)
{
    double freqMHz = freqGHz * 1000.0;
    return 32.45 + 20.0 * log10 (freqMHz) + 20.0 * log10 (distKm);
}

// ============================================================================
// Beamforming gain (steering-aware CSV lookup + analytical fallback)
// ============================================================================

std::map<int, std::map<int, double>> g_beamPatterns;  ///< [steering_deg][elev*10] = gain_dB

struct BeamPatternMeta
{
    int    nAnt         = 0;
    int    nRF          = 0;
    double freqGHz      = 0.0;
    double hybridLossDb = 0.0;
    bool   loaded       = false;
};
BeamPatternMeta g_bfMeta;

/**
 * \brief Parse MATLAB-generated steering-aware beam pattern CSV.
 *
 * Expected format: 3 columns (steering_deg, elevation_deg, gain_dB).
 * Comment lines starting with '#' are scanned for metadata (nAnt, nRF, ...).
 *
 * \param filename  Path to CSV file
 * \return          true on success, false otherwise
 */
static bool
LoadBeamPattern (const std::string &filename)
{
    std::ifstream file (filename);
    if (!file.is_open ())
    {
        std::cerr << "[BF] WARNING: Cannot open '" << filename << "'" << std::endl;
        return false;
    }

    std::string line;
    int dataCount = 0;
    bool sawHeader = false;

    while (std::getline (file, line))
    {
        if (line.empty ()) continue;

        if (line[0] == '#')
        {
            auto findVal = [&line](const std::string &key) -> std::string {
                size_t pos = line.find (key);
                if (pos == std::string::npos) return "";
                pos += key.length ();
                size_t end = line.find_first_of (",\n", pos);
                return line.substr (pos, end == std::string::npos ? end : end - pos);
            };
            std::string s;
            if (!(s = findVal ("nAnt=")).empty ())          g_bfMeta.nAnt = std::stoi (s);
            if (!(s = findVal ("nRF=")).empty ())           g_bfMeta.nRF  = std::stoi (s);
            if (!(s = findVal ("freq=")).empty ())          g_bfMeta.freqGHz = std::stod (s);
            if (!(s = findVal ("hybrid_loss=")).empty ())   g_bfMeta.hybridLossDb = std::stod (s);
            continue;
        }

        if (!sawHeader)
        {
            sawHeader = true;
            if (line.find ("steering_deg") == std::string::npos)
            {
                std::cerr << "[BF] ERROR: '" << filename
                          << "' is OLD 2-column format. Regenerate with beamforming.m"
                          << std::endl;
                file.close ();
                return false;
            }
            continue;
        }

        size_t c1 = line.find (',');
        if (c1 == std::string::npos) continue;
        size_t c2 = line.find (',', c1 + 1);
        if (c2 == std::string::npos) continue;

        double steeringDeg = std::stod (line.substr (0, c1));
        double elevDeg     = std::stod (line.substr (c1 + 1, c2 - c1 - 1));
        double gainDb      = std::stod (line.substr (c2 + 1));

        int steeringKey = (int) round (steeringDeg);
        int elevKey     = (int) round (elevDeg * 10.0);

        g_beamPatterns[steeringKey][elevKey] = gainDb;
        dataCount++;
    }
    file.close ();

    if (g_beamPatterns.empty ())
    {
        std::cerr << "[BF] ERROR: no data rows parsed from '" << filename << "'" << std::endl;
        return false;
    }

    g_bfMeta.loaded = true;

    std::cerr << "[BF] Loaded steering-aware beam pattern: "
              << g_beamPatterns.size () << " sectors × "
              << g_beamPatterns.begin ()->second.size () << " obs angles = "
              << dataCount << " entries from '" << filename << "'" << std::endl;
    if (g_bfMeta.nAnt > 0)
    {
        std::cerr << "  Metadata: nAnt=" << g_bfMeta.nAnt
                  << ", nRF=" << g_bfMeta.nRF
                  << ", freq=" << g_bfMeta.freqGHz << " GHz"
                  << ", hybrid_loss=" << g_bfMeta.hybridLossDb << " dB" << std::endl;
    }

    std::cerr << "  Per-steering-sector peak gain (obs=θ_s):" << std::endl;
    std::cerr << "  ";
    int colCount = 0;
    for (auto &kv : g_beamPatterns)
    {
        int steeringDeg = kv.first;
        const auto &sub = kv.second;
        int peakKey = steeringDeg * 10;
        auto it = sub.find (peakKey);
        if (it != sub.end ())
        {
            std::cerr << "θs=" << std::setw (2) << steeringDeg << "°:"
                      << std::fixed << std::setprecision (1)
                      << std::setw (6) << it->second << "dB  ";
            if (++colCount % 5 == 0) std::cerr << "\n  ";
        }
    }
    std::cerr << std::endl;

    return true;
}

/**
 * \param pattern   Single-sector sub-table (elev_key → gain_dB)
 * \param elevDeg   Observation elevation (deg)
 * \return          Gain (dB), linearly interpolated between adjacent samples
 */
static double
LookupGainInSector (const std::map<int, double> &pattern, double elevDeg)
{
    int elevKey = (int) round (elevDeg * 10.0);
    if (elevKey < 0)   elevKey = 0;
    if (elevKey > 900) elevKey = 900;

    auto gIt = pattern.find (elevKey);
    if (gIt != pattern.end ())
        return gIt->second;

    auto upper = pattern.lower_bound (elevKey);
    if (upper == pattern.end ())   return pattern.rbegin ()->second;
    if (upper == pattern.begin ())  return upper->second;
    auto lower = std::prev (upper);
    double frac = (double)(elevKey - lower->first)
                / (double)(upper->first - lower->first);
    return lower->second + frac * (upper->second - lower->second);
}

/**
 * \brief Compute beamforming gain for a given elevation.
 *
 * Uses steering-aware CSV lookup if loaded; otherwise an analytical fallback
 * 10·log10(N · sin(elev) · η_hybrid). Within ±BLEND_HALF_DEG of a sector
 * boundary, the two adjacent sectors are blended in linear power domain.
 *
 * \param elevDeg   Observation elevation (deg)
 * \param nAnt      Antenna element count (used by analytical fallback)
 * \param nRF       RF chain count (used by analytical fallback)
 * \return          Beamforming gain (dB)
 */
static double
CalcBeamformingGain (double elevDeg, int nAnt, int nRF)
{
    if (nAnt <= 1) return 0.0;

    if (!g_beamPatterns.empty ())
    {
        static const double BLEND_HALF_DEG = 3.0;

        double nearestBoundary = std::round (elevDeg / 10.0 - 0.5) * 10.0 + 5.0;
        nearestBoundary = std::max (5.0, std::min (85.0, nearestBoundary));

        double distToBoundary = std::abs (elevDeg - nearestBoundary);

        int lowerSector = (int) std::floor (nearestBoundary / 10.0) * 10;
        int upperSector = lowerSector + 10;
        lowerSector = std::max (0,  std::min (90, lowerSector));
        upperSector = std::max (0,  std::min (90, upperSector));

        auto lookupSector = [&](int sector) -> double {
            auto it = g_beamPatterns.find (sector);
            if (it == g_beamPatterns.end ())
            {
                it = g_beamPatterns.lower_bound (sector);
                if (it == g_beamPatterns.end ())
                    it = std::prev (g_beamPatterns.end ());
            }
            return LookupGainInSector (it->second, elevDeg);
        };

        double gainDb;

        if (distToBoundary >= BLEND_HALF_DEG)
        {
            int sector = (int) round (elevDeg / 10.0) * 10;
            if (sector < 0)  sector = 0;
            if (sector > 90) sector = 90;
            gainDb = lookupSector (sector);
        }
        else
        {
            double weight_upper, weight_lower;
            if (elevDeg >= nearestBoundary)
            {
                weight_upper = 0.5 + (distToBoundary / (2.0 * BLEND_HALF_DEG));
                weight_lower = 1.0 - weight_upper;
            }
            else
            {
                weight_lower = 0.5 + (distToBoundary / (2.0 * BLEND_HALF_DEG));
                weight_upper = 1.0 - weight_lower;
            }

            double gainLower = lookupSector (lowerSector);
            double gainUpper = lookupSector (upperSector);
            double powLower  = pow (10.0, gainLower / 10.0);
            double powUpper  = pow (10.0, gainUpper / 10.0);
            double powBlend  = weight_lower * powLower + weight_upper * powUpper;
            gainDb = 10.0 * log10 (powBlend);
        }

        return gainDb;
    }

    double elevRad = elevDeg * M_PI / 180.0;
    double taper = sin (elevRad);
    if (taper < 0.1) taper = 0.1;
    double hybridEff = (nRF < nAnt) ? (0.85 + 0.15 * ((double) nRF / nAnt)) : 1.0;
    return 10.0 * log10 ((double) nAnt * taper * hybridEff);
}

/**
 * \param elevDeg  Observation elevation (deg)
 * \return         Nearest 10° steering sector (display only)
 */
static int
GetSteeringSector (double elevDeg)
{
    int s = (int) round (elevDeg / 10.0) * 10;
    if (s < 0)  s = 0;
    if (s > 90) s = 90;
    return s;
}

/**
 * \param band     Band parameters (incl. nAnt, nRF)
 * \param distKm   Slant range (km)
 * \param elevDeg  Elevation (deg)
 * \return         Receiver SNR (dB)
 */
static double
CalcSNR (const SatBandParams &band, double distKm, double elevDeg = 90.0)
{
    double fspl   = CalcFSPL (distKm, band.freqGHz);
    double bfGain = CalcBeamformingGain (elevDeg, band.nAnt, band.nRF);

    double rxPowerDbm = band.eirpDbm + bfGain - fspl - band.atmLossDb
                        + band.rxGainDbi - band.rxLossDb - band.linkMarginDb;

    double bwHz = band.bandwidthGHz * 1e9;
    double noiseDbm = -228.6 + 10.0 * log10 (band.systemTempK)
                             + 10.0 * log10 (bwHz) + 30.0;

    return rxPowerDbm - noiseDbm;
}

/**
 * \param band   Band parameters (uses bandwidthGHz)
 * \param snrDb  SNR (dB)
 * \return       Shannon capacity (Mbps)
 */
static double
CalcShannonCapacity (const SatBandParams &band, double snrDb)
{
    double snrLinear   = pow (10.0, snrDb / 10.0);
    double bwHz        = band.bandwidthGHz * 1e9;
    double capacityBps = bwHz * log2 (1.0 + snrLinear);
    return capacityBps / 1e6;
}

static void
PrintLinkBudget (const SatBandParams &band, double distKm, double elevDeg)
{
    double fspl    = CalcFSPL (distKm, band.freqGHz);
    double bfGain  = CalcBeamformingGain (elevDeg, band.nAnt, band.nRF);
    double snrDb   = CalcSNR (band, distKm, elevDeg);
    double capMbps = CalcShannonCapacity (band, snrDb);

    std::cerr << "\n  Link Budget: " << band.name
              << " | " << band.freqGHz << " GHz, BW=" << band.bandwidthGHz * 1000.0 << " MHz"
              << " | EIRP=" << band.eirpDbm << " dBm";
    if (band.nAnt > 1)
    {
        std::cerr << "\n  Array: " << band.nAnt << " elements, " << band.nRF
                  << " RF chains" << (band.nRF < band.nAnt ? " (hybrid)" : " (full digital)")
                  << ", BF gain=" << std::fixed << std::setprecision(1)
                  << bfGain << " dB at elev=" << elevDeg << " deg";
        if (!g_beamPatterns.empty ())
            std::cerr << " (CSV, steering=" << GetSteeringSector (elevDeg) << "°)";
        else
            std::cerr << " (analytical fallback)";
    }
    std::cerr << "\n               dist=" << distKm << " km, elev=" << elevDeg << " deg"
              << " | FSPL=" << fspl << " dB, SNR=" << snrDb << " dB"
              << " | Shannon=" << capMbps << " Mbps" << std::endl;
}

// ============================================================================
// Per-packet delay measurement (TCP segment-level Tx/Rx matched by UID)
// ============================================================================

map<uint64_t, double> TxTimes;
map<uint64_t, double> delay;

static void
EchoTxRx (std::string context,
          const Ptr<const Packet> packet,
          const TcpHeader &header,
          const Ptr<const TcpSocketBase> socket)
{
    double time = Simulator::Now ().GetSeconds ();
    uint64_t uid = packet->GetUid ();

    if (context.find ("/Tx") != std::string::npos)
        TxTimes[uid] = time;
    else if (context.find ("/Rx") != std::string::npos)
    {
        if (TxTimes.find (uid) != TxTimes.end ())
            delay[uid] = time - TxTimes[uid];
    }

    std::cout << '\r' << Simulator::Now () << ":" << context << ":" << uid
              << ":" << socket->GetNode ()
              << ":" << header.GetSequenceNumber () << std::flush;
}

// ============================================================================
// Effective throughput measurement (application-layer Tx/Rx hooks)
// ============================================================================

struct ThroughputRecord
{
    double   firstTxSec   = -1.0;
    double   firstRxSec   = -1.0;
    double   lastRxSec    = -1.0;
    uint64_t totalTxBytes = 0;
    uint64_t totalRxBytes = 0;
};
static ThroughputRecord g_tput;

static bool     g_fixedVolume = false;
static uint64_t g_volumeBytes = 0;
static bool     g_stopFired   = false;

// [Step-3] One record per (BeginWindow → CloseWindow) pair. windowId starts
//          at 1 (initial connection) and increments each handoff.
struct WindowRecord
{
    int       windowId      = 0;
    uint32_t  satIdx        = 0;
    double    startTimeSec  = -1.0;
    double    endTimeSec    = -1.0;
    double    firstTxSec    = -1.0;
    double    lastRxSec     = -1.0;
    uint64_t  totalTxBytes  = 0;
    uint64_t  totalRxBytes  = 0;
    double    avgEffMbps    = 0.0;
};
static std::vector<WindowRecord> g_windows;
static WindowRecord              g_curWindow;
static bool                      g_windowOpen = false;

// [Step-3] Open a window owned by the given satellite. Called once at sim
//          start (window 1) and once per handoff thereafter.
static void
BeginWindow (uint32_t satIdx)
{
    g_curWindow              = WindowRecord ();
    g_curWindow.windowId     = (int) g_windows.size () + 1;
    g_curWindow.satIdx       = satIdx;
    g_curWindow.startTimeSec = Simulator::Now ().GetSeconds ();
    g_windowOpen             = true;
}

// [Step-3] Close the current window. Uses the same effective-throughput
//          formula as Section 16: rxBytes * 8 / (lastRxSec - firstTxSec).
//          Idempotent — second call is a no-op.
static void
CloseWindow (double endTimeSec)
{
    if (!g_windowOpen) return;

    g_curWindow.endTimeSec = endTimeSec;

    double measSec = g_curWindow.lastRxSec - g_curWindow.firstTxSec;
    if (measSec > 0.0 && g_curWindow.totalRxBytes > 0)
        g_curWindow.avgEffMbps = (g_curWindow.totalRxBytes * 8.0)
                               / (measSec * 1e6);

    g_windowOpen = false;

    std::cerr << "[Step-3] Window " << g_curWindow.windowId
              << " closed: Sat[" << g_curWindow.satIdx << "] "
              << std::fixed << std::setprecision(2)
              << g_curWindow.startTimeSec << "s -> " << endTimeSec
              << "s, rx=" << g_curWindow.totalRxBytes << " B, "
              << "eff=" << std::setprecision(1)
              << g_curWindow.avgEffMbps << " Mbps" << std::endl;

    g_windows.push_back (g_curWindow);
}

// [MODIFIED for Step-3] Also writes to g_curWindow when a window is open.
//                       g_tput cumulative behaviour preserved for Section 16.
static void
AppTxTrace (Ptr<const Packet> p)
{
    double now = Simulator::Now ().GetSeconds ();

    if (g_tput.firstTxSec < 0.0) g_tput.firstTxSec = now;
    g_tput.totalTxBytes += p->GetSize ();

    if (g_windowOpen)
    {
        if (g_curWindow.firstTxSec < 0.0) g_curWindow.firstTxSec = now;
        g_curWindow.totalTxBytes += p->GetSize ();
    }
}

// [MODIFIED for Step-3]
static void
AppRxTrace (Ptr<const Packet> p, const Address &/*from*/)
{
    double now = Simulator::Now ().GetSeconds ();

    if (g_tput.firstRxSec < 0.0) g_tput.firstRxSec = now;
    g_tput.lastRxSec = now;
    g_tput.totalRxBytes += p->GetSize ();

    if (g_windowOpen)
    {
        g_curWindow.lastRxSec = now;
        g_curWindow.totalRxBytes += p->GetSize ();
    }

    // [Step-3] Compare per-window bytes (was g_tput.totalRxBytes). Cumulative
    //          comparison would stop the entire simulation after the FIRST
    //          window receives maxBytes, making further handoffs unreachable.
    //          Per-window comparison preserves single-window semantics exactly
    //          when no handoff occurs.
    if (g_fixedVolume && !g_stopFired
        && g_windowOpen
        && g_curWindow.totalRxBytes >= g_volumeBytes)
    {
        g_stopFired = true;
        Simulator::Stop ();
    }
}

// [Step-3] One-time hookup at sim start. PacketSinks live for the entire
//          run, so AppRxTrace is attached here exactly once. Re-running this
//          on every handoff would stack callbacks on each PacketSink → every
//          Rx event would fire N times (N = number of completed handoffs).
void
connectInitial ()
{
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Tx",
                     MakeCallback (&EchoTxRx));
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Rx",
                     MakeCallback (&EchoTxRx));
    Config::ConnectWithoutContext (
        "/NodeList/*/ApplicationList/*/$ns3::BulkSendApplication/Tx",
        MakeCallback (&AppTxTrace));
    Config::ConnectWithoutContext (
        "/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
        MakeCallback (&AppRxTrace));
}

// [Step-3] Per-handoff hookup. Hooks ONLY what the handoff just created:
//          new TCP socket on UAV (EchoTxRx) and the new BulkSend instance
//          directly (AppTxTrace). PacketSink intentionally untouched.
void
connectAfterHandoff (Ptr<Application> newBulkApp)
{
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Tx",
                     MakeCallback (&EchoTxRx));
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Rx",
                     MakeCallback (&EchoTxRx));

    if (newBulkApp)
        newBulkApp->TraceConnectWithoutContext ("Tx",
                                                MakeCallback (&AppTxTrace));
}

// Backward-compat shim — leftover call sites still work.
void
connect ()
{
    connectInitial ();
}

// ============================================================================
// Coordinate conversion (geodetic ↔ ECEF)
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

/**
 * \param gndEcef  Ground node ECEF position
 * \param satEcef  Satellite ECEF position
 * \return         Elevation angle from ground node's local horizon (deg)
 */
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
// Satellite selection
// ============================================================================

/**
 * \param satellites  Container of LEO satellite nodes
 * \param uavNode     UAV ground node
 * \param topN        Number of nearest satellites to print (debug)
 * \return            Index of the nearest satellite
 */
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
// Adaptive data rate (periodic SNR-driven DataRate update)
// ============================================================================

struct AdaptiveRateRecord
{
    double timeSec;
    double distKm;
    double elevDeg;
    double bfGainDb;
    double snrDb;
    double rateMbps;
};
std::vector<AdaptiveRateRecord> g_rateLog;

// [Step-1] Handoff scanning state
//   linkDown:        whether the current target sat has dropped below cutoff
//   candidateSatIdx: index of the best visible alternative (-1 = none yet)
//   candidateElevDeg / candidateDistKm: cached for logging and Step-2 use
struct HandoffState
{
    bool    linkDown        = false;
    int32_t candidateSatIdx = -1;
    double  candidateElevDeg = 0.0;
    double  candidateDistKm  = 0.0;
};
static HandoffState g_handoff;

// [Step-2] Simulation context shared between callbacks. Populated once in
// main() before Simulator::Run(); read-only thereafter. Avoids passing
// 6+ parameters through every Simulator::Schedule call.
struct SimContext
{
    Ptr<Node>             uavNode;
    NodeContainer         satellites;
    NetDeviceContainer    utNet;
    Ipv4InterfaceContainer utIf;
    SatBandParams         band;
    uint16_t              port      = 0;
    uint32_t              maxBytes  = 0;
    uint32_t              sendSize  = 0;
};
static SimContext g_ctx;

// [Step-2] Currently-active target. Mutated atomically inside PerformHandoff().
struct ActiveTarget
{
    Ptr<Node>      satNode;
    uint32_t       satIdx          = 0;
    Ipv4Address    ipAddr;
    double         connectTimeSec  = 0.0;
};
static ActiveTarget g_active;

// [Step-2] Forward declaration; defined after UpdateAdaptiveRate.
static void PerformHandoff ();

/**
 * Recomputes SNR / Shannon rate from current UAV-satellite geometry and
 * pushes the new rate into the MockNetDevice on both ends.
 *
 * \param uavNode  UAV node
 * \param satNode  Target satellite node
 * \param utNet    NetDeviceContainer holding both endpoints
 * \param band     Band parameters (incl. elevation cutoff)
 * \param satIdx   Index of the target satellite device in utNet
 */
// [MODIFIED] No parameters — reads g_ctx + g_active so that the *current*
//            target satellite is always honored, even after a handoff.
static void
UpdateAdaptiveRate ()
{
    Vector uavPos = g_ctx.uavNode->GetObject<MobilityModel> ()->GetPosition ();
    Vector satPos = g_active.satNode->GetObject<MobilityModel> ()->GetPosition ();
    double distKm  = EcefDistance (uavPos, satPos) / 1000.0;
    double elevDeg = ComputeElevationAngle (uavPos, satPos);

    double bfGain   = CalcBeamformingGain (elevDeg, g_ctx.band.nAnt, g_ctx.band.nRF);
    double snrDb    = CalcSNR (g_ctx.band, distKm, elevDeg);
    double rateMbps = CalcShannonCapacity (g_ctx.band, snrDb);

    if (elevDeg < g_ctx.band.elevAngleDeg)
        rateMbps = 0.001;

    // [Step-2 fix] Floor of 0.001 Mbps must NOT round to "0.0Mbps". The
    // MockNetDevice's CalculateBytesTxTime() divides by rate; a zero rate
    // causes SIGFPE the moment a packet is queued — which TCP's retransmit
    // timer eventually does during a long link-down window between handoffs.
    // (300s simulation never hit this; 7200s does.)
    std::ostringstream rateStr;
    if (rateMbps < 1.0)
        rateStr << std::fixed << std::setprecision(6) << rateMbps << "Mbps";
    else
        rateStr << std::fixed << std::setprecision(1) << rateMbps << "Mbps";

    uint32_t uavDevIdx = g_ctx.utNet.GetN () - 1;
    Ptr<MockNetDevice> uavDev = DynamicCast<MockNetDevice> (g_ctx.utNet.Get (uavDevIdx));
    if (uavDev) uavDev->SetDataRate (DataRate (rateStr.str ()));

    Ptr<MockNetDevice> satDev = DynamicCast<MockNetDevice> (g_ctx.utNet.Get (g_active.satIdx));
    if (satDev) satDev->SetDataRate (DataRate (rateStr.str ()));

    g_rateLog.push_back ({Simulator::Now ().GetSeconds (), distKm, elevDeg,
                          bfGain, snrDb, rateMbps});

    // ------------------------------------------------------------------
    // [Step-1] Link-state tracking + rescan
    // ------------------------------------------------------------------
    double now = Simulator::Now ().GetSeconds ();
    bool currentlyDown = (elevDeg < g_ctx.band.elevAngleDeg);

    if (currentlyDown && !g_handoff.linkDown)
    {
        g_handoff.linkDown = true;
        g_handoff.candidateSatIdx = -1;
        std::cerr << "[Step-1] t=" << std::fixed << std::setprecision(2) << now
                  << "s: Sat[" << g_active.satIdx << "] LINK DOWN (elev="
                  << std::setprecision(2) << elevDeg << "° < cutoff "
                  << g_ctx.band.elevAngleDeg << "°) -> entering RESCAN mode"
                  << std::endl;

        CloseWindow (now);   // [Step-3] window ends at link-down detection
    }
    else if (!currentlyDown && g_handoff.linkDown)
    {
        g_handoff.linkDown = false;
        g_handoff.candidateSatIdx = -1;
        std::cerr << "[Step-1] t=" << std::fixed << std::setprecision(2) << now
                  << "s: Sat[" << g_active.satIdx
                  << "] link recovered -> exiting RESCAN" << std::endl;
    }

    if (g_handoff.linkDown)
    {
        int32_t bestIdx = -1;
        double  bestElev = -90.0, bestDist = 0.0;

        for (uint32_t i = 0; i < g_ctx.satellites.GetN (); i++)
        {
            if (i == g_active.satIdx) continue;
            Vector pos = g_ctx.satellites.Get (i)->GetObject<MobilityModel> ()
                                                  ->GetPosition ();
            double e = ComputeElevationAngle (uavPos, pos);
            if (e >= g_ctx.band.elevAngleDeg && e > bestElev)
            {
                bestElev = e;
                bestIdx  = (int32_t) i;
                bestDist = EcefDistance (uavPos, pos) / 1000.0;
            }
        }

        if (bestIdx >= 0)
        {
            // [Step-2] Trigger handoff IMMEDIATELY upon finding a visible sat.
            //          (Step 1 only cached; Step 2 acts.)
            g_handoff.candidateSatIdx  = bestIdx;
            g_handoff.candidateElevDeg = bestElev;
            g_handoff.candidateDistKm  = bestDist;
            PerformHandoff ();
        }
        else if (g_handoff.candidateSatIdx != -1)
        {
            g_handoff.candidateSatIdx = -1;
            std::cerr << "[Step-1] t=" << std::fixed << std::setprecision(2) << now
                      << "s: candidate lost, still scanning..." << std::endl;
        }
    }
}

// [Step-2] Tear down the current TCP flow and bring up a new BulkSend
//          targeting g_handoff.candidateSatIdx.
static void
PerformHandoff ()
{
    if (g_handoff.candidateSatIdx < 0) return;

    uint32_t newIdx = (uint32_t) g_handoff.candidateSatIdx;
    Ptr<Node> newSat = g_ctx.satellites.Get (newIdx);
    Ipv4Address newAddr = newSat->GetObject<Ipv4> ()
                                ->GetAddress (1, 0).GetLocal ();
    double now = Simulator::Now ().GetSeconds ();

    std::cerr << "[Step-2] t=" << std::fixed << std::setprecision(2) << now
              << "s: HANDOFF Sat[" << g_active.satIdx << "] -> Sat[" << newIdx
              << "] (IP " << newAddr
              << ", elev=" << std::setprecision(2) << g_handoff.candidateElevDeg
              << "°, dist=" << std::setprecision(1) << g_handoff.candidateDistKm
              << " km)" << std::endl;

    // (1) Stop the existing BulkSend on the UAV. Walking ApplicationList is
    //     simpler than tracking a handle through globals; cost is trivial.
    uint32_t nApps = g_ctx.uavNode->GetNApplications ();
    for (uint32_t i = 0; i < nApps; i++)
    {
        Ptr<Application> app = g_ctx.uavNode->GetApplication (i);
        if (DynamicCast<BulkSendApplication> (app))
        {
            app->SetStopTime (Simulator::Now ());
        }
    }

    // (2) Switch the active target BEFORE installing the new app, so the
    //     re-scheduled UpdateAdaptiveRate immediately sees the new geometry.
    g_active.satNode        = newSat;
    g_active.satIdx         = newIdx;
    g_active.ipAddr         = newAddr;
    g_active.connectTimeSec = now;

    // (3) Reset handoff state so future link-down events are detected fresh.
    g_handoff.linkDown         = false;
    g_handoff.candidateSatIdx  = -1;

    BeginWindow (newIdx);      // [Step-3] new window for the new target
    g_stopFired = false;       // [Step-3] allow per-window stop (fixedVolume)

    // (4) Install a new BulkSend pointed at the new satellite.
    BulkSendHelper sender ("ns3::TcpSocketFactory",
                           InetSocketAddress (newAddr, g_ctx.port));
    sender.SetAttribute ("MaxBytes", UintegerValue (g_ctx.maxBytes));
    sender.SetAttribute ("SendSize", UintegerValue (g_ctx.sendSize));
    ApplicationContainer newApp = sender.Install (g_ctx.uavNode);
    newApp.Start (Simulator::Now ());

    // (5) Re-hook trace sources so the newly-created TCP socket and
    //     BulkSendApplication Tx events are observed. Same one-shot scheduler
    //     pattern used at simulation start (avoids racing with socket
    //     construction inside Start()).
    // [MODIFIED for Step-3] Hook ONLY the new BulkSend; PacketSink hooks are
    //                       kept from connectInitial(), preventing N-fold
    //                       callback duplication on each handoff.
    Ptr<Application> newAppPtr = newApp.Get (0);
    Simulator::Schedule (Seconds (1e-7), &connectAfterHandoff, newAppPtr);

    // (6) Re-apply adaptive rate immediately so the device DataRate reflects
    //     the new geometry without waiting for the next periodic tick.
    Simulator::Schedule (Seconds (2e-7), &UpdateAdaptiveRate);
}

// ============================================================================
// Helpers
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

NS_LOG_COMPONENT_DEFINE ("UavToLeoExample");

// ============================================================================
// Main
// ============================================================================

int
main (int argc, char *argv[])
{
    // ------------------------------------------------------------------------
    // 1. Command-line parameters
    // ------------------------------------------------------------------------
    std::string orbitFile;
    std::string traceFile;

    double uavLatDeg = 24.80;
    double uavLonDeg = 120.97;
    double uavAltM   = 300.0;

    std::string bandName = "Ku-User";
    int32_t  targetSatIndex = -1;

    uint16_t port     = 9;
    uint32_t maxBytes = 10 * 1024 * 1024;
    uint32_t sendSize = 1024;
    double   duration = 7200.0;

    uint64_t ttlThresh    = 0;
    double   routeTimeout = 300.0;
    double   rateInterval = 1.0;

    bool        pcap = false;
    std::string bpFile;
    int         nAnt = 16;
    int         nRF  = 4;
    bool        fixedVolume = true;

    // ------------------------------------------------------------------------
    // 2. Parse command line
    // ------------------------------------------------------------------------
    CommandLine cmd;
    cmd.AddValue ("orbitFile",      "CSV file with orbit parameters",         orbitFile);
    cmd.AddValue ("traceFile",      "CSV file to redirect stdout to",         traceFile);
    cmd.AddValue ("precision",      "ns3::LeoCircularOrbitMobilityModel::Precision");
    cmd.AddValue ("duration",       "Simulation duration (seconds)",          duration);
    cmd.AddValue ("uavLat",         "UAV latitude  (degrees N)",              uavLatDeg);
    cmd.AddValue ("uavLon",         "UAV longitude (degrees E)",              uavLonDeg);
    cmd.AddValue ("uavAlt",         "UAV altitude  (meters ASL)",             uavAltM);
    cmd.AddValue ("band",           "Sat band: Ku-User|Ka-Gateway|Ka-User|S-band", bandName);
    cmd.AddValue ("targetSatIndex", "Satellite index (-1 = auto-closest)",    targetSatIndex);
    cmd.AddValue ("maxBytes",       "Total bytes to send (0 = unlimited)",    maxBytes);
    cmd.AddValue ("sendSize",       "TCP segment size (bytes)",               sendSize);
    cmd.AddValue ("ttlThresh",      "AODV TTL threshold",                     ttlThresh);
    cmd.AddValue ("routeTimeout",   "AODV ActiveRouteTimeout (seconds)",      routeTimeout);
    cmd.AddValue ("rateInterval",   "Adaptive rate update interval (seconds)", rateInterval);
    cmd.AddValue ("bpFile",         "MATLAB beam pattern CSV file",            bpFile);
    cmd.AddValue ("nAnt",           "Number of antenna elements (1/4/16/64)",  nAnt);
    cmd.AddValue ("nRF",            "Number of RF chains (hybrid: nRF <= nAnt)", nRF);
    cmd.AddValue ("destOnly",       "ns3::aodv::RoutingProtocol::DestinationOnly");
    cmd.AddValue ("pcap",           "Enable PCAP packet capture",             pcap);
    cmd.AddValue ("fixedVolume",    "Stop simulation when maxBytes received "
                                    "(default true; pass false to run full duration)",
                                    fixedVolume);
    cmd.Parse (argc, argv);

    g_fixedVolume = fixedVolume;
    g_volumeBytes = maxBytes;

    // ------------------------------------------------------------------------
    // 3. TCP buffer defaults (must precede InternetStackHelper::Install)
    // ------------------------------------------------------------------------
    Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (8 * 1024 * 1024));
    Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (8 * 1024 * 1024));

    // ------------------------------------------------------------------------
    // 4. Resolve frequency band
    // ------------------------------------------------------------------------
    SatBandParams band;
    if      (bandName == "Ku-User")    band = BAND_KU_USER;
    else if (bandName == "Ka-Gateway") band = BAND_KA_GATEWAY;
    else if (bandName == "Ka-User")    band = BAND_KA_USER;
    else if (bandName == "S-band")     band = BAND_S;
    else
    {
        std::cerr << "ERROR: unknown band '" << bandName
                  << "'. Using Ku-User." << std::endl;
        band = BAND_KU_USER;
    }

    if (nAnt > 1)
    {
        band.nAnt = nAnt;
        band.nRF  = nRF;
    }
    if (band.nRF > band.nAnt) band.nRF = band.nAnt;

    std::cerr << "Band: " << band.name << " (" << band.freqGHz << " GHz, BW="
              << band.bandwidthGHz * 1000.0 << " MHz, elev cutoff="
              << band.elevAngleDeg << " deg)" << std::endl;
    if (band.nAnt > 1)
    {
        std::cerr << "Beamforming: nAnt=" << band.nAnt
                  << ", nRF=" << band.nRF << " (hybrid)" << std::endl;
    }

    if (!bpFile.empty ())
    {
        bool ok = LoadBeamPattern (bpFile);
        if (!ok)
            std::cerr << "[BF] Falling back to analytical beamforming model" << std::endl;
    }
    else if (band.nAnt > 1)
    {
        std::cerr << "[BF] No --bpFile specified, using analytical fallback "
                  << "(10*log10(N)*sin(elev)*eff)" << std::endl;
    }

    // ------------------------------------------------------------------------
    // 5. Redirect stdout (optional)
    // ------------------------------------------------------------------------
    std::streambuf *coutbuf = std::cout.rdbuf ();
    std::ofstream out;
    if (!traceFile.empty ())
    {
        out.open (traceFile);
        if (out.is_open ()) std::cout.rdbuf (out.rdbuf ());
    }

    // ------------------------------------------------------------------------
    // 6. Create LEO satellite constellation
    // ------------------------------------------------------------------------
    LeoOrbitNodeHelper orbit;
    NodeContainer satellites;
    if (!orbitFile.empty ())
        satellites = orbit.Install (orbitFile);
    else
        satellites = orbit.Install ({LeoOrbit (1200, 53, 22, 12)});

    uint32_t numSats = satellites.GetN ();
    std::cerr << "Created " << numSats << " LEO satellites" << std::endl;

    // ------------------------------------------------------------------------
    // 7. Create UAV node
    // ------------------------------------------------------------------------
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

    // ------------------------------------------------------------------------
    // 8. Select target satellite
    // ------------------------------------------------------------------------
    uint32_t autoClosest = FindClosestSatellite (satellites, mainUav, 3);
    if (targetSatIndex < 0 || (uint32_t) targetSatIndex >= numSats)
        targetSatIndex = (int32_t) autoClosest;

    Ptr<Node> targetSat = satellites.Get ((uint32_t) targetSatIndex);
    std::cerr << "  Selected: Sat[" << targetSatIndex << "]" << std::endl;

    // ------------------------------------------------------------------------
    // 9. Initial link budget + LEO channel setup
    // ------------------------------------------------------------------------
    Vector satPos = targetSat->GetObject<MobilityModel> ()->GetPosition ();
    double initDistKm  = EcefDistance (uavEcef, satPos) / 1000.0;
    double initElevDeg = ComputeElevationAngle (uavEcef, satPos);

    double fsplDb      = CalcFSPL (initDistKm, band.freqGHz);
    double snrDb       = CalcSNR (band, initDistKm, initElevDeg);
    double shannonMbps = CalcShannonCapacity (band, snrDb);

    PrintLinkBudget (band, initDistKm, initElevDeg);

    std::ostringstream dataRateStr;
    dataRateStr << std::fixed << std::setprecision(1) << shannonMbps << "Mbps";
    std::cerr << "Computed data rate: " << dataRateStr.str () << std::endl;

    LeoChannelHelper utCh;
    if (band.freqGHz > 20.0) utCh.SetConstellation ("TelesatGateway");
    else                      utCh.SetConstellation ("TelesatUser");

    utCh.SetGndDeviceAttribute ("DataRate", StringValue (dataRateStr.str ()));
    utCh.SetSatDeviceAttribute ("DataRate", StringValue (dataRateStr.str ()));
    utCh.SetGndDeviceAttribute ("TxPower",  DoubleValue (band.eirpDbm));
    utCh.SetSatDeviceAttribute ("TxPower",  DoubleValue (band.eirpDbm));
    utCh.SetGndDeviceAttribute ("RxGain",   DoubleValue (band.rxGainDbi));
    utCh.SetSatDeviceAttribute ("RxGain",   DoubleValue (band.rxGainDbi));

    NetDeviceContainer utNet = utCh.Install (satellites, uavNodes);

    // ------------------------------------------------------------------------
    // 10. Internet stack with AODV
    // ------------------------------------------------------------------------
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

    // ------------------------------------------------------------------------
    // 11. IP assignment
    // ------------------------------------------------------------------------
    Ipv4AddressHelper ipv4;
    ipv4.SetBase ("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer utIf = ipv4.Assign (utNet);

    // [MODIFIED] After resolving targetSatIndex / targetSat / targetAddr,
    //            populate the globals that callbacks rely on.
    Ipv4Address targetAddr = targetSat->GetObject<Ipv4> ()
                                 ->GetAddress (1, 0).GetLocal ();

    // [Step-2] Populate simulation context (read-only after this point)
    g_ctx.uavNode    = mainUav;
    g_ctx.satellites = satellites;
    g_ctx.utNet      = utNet;
    g_ctx.utIf       = utIf;
    g_ctx.band       = band;
    g_ctx.port       = port;
    g_ctx.maxBytes   = maxBytes;
    g_ctx.sendSize   = sendSize;

    // [Step-2] Bootstrap the active target with the initial selection
    g_active.satNode        = targetSat;
    g_active.satIdx         = (uint32_t) targetSatIndex;
    g_active.ipAddr         = targetAddr;
    g_active.connectTimeSec = 0.0;

    BeginWindow ((uint32_t) targetSatIndex);   // [Step-3] window 1

    // ------------------------------------------------------------------------
    // 12. Applications: BulkSend on UAV, PacketSink on every satellite
    // ------------------------------------------------------------------------
    BulkSendHelper sender ("ns3::TcpSocketFactory",
                           InetSocketAddress (targetAddr, port));
    sender.SetAttribute ("MaxBytes", UintegerValue (maxBytes));
    sender.SetAttribute ("SendSize", UintegerValue (sendSize));
    ApplicationContainer sourceApps = sender.Install (mainUav);
    sourceApps.Start (Seconds (0.0));

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

    // ------------------------------------------------------------------------
    // 13. Schedule trace hookup and adaptive rate updates
    // ------------------------------------------------------------------------
    Simulator::Schedule (Seconds (1e-7), &connectInitial);

    for (double t = rateInterval; t <= duration; t += rateInterval)
    {
        Simulator::Schedule (Seconds (t), &UpdateAdaptiveRate);
    }

    // ------------------------------------------------------------------------
    // 14. PCAP (optional)
    // ------------------------------------------------------------------------
    if (pcap)
    {
        AsciiTraceHelper ascii;
        utCh.EnableAsciiAll (ascii.CreateFileStream ("uav-to-leo.tr"));
        utCh.EnablePcapAll ("uav-to-leo", false);
    }

    // ------------------------------------------------------------------------
    // 15. Run simulation
    // ------------------------------------------------------------------------
    std::cerr << "\n=== Simulation: " << duration << "s, "
              << band.name << ", rate=" << dataRateStr.str ()
              << ", Sat[" << targetSatIndex << "], "
              << "adaptive interval=" << rateInterval << "s ===" << std::endl;

    NS_LOG_INFO ("Run Simulation.");
    Simulator::Stop (Seconds (duration));
    Simulator::Run ();

    // [Step-3] If the simulation ended while a window was still open (no
    //          link-down ever fired for the last sat before t=duration),
    //          close it now using the simulation end time.
    if (g_windowOpen) CloseWindow (Simulator::Now ().GetSeconds ());

    Simulator::Destroy ();

    // [Step-3] Per-window summary (CSV export comes in Step 4)
    std::cout << "\n--- Per-Window Effective Throughput ---" << std::endl;
    std::cout << std::left
              << std::setw(4)  << "id"   << std::setw(8)  << "sat"
              << std::setw(12) << "start" << std::setw(12) << "end"
              << std::setw(11) << "dur"   << std::setw(13) << "rx_bytes"
              << std::setw(11) << "eff_Mbps" << std::endl;
    for (auto &w : g_windows)
    {
        std::cout << std::left << std::fixed << std::setprecision(2)
                  << std::setw(4)  << w.windowId
                  << std::setw(8)  << w.satIdx
                  << std::setw(12) << w.startTimeSec
                  << std::setw(12) << w.endTimeSec
                  << std::setw(11) << (w.endTimeSec - w.startTimeSec)
                  << std::setw(13) << w.totalRxBytes
                  << std::setw(11) << std::setprecision(1) << w.avgEffMbps
                  << std::endl;
    }

    // ------------------------------------------------------------------------
    // 16. Compute summary metrics
    // ------------------------------------------------------------------------
    Ptr<PacketSink> pktSink = DynamicCast<PacketSink> (sinkApps.Get (0));
    uint64_t totalRx = pktSink->GetTotalRx ();

    double initBfGain = CalcBeamformingGain (initElevDeg, band.nAnt, band.nRF);

    // Per-packet delay stats
    double avgDelayMs = 0.0, minDelayMs = 0.0, maxDelayMs = 0.0;
    int    delayCount = 0;
    if (!delay.empty ())
    {
        double total = 0.0, mn = 1e9, mx = 0.0;
        for (auto &[uid, d] : delay)
        {
            total += d;
            if (d < mn) mn = d;
            if (d > mx) mx = d;
        }
        delayCount  = (int) delay.size ();
        avgDelayMs  = (total / delayCount) * 1000.0;
        minDelayMs  = mn * 1000.0;
        maxDelayMs  = mx * 1000.0;
    }

    // Effective throughput (application-layer, computed from Tx/Rx hooks)
    double effMbps     = 0.0;
    double measSec     = 0.0;
    double shannonRef  = shannonMbps;
    double efficiency  = 0.0;
    bool   tputValid   = (g_tput.firstTxSec >= 0.0
                          && g_tput.lastRxSec > g_tput.firstTxSec
                          && g_tput.totalRxBytes > 0);
    if (tputValid)
    {
        measSec = g_tput.lastRxSec - g_tput.firstTxSec;
        effMbps = (g_tput.totalRxBytes * 8.0) / (measSec * 1e6);

        double sum = 0.0;
        int    count = 0;
        for (auto &r : g_rateLog)
        {
            if (r.timeSec >= g_tput.firstTxSec
                && r.timeSec <= g_tput.lastRxSec
                && r.rateMbps > 0.001)
            {
                sum += r.rateMbps;
                count++;
            }
        }
        if (count > 0) shannonRef = sum / count;
        if (shannonRef > 0.0) efficiency = effMbps / shannonRef * 100.0;
    }

    // Visible time window (first OK→DOWN transition observed in g_rateLog)
    double visStart = -1.0, visEnd = -1.0;
    for (auto &r : g_rateLog)
    {
        bool down = (r.elevDeg < band.elevAngleDeg);
        if (!down && visStart < 0)      visStart = r.timeSec;
        else if (down && visEnd  < 0)   visEnd   = r.timeSec;
    }

    // ------------------------------------------------------------------------
    // 17. Terminal output
    // ------------------------------------------------------------------------
    std::cout << "\n========== UAV-to-LEO Simulation Results ==========" << std::endl;
    std::cout << "UAV node " << mainUav->GetId ()
              << " -> Sat[" << targetSatIndex << "] node "
              << targetSat->GetId () << " (IP " << targetAddr << ")" << std::endl;
    std::cout << "Band:           " << band.name
              << " (" << band.freqGHz << " GHz, BW=" << band.bandwidthGHz * 1000.0
              << " MHz)" << std::endl;

    if (band.nAnt > 1)
    {
        std::cout << "Beamforming:    nAnt=" << band.nAnt << " nRF=" << band.nRF
                  << " → gain=" << std::fixed << std::setprecision(2) << initBfGain
                  << " dB at elev " << initElevDeg << "°";
        if (!g_beamPatterns.empty ())
            std::cout << " (CSV, steering=" << GetSteeringSector (initElevDeg)
                      << "°)" << std::endl;
        else
            std::cout << " (analytical fallback)" << std::endl;
    }
    else
    {
        std::cout << "Beamforming:    none (single antenna)" << std::endl;
    }

    std::cout << "Elev cutoff:    " << band.elevAngleDeg
              << " deg  <-- decides LINK DOWN (from " << band.name << " band spec)"
              << std::endl;
    std::cout << "Init link:      dist=" << std::fixed << std::setprecision(1)
              << initDistKm << " km, elev=" << initElevDeg
              << " deg, FSPL=" << fsplDb << " dB, SNR=" << snrDb
              << " dB, Shannon=" << shannonMbps << " Mbps" << std::endl;
    std::cout << "Duration:       " << duration << " s" << std::endl;
    std::cout << "Bytes:          " << totalRx << " / " << maxBytes << " received"
              << std::endl;

    if (duration > 0)
    {
        double avgMbps = (totalRx * 8.0) / (duration * 1e6);
        std::cout << "Avg throughput: " << avgMbps << " Mbps" << std::endl;
    }

    if (delayCount > 0)
    {
        std::cout << "Delay:          avg=" << avgDelayMs
                  << " ms, min=" << minDelayMs
                  << " ms, max=" << maxDelayMs
                  << " ms (" << delayCount << " pkts)" << std::endl;
    }

    std::cout << "\n--- Visible Time Window ---" << std::endl;
    std::cout << "Cutoff:              " << std::fixed << std::setprecision(1)
              << band.elevAngleDeg << " deg" << std::endl;
    std::cout << "Rate-log interval:   " << rateInterval << " s" << std::endl;
    std::cout << "Window start:        " << visStart << " s" << std::endl;
    std::cout << "Window end:          " << visEnd   << " s" << std::endl;

    std::cout << "\n--- Effective Throughput Measurement ---" << std::endl;
    std::cout << "Mode:                "
              << (g_fixedVolume ? "fixed-volume (stop on maxBytes)"
                                : "fixed-time (run full duration)")
              << std::endl;

    if (tputValid)
    {
        std::cout << std::fixed;
        std::cout << "Total Tx bytes:      " << g_tput.totalTxBytes << std::endl;
        std::cout << "Total Rx bytes:      " << g_tput.totalRxBytes << std::endl;
        std::cout << "First Tx time:       " << std::setprecision(6)
                                              << g_tput.firstTxSec << " s" << std::endl;
        std::cout << "Last  Rx time:       " << g_tput.lastRxSec  << " s" << std::endl;
        std::cout << "Measured duration:   " << std::setprecision(3)
                                              << measSec * 1000.0 << " ms" << std::endl;
        std::cout << "Effective throughput:" << std::setprecision(3)
                                              << effMbps << " Mbps" << std::endl;
        if (shannonRef > 0.0)
        {
            std::cout << "Shannon (theoretical):"
                      << std::setprecision(3) << shannonRef << " Mbps" << std::endl;
            std::cout << "Efficiency (eff/Shannon): "
                      << std::setprecision(1) << efficiency << " %" << std::endl;
        }
    }
    else
    {
        std::cout << "No effective Tx/Rx recorded "
                  << "(link may have been DOWN, or sim ended too early)."
                  << std::endl;
    }

    std::cout << "=====================================================" << std::endl;

    // ------------------------------------------------------------------------
    // 18. CSV outputs
    // ------------------------------------------------------------------------
    const std::string outputDir = "outputs";
    if (!std::filesystem::exists (outputDir))
        std::filesystem::create_directories (outputDir);

    // 18a. Per-interval adaptive rate log
    if (!g_rateLog.empty ())
    {
        std::ofstream rateLogFile (outputDir + "/adaptiveRateLog.csv");
        rateLogFile << "Adaptive Rate Log (interval=" << rateInterval << "s)" << std::endl;
        rateLogFile << "Time,Dist(km),Elev,BF(dB),SNR,Rate(Mbps),Status" << std::endl;
        for (auto &r : g_rateLog)
        {
            bool down = (r.elevDeg < band.elevAngleDeg);
            rateLogFile << std::fixed << std::setprecision(2)
                        << std::setw(6)  << r.timeSec << "s,"
                        << std::setw(10) << r.distKm  << ","
                        << std::setw(8)  << r.elevDeg << "°,"
                        << std::setw(7)  << r.bfGainDb << ","
                        << std::setw(8)  << r.snrDb << " dB,"
                        << std::setw(11) << r.rateMbps << ","
                        << "  " << (down ? "[DOWN <" : "[OK   >=")
                        << band.elevAngleDeg << "°]" << std::endl;
        }
        rateLogFile.close ();
    }

    // 18b. Single-row summary of headline metrics
    {
        std::ofstream resultFile (outputDir + "/uav-to-leo_result.csv");
        resultFile << "Effective Throughput (Mbps),"
                   << "Elevation Cutoff (Deg),"
                   << "Visible Time Window Start (s),"
                   << "Visible Time Window End (s)" << std::endl;
        resultFile << std::fixed << std::setprecision(6)
                   << effMbps           << ","
                   << band.elevAngleDeg << ","
                   << visStart          << ","
                   << visEnd            << std::endl;
        resultFile.close ();
    }

    if (out.is_open ())
    {
        out.close ();
        std::cout.rdbuf (coutbuf);
    }
    return 0;
}