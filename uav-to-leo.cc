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
 *
 * [UPDATE-3] Hybrid beamforming via MATLAB-generated steering-aware CSV lookup。
 *   - MATLAB (beamforming.m) 對 steering 角度 θ_s = 0°, 10°, ..., 90° 各算一份
 *     ULA array factor + element pattern + hybrid loss → 9010 行三欄 CSV：
 *       steering_deg, elevation_deg, gain_dB
 *   - ns-3 啟動時讀入 CSV，建立 steering→elev→gain 兩層 lookup table。
 *   - 模擬執行中：依即時仰角 (1) 四捨五入到最近 10° 找 steering sector，
 *     (2) 在該 sector 內查精確仰角的 gain。
 *   這正確表達「主瓣對準衛星、N 支天線建設性干涉」的物理效應，
 *   讓 BF gain 真正提升 SNR（峰值約 +15 dB at N=16），而非舊版 broadside-only
 *   pattern 帶來的負增益問題。
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

static const double EARTH_RADIUS = 6.37101e6;  // meters (same as LEO_PROP_EARTH_RAD)

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
    double elevAngleDeg;
    int    nAnt;
    int    nRF;                // [UPDATE-3] RF chain 數 (hybrid: nRF < nAnt)
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
    1,                         // [UPDATE-3] nAnt: 1 (single antenna, no array)
    1                          // [UPDATE-3] nRF:  1 (single RF chain)
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
    1, 1                       // [UPDATE-3] nAnt=1, nRF=1
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
    1, 1                       // [UPDATE-3] nAnt=1, nRF=1
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
    1, 1                       // [UPDATE-3] nAnt=1, nRF=1
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

// ============================================================================
// [UPDATE-3 v2] Beamforming Gain — MATLAB steering-aware beam pattern CSV
// ============================================================================
//
// 教授指示的架構（修正版）：
//   1. MATLAB (beamforming.m) 對每個 steering 角度 θ_s = 0°,10°,…,90° 各跑一次
//      ULA array factor 計算（以該方向為主瓣指向），匯出 steering-aware CSV
//   2. ns-3 啟動時讀入 CSV，建立 (steering → elev → gain) 兩層 lookup table
//   3. 模擬期間每次查表：
//      Step 1：round(elev / 10) × 10  → 找最近的 steering sector
//      Step 2：在該 sector 的 sub-table 內查精確仰角的 gain
//
// 為什麼這樣設計？
//   - 舊版 CSV 只記錄一份「broadside (θ_s=0) 的 array factor 圖形」，
//     沒有 beam steering 概念，導致實際 BF gain 永遠是負值（旁瓣區域），
//     反而拖低 SNR。修正後 CSV 記錄各 steering 方向下的圖形，主瓣對準
//     衛星時 gain 為正值（峰值 ≈ 10·log10(N) + element_max + η_hybrid）。
//   - 10° sector 量化也呼應實際相位陣列的 codebook-based beam steering
//     （類比 phase shifter 通常是 2–3 bit 量化）。
//
// CSV 格式（由 beamforming.m 產生）：
//   # Generated by beamforming.m
//   # nAnt=16, nRF=4, freq=13.5GHz, d=0.50*lambda, hybrid_loss=-1.5dB
//   steering_deg,elevation_deg,gain_dB
//   0,0.0,3.5012
//   0,0.1,3.4987
//   ...
//   90,90.0,15.4321
//
// 如果沒有提供 CSV 檔（--bpFile 為空），使用 nAnt/nRF 參數做簡化計算作為 fallback。

/// [UPDATE-3 v2] Steering-aware beam pattern：g_beamPatterns[steering_deg][elev*10] = gain_dB
///   外層 key：steering 角度（0, 10, 20, ..., 90）
///   內層 key：觀察仰角 × 10（0, 1, 2, ..., 900）
std::map<int, std::map<int, double>> g_beamPatterns;

/// [UPDATE-3 v2] CSV metadata（從註解行擷取，僅供顯示）
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
 * \brief 讀取 MATLAB 匯出的 steering-aware beam pattern CSV
 *
 * CSV 格式（三欄）：
 *   # Generated by beamforming.m
 *   # nAnt=16, nRF=4, freq=13.5GHz, d=0.50*lambda, hybrid_loss=-1.5dB
 *   steering_deg,elevation_deg,gain_dB
 *   0,0.0,3.5012
 *   ...
 *   90,90.0,15.4321
 *
 * 內部資料結構：
 *   g_beamPatterns[steeringDeg][elevKey] = gainDb
 *   - steeringDeg ∈ {0, 10, 20, ..., 90}
 *   - elevKey = round(elevDeg × 10) ∈ {0, 1, ..., 900}
 *
 * 也會偵測舊版 2 欄 CSV 並回報錯誤（要求重新跑 beamforming.m）。
 *
 * \param filename  CSV 檔路徑
 * \return          true = 讀取成功
 */
static bool
LoadBeamPattern (const std::string &filename)
{
    std::ifstream file (filename);
    if (!file.is_open ())
    {
        std::cerr << "[UPDATE-3] WARNING: Cannot open '" << filename << "'" << std::endl;
        return false;
    }

    std::string line;
    int dataCount = 0;
    bool sawHeader = false;

    while (std::getline (file, line))
    {
        if (line.empty ()) continue;

        // ---- 註解行（# 開頭）：擷取 metadata ----
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

        // ---- 第一個非註解行：欄位標頭 ----
        if (!sawHeader)
        {
            sawHeader = true;
            if (line.find ("steering_deg") == std::string::npos)
            {
                std::cerr << "[UPDATE-3] ERROR: '" << filename
                          << "' is OLD 2-column format. Regenerate with beamforming.m"
                          << std::endl;
                file.close ();
                return false;
            }
            continue;
        }

        // ---- 資料行：steering_deg,elevation_deg,gain_dB ----
        size_t c1 = line.find (',');
        if (c1 == std::string::npos) continue;
        size_t c2 = line.find (',', c1 + 1);
        if (c2 == std::string::npos) continue;

        double steeringDeg = std::stod (line.substr (0, c1));
        double elevDeg     = std::stod (line.substr (c1 + 1, c2 - c1 - 1));
        double gainDb      = std::stod (line.substr (c2 + 1));

        int steeringKey = (int) round (steeringDeg);     // 0, 10, 20, ..., 90
        int elevKey     = (int) round (elevDeg * 10.0);  // 0, 1, ..., 900

        g_beamPatterns[steeringKey][elevKey] = gainDb;
        dataCount++;
    }
    file.close ();

    if (g_beamPatterns.empty ())
    {
        std::cerr << "[UPDATE-3] ERROR: no data rows parsed from '" << filename << "'"
                  << std::endl;
        return false;
    }

    g_bfMeta.loaded = true;

    // ---- 顯示載入摘要 ----
    std::cerr << "[UPDATE-3] Loaded steering-aware beam pattern: "
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

    // 印出每個 steering sector 的峰值 gain（位於 obs = θ_s）作 sanity check
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
 * \brief 在單一 steering sector 的 sub-table 內查詢指定仰角的 gain
 *
 * 精確匹配優先；找不到時在相鄰兩筆之間線性內插。
 *
 * \param pattern   該 sector 的 elev_key → gain_dB map
 * \param elevDeg   觀察仰角 (deg)
 * \return          gain (dB)
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

    // 線性內插
    auto upper = pattern.lower_bound (elevKey);
    if (upper == pattern.end ())   return pattern.rbegin ()->second;
    if (upper == pattern.begin ())  return upper->second;
    auto lower = std::prev (upper);
    double frac = (double)(elevKey - lower->first)
                / (double)(upper->first - lower->first);
    return lower->second + frac * (upper->second - lower->second);
}

/**
 * \brief 兩階段 + sector 邊界平滑混合的 beamforming gain lookup
 *
 * 問題背景：
 *   純 round() 選 sector 會在邊界（如 75°：80° vs 70°）產生突變，
 *   因為兩份 pattern 在邊界附近的旁瓣分布不同，造成 BF gain 非單調震盪。
 *
 * 解法：Sector 邊界 ±BLEND_HALF_DEG 範圍內做加權線性混合：
 *   - 若 elev 遠離邊界（>BLEND_HALF_DEG）→ 純單一 sector，行為與舊版一致
 *   - 若 elev 在邊界 ±BLEND_HALF_DEG 內 → 兩個相鄰 sector 依距離加權混合
 *
 *   例：BLEND_HALF_DEG = 3, elev = 73.2°（距 70°/80° 邊界 75° 僅 1.8°）
 *     weight_80 = (3 - 1.8) / (2*3) = 0.2  → 80° sector 貢獻 20%
 *     weight_70 = 1 - 0.2 = 0.8             → 70° sector 貢獻 80%
 *     gain = 0.2 * gain_80(73.2°) + 0.8 * gain_70(73.2°)
 *
 *   注意：混合在線性功率域（mW）進行，再轉回 dB，避免 dB 域線性混合的偏差。
 *
 * Step 1：定位相鄰的兩個 sector（lower_sector, upper_sector）
 * Step 2：計算混合權重
 * Step 3：各自 lookup 後在功率域加權，轉回 dB
 *
 * 若 CSV 未載入：fallback 到 analytical 公式
 *   gain = 10·log10(N · sin(elev) · η_hybrid)
 *
 * \param elevDeg   UAV 看衛星的仰角 (deg)
 * \param nAnt      天線元素數（fallback 用）
 * \param nRF       RF chain 數（fallback 用）
 * \return          Beamforming gain (dB)
 */
static double
CalcBeamformingGain (double elevDeg, int nAnt, int nRF)
{
    if (nAnt <= 1) return 0.0;

    // === 有 CSV：兩階段 lookup + sector 邊界平滑混合 ===
    if (!g_beamPatterns.empty ())
    {
        // ---- 邊界混合帶寬（單邊）：±3° 範圍內進行跨 sector 混合 ----
        static const double BLEND_HALF_DEG = 3.0;

        // 每個 sector 的中心在 0, 10, 20, ..., 90
        // 相鄰兩 sector 的邊界在 5, 15, 25, ..., 85
        // → 找出 elev 最近的邊界：nearest_boundary = round(elev/10 - 0.5)*10 + 5
        double nearestBoundary = std::round (elevDeg / 10.0 - 0.5) * 10.0 + 5.0;
        nearestBoundary = std::max (5.0, std::min (85.0, nearestBoundary));

        double distToBoundary = std::abs (elevDeg - nearestBoundary);

        // ---- 確認邊界兩側的 sector ----
        // lower_sector = sector 編號 < nearestBoundary，upper_sector > nearestBoundary
        int lowerSector = (int) std::floor (nearestBoundary / 10.0) * 10;  // e.g. 70 for boundary=75
        int upperSector = lowerSector + 10;                                  // e.g. 80

        // Clamp to valid range
        lowerSector = std::max (0,  std::min (90, lowerSector));
        upperSector = std::max (0,  std::min (90, upperSector));

        auto lookupSector = [&](int sector) -> double {
            auto it = g_beamPatterns.find (sector);
            if (it == g_beamPatterns.end ())
            {
                // 找不到精確 sector，退而找最近的
                it = g_beamPatterns.lower_bound (sector);
                if (it == g_beamPatterns.end ())
                    it = std::prev (g_beamPatterns.end ());
            }
            return LookupGainInSector (it->second, elevDeg);
        };

        double gainDb;

        if (distToBoundary >= BLEND_HALF_DEG)
        {
            // ---- 遠離邊界：純單一 sector ----
            // 判斷 elev 落在哪個 sector
            int sector = (int) round (elevDeg / 10.0) * 10;
            if (sector < 0)  sector = 0;
            if (sector > 90) sector = 90;
            gainDb = lookupSector (sector);
        }
        else
        {
            // ---- 邊界混合帶：對兩個相鄰 sector 加權 ----
            // weight_upper = 在 upper_sector 側的比例
            // distToBoundary 越大（越靠近某 sector 中心）→ 該 sector 權重越高
            // 規則：elev > boundary → 偏向 upper_sector；elev < boundary → 偏向 lower_sector
            double weight_upper, weight_lower;
            if (elevDeg >= nearestBoundary)
            {
                // 在 upper_sector 那側
                weight_upper = 0.5 + (distToBoundary / (2.0 * BLEND_HALF_DEG));
                weight_lower = 1.0 - weight_upper;
            }
            else
            {
                // 在 lower_sector 那側
                weight_lower = 0.5 + (distToBoundary / (2.0 * BLEND_HALF_DEG));
                weight_upper = 1.0 - weight_lower;
            }

            // 在功率域混合（避免 dB 域線性混合的非線性誤差）
            double gainLower = lookupSector (lowerSector);
            double gainUpper = lookupSector (upperSector);
            double powLower  = pow (10.0, gainLower / 10.0);
            double powUpper  = pow (10.0, gainUpper / 10.0);
            double powBlend  = weight_lower * powLower + weight_upper * powUpper;
            gainDb = 10.0 * log10 (powBlend);
        }

        return gainDb;
    }

    // === 沒 CSV：fallback 到簡化 analytical 公式 ===
    //   gain ≈ 10·log10(N · sin(elev) · η_hybrid)
    double elevRad = elevDeg * M_PI / 180.0;
    double taper = sin (elevRad);
    if (taper < 0.1) taper = 0.1;
    double hybridEff = (nRF < nAnt) ? (0.85 + 0.15 * ((double) nRF / nAnt)) : 1.0;
    return 10.0 * log10 ((double) nAnt * taper * hybridEff);
}

/**
 * \brief 根據仰角找出對應的 steering sector（供顯示用）
 *
 * \param elevDeg   觀察仰角 (deg)
 * \return          最近的 10° sector（0, 10, 20, ..., 90）
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
 * \brief 計算接收端 SNR
 *
 * Link budget:
 *   bfGain  = CalcBeamformingGain(elev, nAnt, nRF)  [UPDATE-3]
 *   Rx Power (dBm) = EIRP + bfGain - FSPL - atmLoss + rxGain - rxLoss - linkMargin
 *   Noise (dBm)    = -228.6 + 10*log10(T) + 10*log10(B_Hz) + 30
 *   SNR (dB)       = Rx Power - Noise
 *
 * [UPDATE-3] bfGain 不再是固定值，而是由 CalcBeamformingGain() 根據
 *   天線數 (nAnt)、RF chain 數 (nRF)、頻率、即時仰角動態計算。
 *   仰角高 → element taper 大 → gain 高；仰角低 → gain 降低。
 *
 * \param band      頻段參數（含 nAnt, nRF）
 * \param distKm    斜距 (km)
 * \param elevDeg   當前仰角 (deg)，用於計算 beamforming gain
 * \return          SNR (dB)
 */
static double
CalcSNR (const SatBandParams &band, double distKm, double elevDeg = 90.0)
{
    double fspl = CalcFSPL (distKm, band.freqGHz);

    // [UPDATE-3] 根據仰角查表（或 fallback 計算）取得 beamforming gain
    double bfGain = CalcBeamformingGain (elevDeg, band.nAnt, band.nRF);

    // Received power (dBm)
    double rxPowerDbm = band.eirpDbm
                        + bfGain            // [UPDATE-3] 動態 beamforming gain
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
    double bfGain = CalcBeamformingGain (elevDeg, band.nAnt, band.nRF);
    double snrDb = CalcSNR (band, distKm, elevDeg);
    double capacityMbps = CalcShannonCapacity (band, snrDb);

    std::cerr << "\n  Link Budget: " << band.name
              << " | " << band.freqGHz << " GHz, BW=" << band.bandwidthGHz * 1000.0 << " MHz"
              << " | EIRP=" << band.eirpDbm << " dBm";
    if (band.nAnt > 1)
    {
        std::cerr << "\n  [UPDATE-3] Array: " << band.nAnt << " elements, "
                  << band.nRF << " RF chains"
                  << (band.nRF < band.nAnt ? " (hybrid)" : " (full digital)")
                  << ", BF gain=" << std::fixed << std::setprecision(1)
                  << bfGain << " dB at elev=" << elevDeg << " deg";
        if (!g_beamPatterns.empty ())
        {
            int sect = GetSteeringSector (elevDeg);
            std::cerr << " (CSV, steering=" << sect << "°)";
        }
        else
        {
            std::cerr << " (analytical fallback)";
        }
    }
    std::cerr << "\n               dist=" << distKm << " km, elev=" << elevDeg << " deg"
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

// ============================================================================
// [UPDATE-4] Effective throughput measurement (application-layer hooks)
// ============================================================================
// 教授指示：data rate (Shannon capacity) 是理論上限，實際 throughput 受
//   header / ACK / idle gap / TCP ramp-up 影響，必須由實測得出。
//
//   effective throughput = totalRxBytes * 8 / (lastRxSec - firstTxSec)
//
// firstTxSec : BulkSend 第一次把 segment 推進 TCP socket buffer 的時刻
// lastRxSec  : PacketSink 收到最後一段 application payload 的時刻
// 區間內天然包含了 header overhead、TCP 等待、ACK 往返等所有「非純資料」時間。

struct ThroughputRecord
{
    double   firstTxSec  = -1.0;
    double   firstRxSec  = -1.0;
    double   lastRxSec   = -1.0;
    uint64_t totalTxBytes = 0;
    uint64_t totalRxBytes = 0;
};
static ThroughputRecord g_tput;

// fixed-volume 模式：收到 g_volumeBytes 後立即停止模擬
static bool     g_fixedVolume = false;
static uint64_t g_volumeBytes = 0;
static bool     g_stopFired   = false;

// BulkSend Tx trace：每次 application 將一段 sendSize 推進 socket
static void
AppTxTrace (Ptr<const Packet> p)
{
    double now = Simulator::Now ().GetSeconds ();
    if (g_tput.firstTxSec < 0.0)
        g_tput.firstTxSec = now;
    g_tput.totalTxBytes += p->GetSize ();
}

// PacketSink Rx trace：每次 sink 收到一段 TCP payload
static void
AppRxTrace (Ptr<const Packet> p, const Address &/*from*/)
{
    double now = Simulator::Now ().GetSeconds ();
    if (g_tput.firstRxSec < 0.0)
        g_tput.firstRxSec = now;
    g_tput.lastRxSec = now;
    g_tput.totalRxBytes += p->GetSize ();

    // [UPDATE-4] fixed-volume mode: stop sim once target bytes are received
    if (g_fixedVolume && !g_stopFired
        && g_tput.totalRxBytes >= g_volumeBytes)
    {
        g_stopFired = true;
        Simulator::Stop ();   // 立刻停止
    }
}

void
connect ()
{
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Tx",
                     MakeCallback (&EchoTxRx));
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Rx",
                     MakeCallback (&EchoTxRx));

    // [UPDATE-4] 新增：application 層 (effective throughput)
    Config::ConnectWithoutContext (
        "/NodeList/*/ApplicationList/*/$ns3::BulkSendApplication/Tx",
        MakeCallback (&AppTxTrace));
    Config::ConnectWithoutContext (
        "/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
        MakeCallback (&AppRxTrace));
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
    double bfGainDb;   // [UPDATE-3] beamforming gain at this elevation
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

    // [UPDATE-3] 根據即時仰角查表取得 beamforming gain
    double bfGain = CalcBeamformingGain (elevDeg, band.nAnt, band.nRF);

    // 計算即時 SNR 和 Shannon rate
    // [UPDATE-3] CalcSNR 內部也會呼叫 CalcBeamformingGain
    double snrDb = CalcSNR (band, distKm, elevDeg);
    double rateMbps = CalcShannonCapacity (band, snrDb);

    // 如果仰角低於門檻，link 不可用，設最低 rate
    if (elevDeg < band.elevAngleDeg)
    {
        rateMbps = 0.001;
    }

    // 轉成 DataRate 字串
    std::ostringstream rateStr;
    rateStr << std::fixed << std::setprecision(1) << rateMbps << "Mbps";

    // 更新 UAV 和 satellite 的 device DataRate
    uint32_t uavDevIdx = utNet.GetN () - 1;
    Ptr<MockNetDevice> uavDev = DynamicCast<MockNetDevice> (utNet.Get (uavDevIdx));
    if (uavDev)
        uavDev->SetDataRate (DataRate (rateStr.str ()));

    Ptr<MockNetDevice> satDev = DynamicCast<MockNetDevice> (utNet.Get (satIdx));
    if (satDev)
        satDev->SetDataRate (DataRate (rateStr.str ()));

    // [UPDATE-3] 記錄包含 BF gain
    g_rateLog.push_back ({Simulator::Now ().GetSeconds (), distKm, elevDeg,
                          bfGain, snrDb, rateMbps});
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

    // [UPDATE-3] Beamforming parameters
    std::string bpFile;    // MATLAB beam pattern CSV file path (empty = use fallback)
    // int nAnt = 16;         // 天線元素數（例如 16 = 4×4 UPA），--nAnt 可覆寫
    // int nRF  = 4;          // RF chain 數（hybrid: nRF < nAnt），--nRF 可覆寫

    // [UPDATE-3] Hybrid beamforming: antenna array configuration
    // nAnt = total antenna elements (e.g. 16 for 4x4 UPA)
    // nRF  = number of RF chains (hybrid: nRF < nAnt)
    // Default: 16 antennas, 4 RF chains (hybrid BF — typical UAV phased-array)
    int nAnt = 16;
    int nRF  = 4;

    // [UPDATE-4] Throughput measurement mode
    bool fixedVolume = false;   // true = stop on receiving maxBytes

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
    cmd.AddValue ("bpFile",          "MATLAB beam pattern CSV file",            bpFile);
    cmd.AddValue ("nAnt",            "Number of antenna elements (1/4/16/64)",  nAnt);
    cmd.AddValue ("nRF",             "Number of RF chains (hybrid: nRF <= nAnt)", nRF);
    cmd.AddValue ("destOnly",        "ns3::aodv::RoutingProtocol::DestinationOnly");
    cmd.AddValue ("pcap",            "Enable PCAP packet capture",             pcap);
    cmd.AddValue ("fixedVolume",     "Stop simulation when maxBytes received "
                                     "(false = fixed duration)",   fixedVolume);
    cmd.Parse (argc, argv);

    // [UPDATE-4] 把 CLI flag 同步到 trace callback 用的 globals
    g_fixedVolume = fixedVolume;
    g_volumeBytes = maxBytes;

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

    // [UPDATE-3] 套用命令列指定的天線配置
    if (nAnt > 1)
    {
        band.nAnt = nAnt;
        band.nRF  = nRF;
    }
    // 確保 nRF <= nAnt
    if (band.nRF > band.nAnt) band.nRF = band.nAnt;

    std::cerr << "Band: " << band.name << " (" << band.freqGHz << " GHz, BW="
              << band.bandwidthGHz * 1000.0 << " MHz, elev cutoff="
              << band.elevAngleDeg << " deg)"
              << std::endl;
    // [UPDATE-3] 顯示 beamforming 設定
    if (band.nAnt > 1)
    {
        std::cerr << "[UPDATE-3] Beamforming: nAnt=" << band.nAnt
                  << ", nRF=" << band.nRF
                  << " (hybrid)" << std::endl;
    }

    // [UPDATE-3] 讀取 MATLAB beam pattern CSV（如果有指定）
    if (!bpFile.empty ())
    {
        bool ok = LoadBeamPattern (bpFile);
        if (!ok)
        {
            std::cerr << "[UPDATE-3] Falling back to analytical beamforming model" << std::endl;
        }
    }
    else if (band.nAnt > 1)
    {
        std::cerr << "[UPDATE-3] No --bpFile specified, using analytical fallback "
                  << "(10*log10(N)*sin(elev)*eff)" << std::endl;
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
    // [UPDATE-3] 傳入初始仰角，讓 beamforming gain 考慮 element taper
    double snrDb  = CalcSNR (band, initDistKm, initElevDeg);
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

    // [UPDATE-3 v2] 顯示 beamforming gain（由 CSV lookup 或 analytical fallback）
    double initBfGain = CalcBeamformingGain (initElevDeg, band.nAnt, band.nRF);
    if (band.nAnt > 1)
    {
        std::cout << "Beamforming:    nAnt=" << band.nAnt << " nRF=" << band.nRF
                  << " → gain=" << std::fixed << std::setprecision(2) << initBfGain
                  << " dB at elev " << initElevDeg << "°";
        if (!g_beamPatterns.empty ())
        {
            int sect = GetSteeringSector (initElevDeg);
            std::cout << " (CSV, steering=" << sect << "°)" << std::endl;
        }
        else
        {
            std::cout << " (analytical fallback)" << std::endl;
        }
    }
    else
    {
        std::cout << "Beamforming:    none (single antenna)" << std::endl;
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
              << " dB, Shannon=" << shannonMbps << " Mbps" << std::endl;
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
        std::cout << "  Time   Dist(km)   Elev   BF(dB)   SNR    Rate(Mbps)  Status" << std::endl;
        for (auto &r : g_rateLog)
        {
            bool down = (r.elevDeg < band.elevAngleDeg);
            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(6) << r.timeSec << "s"
                      << std::setw(10) << r.distKm
                      << std::setw(8) << r.elevDeg << "°"
                      << std::setw(7) << r.bfGainDb
                      << std::setw(8) << r.snrDb << " dB"
                      << std::setw(11) << r.rateMbps
                      << "  " << (down ? "[DOWN <" : "[OK   >=")
                      << band.elevAngleDeg << "°]"
                      << std::endl;
        }
    }

    // ========================================================================
    // [UPDATE-4] Effective throughput measurement results
    // ========================================================================
    std::cout << "\n--- Effective Throughput Measurement ---" << std::endl;
    std::cout << "Mode:                "
              << (g_fixedVolume ? "fixed-volume (stop on maxBytes)"
                                : "fixed-time (run full duration)")
              << std::endl;

    if (g_tput.firstTxSec >= 0.0
        && g_tput.lastRxSec  >  g_tput.firstTxSec
        && g_tput.totalRxBytes > 0)
    {
        double measSec = g_tput.lastRxSec - g_tput.firstTxSec;
        double effMbps = (g_tput.totalRxBytes * 8.0) / (measSec * 1e6);

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

        // [UPDATE-4 fix] Shannon reference must come from the *transmission window*,
        // not the end of simulation. For a typical 10 MB transfer (~0.7s) the window
        // is shorter than rateInterval (10s), so g_rateLog has no entry inside it
        // → fall back to the pre-sim initial Shannon (shannonMbps, t=0).
        // For longer transfers spanning multiple adaptive updates, average the
        // valid (non-DOWN) entries that fall inside [firstTxSec, lastRxSec].
        double shannonRef = shannonMbps;
        {
            double sum = 0.0;
            int    count = 0;
            for (auto &r : g_rateLog)
            {
                if (r.timeSec >= g_tput.firstTxSec
                    && r.timeSec <= g_tput.lastRxSec
                    && r.rateMbps > 0.001)        // skip link-DOWN entries
                {
                    sum += r.rateMbps;
                    count++;
                }
            }
            if (count > 0) shannonRef = sum / count;
        }

        if (shannonRef > 0.0)
        {
            std::cout << "Shannon (theoretical):"
                      << std::setprecision(3) << shannonRef << " Mbps" << std::endl;
            std::cout << "Efficiency (eff/Shannon): "
                      << std::setprecision(1) << (effMbps / shannonRef * 100.0)
                      << " %" << std::endl;
        }
    }
    else
    {
        std::cout << "No effective Tx/Rx recorded "
                  << "(link may have been DOWN, or sim ended too early)."
                  << std::endl;
    }

    std::cout << "=====================================================" << std::endl;

    if (out.is_open ())
    {
        out.close ();
        std::cout.rdbuf (coutbuf);
    }
    return 0;
}