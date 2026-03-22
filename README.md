# CSProject

## Overview
This project focuses on simulating unmanned aerial systems (UAS) communication and routing protocols using NS-3 network simulator.

## Topics
- Flying Vehicle Communication
- UAS-based VANET (Vehicular Ad Hoc Network)
- Inter-Satellite Links (ISL) Routing
- LEO (Low Earth Orbit) Satellite Constellations

## Resources & References

### Flying Vehicle Communication
1. [How to Implement Flying Vehicle Communication in NS-3](https://ns3simulation.com/how-to-implement-flying-vehicle-communication-in-ns3/)
2. [How to Simulate UAS-based VANET Projects Using NS-3](https://phdprime.com/how-to-simulate-uas-based-vanet-projects-using-ns3/)

### ISL Routing

#### Static Routing
For predictable LEO orbits, routing tables can be pre-calculated in advance for specific time slices (snapshots) when the network topology is considered stable. The `sat-constellation-example.cc` in the `sns3-satellite` module uses static routing as a basic implementation.

**Related Repository:**
- [SNS3 Satellite Module](https://github.com/sns3/sns3-satellite?tab=readme-ov-file)

## Updates (from latest meeting)
- [x] 根據 SNR 更新 data rate: 根據實測或模擬 SNR 結果動態調整鏈路 data rate（例如 C/N 對應 Modulation and Coding Scheme）。
- [x] UAV to LEO 部分使用衛星通訊頻段計算 Data rate：將 UAV→衛星鏈路改為衛星通訊常見頻段（如 Ka、Ku、S 波段）並依頻段特性估算理論/實際 data rate。
- [x] UAV 改成使用 beamforming 射向衛星：引入波束賦形（Beamforming/Hybrid Beamforming）方法提高 UAV 與衛星連線效率（參考世杰學長提供：[Introduction to Hybrid Beamforming](https://ww2.mathworks.cn/help/phased/ug/introduction-to-hybrid-beamforming.html)）。

## TODO
- [ ] Implement flying vehicle communication simulation
- [ ] Setup UAS-based VANET simulation
- [ ] Configure ISL routing for satellite constellations
- [ ] Review and integrate sns3-satellite examples
