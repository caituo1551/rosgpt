# 🤖 ROSGPT for ROS1 — 自然語言指令控制 AMR 的智能系統

本專案為基於 [Anis Koubaa 原始 ROSGPT 架構](https://www.preprints.org/manuscript/202304.0827/v2) 所改寫之 ROS1 版本，並擴充應用於實體 AMR（Autonomous Mobile Robot）環境。  
系統整合 ChatGPT（GPT-4）與 ROS，實現從自然語言（語音或文字）生成 JSON 指令，透過 ROS 控制機器人移動。

> 🔍 本專案以學術與非商業目的為主，保留原始概念與引用，並在 ROS1 環境下重構與擴展原有功能。

---

## 📖 引用與原始論文來源

本專案最初設計理念與程式架構參考自：

**Anis Koubaa**, *ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS*, Preprints.org, 2023  
[DOI: 10.20944/preprints202304.0827.v2](https://www.preprints.org/manuscript/202304.0827/v2)

```bibtex
@article{koubaa2023rosgpt,
  title={ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS},
  author={Koubaa, Anis},
  journal={Preprints.org},
  year={2023},
  volume={2023},
  pages={2023040827},
  doi={10.20944/preprints202304.0827.v2}
}


## 🗣️ 常用自然語言指令建議（語音／文字輸入）

以下為 ROSGPT 系統支援的自然語言指令範例，您可以使用中文語音或文字直接說出以下指令：

| 自然語言指令 | 對應動作 |
|------------------|--------------|
| 前進 1 公尺        | 相對前進 1 公尺 |
| 後退 0.5 公尺      | 相對後退 0.5 公尺 |
| 向右旋轉 90 度     | 原地順時針旋轉 90 度 |
| 向左轉 45 度       | 原地逆時針旋轉 45 度 |
| 前往位置 2.0 1.5    | 導航至指定地圖座標 (2.0, 1.5) |
| 前往工作站 A        | （若設有別名：ChatGPT 會自動補齊為 goto 指令） |

