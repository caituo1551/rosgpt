# 🧠 ROSGPT AMR 指令格式說明文件

本文件定義了送入 `/voice_cmd` 的 JSON 指令格式，用於實體 AMR 的動作控制。  
目前由 `rosgptparser_amr.py` 負責解析與執行，透過 `move_base` 控制機器人移動。

---

## ✅ 通用格式

```json
[
  {
    "action": "指令名稱",
    "params": {
      // 各指令對應的參數
    }
  }
]

