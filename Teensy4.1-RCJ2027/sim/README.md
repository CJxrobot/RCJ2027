# PC 模擬器（act 程式）

在電腦上跑機器人的 Act 程式，**程式本身不用改**：
`lib/core_act/core_act.cpp` + `src/<程式>.cpp` 用 g++ 編譯，接上假的硬體（陀螺儀、Sense、相機、馬達都是假的，但送的 bytes 和真的一樣）。
結果是一個 HTML 檔，用瀏覽器打開就能播放。

---

## 一、Setup（每台電腦做一次）

### 1. 抓程式碼（sim 分支）
需要 git（Windows：`winget install -e --id Git.Git --source winget`，裝完開新的終端機）。
repo 是私人的話，clone 時會要求登入 GitHub。

```
git clone -b sim https://github.com/CJxrobot/RCJ2027.git
cd RCJ2027/Teensy4.1-RCJ2027/sim
```
已經 clone 過：
```
git checkout sim
git pull
```

### 2. 安裝 g++（C++ 編譯器）

**Windows**（PowerShell）
```
winget install -e --id BrechtSanders.WinLibs.POSIX.UCRT --source winget
```
- 裝完一定要**關掉 PowerShell 再開一個新的**，不然找不到 g++。
- 一定要加 `--source winget`，不加會出現 msstore 的錯誤。
- 檢查：`g++ --version` 有印出版本就好了。

**macOS**
```
xcode-select --install
```

**Linux**（Ubuntu / Debian）
```
sudo apt install g++
```

### 3. 試跑一次
**Windows**（PowerShell，在 `sim` 資料夾裡）
```
.\run.bat act_defense_smach2 ball_front
```
**macOS / Linux / Git Bash**
```
chmod +x run.sh      # 只要第一次
./run.sh act_defense_smach2 ball_front
```
看到 `-> out/act_defense_smach2_ball_front.html` 就成功了（Windows 會自動打開瀏覽器）。

> Windows 不要打 `.\run.sh`：PowerShell 會用 VS Code 打開它，不會執行。用 `.\run.bat`。

---

## 二、使用

### 跑情境
| 指令（Windows 用 `.\run.bat`，其他用 `./run.sh`） | 作用 |
|---|---|
| `run.bat` | act_defense_smach，全部防守情境 |
| `run.bat act_defense_smach2` | 指定程式，全部情境 |
| `run.bat act_defense_smach2 corner_unlock` | 只跑一個情境（Windows 會自動打開結果） |
| `run.bat act_attack2` | 攻擊程式：只跑 `atk_*` 情境 |
| `run.bat act_defense_smach2 xxx` | 打錯名字會列出全部情境和說明 |

程式名稱就是 `src/` 裡的檔名（不含 `.cpp`）：
`act_defense_smach`、`act_defense_smach2`、`act_attack`、`act_attack2`。
名字含 `defense` 的會開 mux 2 線感測器，其他只有 mux 1。

### 看結果
- 輸出在 `out/<程式>_<情境>.html`，用瀏覽器打開：Play / 拖時間軸。
- `...html#t=3.5` 會停在 3.5 秒。
- 下面的 Console 是程式從 Serial 印出來的東西（和實機一樣），加上時間。
- `[SIM]` 開頭的是模擬器自己的訊息：
  - `RULE: ...` = 違反規則（整台進禁區、碰牆、整台出白線）。**每次改完程式，全部情境都不能有 RULE。**
  - `GOAL (opponent goal)` = 進球，`OWN GOAL` = 烏龍球。
  - `KICK`、`robot touched the ball`、`robot pushed`。

### Live 模式（自己拖球）
```
.\run.bat live                      # Windows，防守程式，瀏覽器自動打開
.\run.bat act_attack2 live          # Windows，攻擊程式
./run.sh act_defense_smach2 live    # macOS / Linux，再打開 http://localhost:8765
```
- 滑鼠：點一下 = 把球放在那裡，拖 = 移動球，拖著放開 = 丟球，拖機器人 = 移動機器人。
- 按鈕：Pause、速度（0.25x-2x）、Take ball away。
- 結束：在終端機按 Ctrl+C。

### 更新
```
git pull
```
然後照常 `run.bat` / `run.sh`（每次都會重新編譯，不用另外做什麼）。

---

## 三、常見問題
| 狀況 | 解法 |
|---|---|
| `g++ not found` / `'g++' 不是內部或外部命令` | 裝完 g++ 沒開新的終端機；關掉重開。還是不行就重新開機 |
| winget 出現 msstore 錯誤 | 指令後面加 `--source winget` |
| PowerShell 打 `.\run.sh` 打開了 VS Code | Windows 用 `.\run.bat` |
| `../src/xxx.cpp: No such file or directory` | 程式名稱打錯，或檔案不在 `src/`（`src/test/` 裡的不能直接跑） |
| live 模式打不開 | 8765 port 被佔用：關掉上一個 live（Ctrl+C）再開 |
| 防毒擋住 `out\sim_*.exe` | 允許它（是剛剛自己編譯出來的） |
| `git pull` 失敗：有本機修改 | `git stash`，`git pull`，要拿回修改再 `git stash pop` |

---

## 四、檔案
| 檔案 | 作用 |
|---|---|
| `run.bat` / `run.sh` | 編譯 + 執行（Windows / 其他） |
| `stub/` | 假的 Arduino.h、Wire、OLED、DMAChannel（只有機器人程式用到的部分） |
| `fake_hw.cpp` | 時間、腳位、序列埠、相機 DMA |
| `world.cpp` | 場地、機器人、球、感測器、情境；所有數字在最上面 |
| `live.cpp` | live 模式的小網頁伺服器（只有這台電腦，port 8765） |
| `viewer.html` | 播放頁和 live 頁 |
| `out/` | 輸出（不會上傳到 git） |

## 五、怎麼運作
- 感測器送的 bytes 和真的一樣：陀螺儀 Serial2（100 Hz）、Sense Serial8 + PIN_0 / PIN_1（500 Hz）、超音波 AB 封包（每 30 ms 一顆，同樣的濾波）、相機 DMA（200 Hz）。
- 馬達：讀回 PWM / DIR 腳位換成機器人移動。100 % = 300 cm/s，馬達延遲 50 ms。向右橫移會往前飄（`DRIFT_R`，和實機一樣）。
- 對手推擠：情境裡的 `shove`（一段時間內外力推機器人）。

沒有模擬：輪子打滑、真實的 IR / 相機雜訊、球旋轉、其他機器人、馬達太小推不動（死區）。
用來檢查邏輯和情況，不是用來調參數。

要量了再改的猜測值（`world.cpp` 最上面）：IR 強度對距離、相機球的像素、輪子到中心 9 cm、`DRIFT_R`。

### 新增情境
在 `world.cpp` 的 `scenarios` 加一行：名字、說明、機器人起點 (x, y, 方向)、秒數、球的位置（時間, x, y, 看得到）。
座標：場地中心 (0, 0)，我方球門在 y < 0，對方球門在 y > 0，單位 cm。
名字用 `atk_` 開頭的只給攻擊程式跑，其他的給防守程式跑。
