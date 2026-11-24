# PC 模擬器（act 程式）

Runs the robot program on a PC **without changing it**:
`lib/core_act/core_act.cpp` + `src/<program>.cpp` are compiled with g++ against fake hardware.

```
./run.sh                                # act_defense_smach, all scenarios
./run.sh act_defense_smach ball_front   # one scenario
```
Live mode (drag the ball yourself):
```
.\run.bat live                 # Windows, defense program, browser opens by itself
.\run.bat act_attack live      # attack program
./run.sh act_defense_smach live # Linux, then open http://localhost:8765
```
Mouse: click = put the ball there, drag = move it, let go while moving = throw it, drag the robot = move the robot.
Buttons: Pause, speed (0.25x-2x), Take ball away. Stop with Ctrl+C in the terminal.

Output: `out/<program>_<scenario>.html`. Open it in a browser (Play / slider; `...html#t=3.5` opens paused at 3.5 s).
Windows (PowerShell): `.\run.bat` or `.\run.bat act_defense_smach ball_front`.
Needs g++ once: `winget install -e --id BrechtSanders.WinLibs.POSIX.UCRT --source winget`, then open a new terminal.

| File | What it does |
|---|---|
| `stub/` | Fake Arduino.h, Wire, OLED, DMAChannel (only what the robot code uses) |
| `fake_hw.cpp` | Time, pins, serial ports, camera DMA ring |
| `world.cpp` | Field, robot, ball, sensors, scenarios, all numbers at the top |
| `live.cpp` | Tiny web server for live mode (this computer only, port 8765) |
| `viewer.html` | Playback page and live page |

How it works:
- Sensors: the same bytes as the real ones. Gyro on Serial2 (100 Hz), Sense on Serial8 plus PIN_0/PIN_1 (500 Hz), camera into the DMA ring (200 Hz).
- Motors: the PWM / DIR pins are read back and turned into robot motion. 100 % = 300 cm/s, with a 50 ms motor lag.
- Console: everything the program prints on Serial, with the time. `[SIM]` lines come from the simulator (kick, goal, touch).

Not modelled: wheel slip, real IR / camera noise, ball spin, other robots. Use it to check logic and situations, not tuning.

Guesses to check (top of `world.cpp`): mux 2 sensor radii, IR strength vs distance, camera ball pixels, wheel distance 9 cm.
Add a scenario: one line in `scenarios` (robot start + ball positions over time).
