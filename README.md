[README (2).md](https://github.com/user-attachments/files/34886770/README.2.md)
[README.md](https://github.com/user-attachments/files/34886751/README.md)
# Load cell reader (HX711 + STM32 Black Pill)

Measures force with a load cell, an HX711 amplifier and an STM32 Black Pill (STM32F411CE),
and shows it live in a desktop app on the PC. Built to measure the drag of a parachute
(force in newtons, then `Cd = 2F / (rho * v^2 * A)`).

- **Firmware** (`firmware/`): reads the HX711, filters the signal, zeroes it, converts to newtons
  and talks to the PC over USB (virtual COM port) using simple text commands.# Load cell reader (HX711 + STM32 Black Pill)

Measures force with a load cell, an HX711 amplifier and an STM32 Black Pill (STM32F411CE),
and shows it live in a desktop app on the PC. Built to measure the drag of a parachute
(force in newtons, then `Cd = 2F / (rho * v^2 * A)`).

- **Firmware**: reads the HX711, filters the signal, zeroes it, converts to newtons
  and talks to the PC over USB (virtual COM port) using simple text commands.
- **App**: Python window with live plot, zero button, load cell selector,
  calibration wizard, CSV recording and a drag coefficient calculator.

## Contents

1. [Hardware and wiring](#1-hardware-and-wiring)
2. [Firmware setup (STM32CubeIDE)](#2-firmware-setup-stm32cubeide)
3. [Quick test in a serial terminal](#3-quick-test-in-a-serial-terminal)
4. [The desktop app](#4-the-desktop-app)
5. [Using the app](#5-using-the-app)
6. [Calibration and where the factors live](#6-calibration-and-where-the-factors-live)
7. [Serial protocol reference](#7-serial-protocol-reference)
8. [Troubleshooting](#8-troubleshooting)
9. [Status and limitations](#9-status-and-limitations)
10. [Adding this to GitHub](#10-adding-this-to-github)

---

## 1. Hardware and wiring

### HX711 module to the Black Pill

| HX711 pin | Black Pill pin | Notes |
|---|---|---|
| VCC | 3.3V | 3.3 V keeps DT/SCK at the STM32's logic level |
| GND | GND | |
| DT (DOUT) | PB4 | input |
| SCK | PB5 | output |

### Load cell to the HX711 (screw terminals)

| Load cell wire (typical colors) | HX711 terminal |
|---|---|
| Red, excitation + | E+ |
| Black, excitation - | E- |
| White, signal - | A- |
| Green, signal + | A+ |
| Shield (bare or yellow, if present) | leave unconnected |

- **Colors vary between manufacturers.** Check the datasheet of your cell.
- Swapping A+ and A- only flips the sign of the reading. Mixing excitation and signal wires
  saturates the HX711 (see [Troubleshooting](#8-troubleshooting)).
- Leave B+ / B- unconnected: the firmware uses channel A, gain 128.
- If the reading is noisy, add 100 nF + 10 uF capacitors across the HX711's VCC/GND, keep the cell
  cable short and away from the USB cable, and connect the shield to the HX711 GND at that end only.
- Most HX711 boards sample at 10 samples/s (RATE pin low). The firmware is tuned for that.

---

## 2. Firmware setup (STM32CubeIDE)

### 2.1 Project configuration (`.ioc`)

If you start from scratch, create a project for **STM32F411CEUx** and set:

| Where in CubeMX | Setting |
|---|---|
| System Core, RCC | High Speed Clock (HSE) = Crystal/Ceramic Resonator |
| Connectivity, USB_OTG_FS | Mode = **Device_Only** |
| Middleware, USB_DEVICE | Class = **Communication Device Class (Virtual Port Com)** |
| Pin PB4 | GPIO_Input |
| Pin PB5 | GPIO_Output (push-pull, no pull, low speed) |
| Analog, ADC1 | IN0 enabled (left over from the first version; `main.c` still contains its init code) |
| Clock Configuration | HSE 25 MHz, PLL /M=25 x N=336 /P=4 -> SYSCLK 84 MHz, /Q=7 -> USB 48 MHz |

Then **Generate Code**.

If your `.ioc` has no ADC1, delete `hadc1`, `MX_ADC1_Init` (prototype, call and function) from
`main.c` instead of enabling it.

### 2.2 Our code

1. Replace the generated `Core/Src/main.c` with `main.c` from this repo. It keeps the
   `USER CODE BEGIN/END` markers, so regenerating from the `.ioc` later preserves our code.
2. Open `USB_DEVICE/App/usbd_cdc_if.c`, find `CDC_Receive_FS` and add **two lines** between
   `USER CODE BEGIN 6` and the existing `USBD_CDC_SetRxBuffer` call:

   ```c
   static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
   {
     /* USER CODE BEGIN 6 */
     extern void App_OnUsbRx(const uint8_t *buf, uint32_t len);   // <-- add
     App_OnUsbRx(Buf, *Len);                                      // <-- add

     USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
     USBD_CDC_ReceivePacket(&hUsbDeviceFS);
     return (USBD_OK);
     /* USER CODE END 6 */
   }
   ```
   Nothing may sit after `return (USBD_OK);` except the `USER CODE END 6` comment. If you have the
   older `usb_rx_cmd` lines from the first version, **delete them**.
3. Build (hammer icon).

### 2.3 Flashing

- **With an ST-Link:** connect SWDIO, SWCLK, GND and 3V3 to the Black Pill's 4-pin SWD header and press
  Run in CubeIDE.
- **Without an ST-Link (USB DFU):** hold the **BOOT0** button, tap **NRST** (reset), release BOOT0. Open
  STM32CubeProgrammer, choose **USB**, and program the `.hex`/`.bin` from the project's `Debug` folder.
  Press NRST afterwards to run it.

After flashing, plug the board's USB cable into the PC. Device Manager, "Ports (COM & LPT)" should show
**STMicroelectronics Virtual COM Port (COMx)**. Note the `COMx` number.

### 2.4 Behaviour at power-up

The board waits 1 s, selects load cell 1 and zeroes it (about 2 s). **Keep the load cell unloaded
while powering the board.** Until it receives its first command it prints
`# Load cell reader v2.0 ready. Type HELP and press Enter.` every 2 s.

### 2.5 Things you can change in `main.c`

| Constant | Meaning |
|---|---|
| `load_cells[]` | Names and scale factors (counts per gram). Names must not contain commas |
| `DEFAULT_CELL` | Cell selected at power-up (0 = first) |
| `MEDIAN_N`, `EMA_ALPHA` | Filter: median of N samples, then smoothing (smaller alpha = smoother and slower) |
| `ZERO_DEADBAND_N` | Readings below this (newtons) are reported as exactly 0 |
| `AUTO_ZERO_*` | Slow zero tracking: window (N) and speed |
| `TARE_SAMPLES` | Samples averaged when zeroing (about 2 s) |
| `CAL_*` | Calibration: samples discarded, samples averaged, minimum change that counts as "weight detected" |

---

## 3. Quick test in a serial terminal

Do this once after flashing, before using the app.

1. Open **Tera Term**: File, New connection, **Serial**, pick the STMicroelectronics COM port, OK.
2. Optional: Setup, Terminal, tick **Local echo** to see what you type.
3. Type a command and press **Enter**:

```
HELP          list the commands
PING          -> OK,PING,2.0
STATUS        -> STATUS,0,2000,1,9.0090,1,<raw>,<offset>,0
STREAM 1      -> F,<ms>,0.000   (a reading every 2 s; empty cell reads 0.000)
STREAM 0
TARE
```

If `PING` is answered, the firmware and the USB side work. **Close Tera Term before using the app:**
only one program can open the COM port.

### First bring-up checklist

1. Board shows up as a COM port.
2. `PING` answers `OK,PING,2.0`.
3. `STREAM 1` with an empty cell shows `F,...,0.000`. If you see `W,SATURATED` instead, fix the wiring first.
4. Put a known weight on: the force should be `mass_in_kg x 9.80665` newtons (for example 500 g gives 4.903 N)
   once the cell is calibrated.
5. Run the calibration wizard for every cell you plan to use.

---

## 4. The desktop app

### 4.1 One-time setup (Windows)

1. Install **Python 3.12** from python.org. On the first installer screen tick
   **"Add python.exe to PATH"**. Python 3.13 or newer may not have packages for PySide6 yet.
2. Put `app.py`, `core.py` and `requirements.txt` together in a folder **outside OneDrive**, for example
   `C:\instru_app` (`app.py` needs `core.py` next to it). The virtual environment has thousands of files
   that OneDrive would try to sync.
3. Open a terminal **in that folder**: open the folder in File Explorer, click the address bar, type
   `powershell` and press Enter.
4. Run:

```powershell
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

If `py -3.12` says "No suitable Python runtime found", Python 3.12 is not installed; check with
`py --list`. Calling `.venv\Scripts\python.exe` directly means you never have to "activate" the
environment (PowerShell often blocks the activation script).

Do **not** move or rename the folder after this: virtual environments are not relocatable. If you do, delete
`.venv` and repeat step 4.

### 4.2 Running the app

**From the terminal** (any time, same folder as step 3):

```powershell
.venv\Scripts\python.exe app.py
```

Plain `python app.py` will not work: the system Python does not have the packages, only the one in `.venv`.

**With a shortcut, no terminal:** create `run_app.bat` in that same folder:

```bat
@echo off
cd /d "%~dp0"
start "" ".venv\Scripts\pythonw.exe" app.py
```

Double-click it, or right-click it, Show more options, Send to, Desktop (create shortcut). For error
messages, use a second file `run_app_debug.bat`, which keeps the window open:

```bat
@echo off
cd /d "%~dp0"
.venv\Scripts\python.exe app.py
pause
```

### 4.3 Trying it without the board

In the **Port** list choose **Simulator (no hardware)** and press Connect. It behaves like the firmware
(same timing, same commands) and shows a panel to set the simulated weight, the cell's sensitivity and
a saturated input. Use it to learn the app or test changes.

---

## 5. Using the app

1. Pick the board's COM port and press **Connect**. The status line shows the firmware version and
   the active load cell. The last port used is remembered.
2. **Start readings** / **Stop readings** toggles the stream. **Every** sets the interval between readings
   (100 to 60000 ms, default 200). The big number is the latest force in newtons.
3. **Zero (tare)** zeroes the force. Keep the cell unloaded and still (about 2 s).
4. **Auto-zero** slowly follows drift while the reading stays within +-0.2 N of zero. It can absorb a very
   small real load that stays for a long time; turn it off if that matters.
5. **Load cell** selects which cell is plugged in. Selecting one also zeroes it, so the app asks for confirmation.
6. **Calibrate...** starts the wizard (section 6).
7. **Plot:** *Follow* scrolls with the data over the chosen window; untick it to pan and zoom with the mouse.
   *Clear plot* discards the displayed data.
8. **Selected region:** drag the shaded band over a steady segment. The app shows samples, mean, standard
   deviation, minimum and maximum. *Selection = last 10 s* snaps it to the latest data.
9. **Drag coefficient:** type air speed, air density (1.225 kg/m3 at sea level, 15 C) and reference area.
   `Cd = 2 x mean force / (density x speed^2 x area)` is computed for the selected region.
10. **Marker** adds a labelled vertical line to the plot and a row to the CSV (for example "release").
11. **Record CSV** asks for a file and writes every reading while the stream is on.

### CSV format

```
# app: loadcell_app
# started: 2026-11-15T18:30:00
# firmware: 2.0
# cell: 2 (Load cell 2)
# scale_counts_per_g: 44.9390
# interval_ms: 200
# unit: newtons
pc_time,device_ms,elapsed_s,force_N,cell,scale_counts_per_g,marker
2026-11-15T18:30:01.250,12400,0.000,0.000,2,44.9390,
```

Read it with pandas: `pd.read_csv(path, comment="#")`. `device_ms` is the board's own clock (use this for
timing); `pc_time` is the PC's clock when the line arrived.

### Where the app keeps its settings

`%USERPROFILE%\.loadcell_app.json` stores the last port, the interval and the saved calibrations. It is in your
user folder, not the app folder, so replacing `app.py` or `core.py` keeps it. Delete the file to reset.

---

## 6. Calibration and where the factors live

The **scale factor** is in **counts per gram**: `grams = (counts - zero) / factor`, and
`force_N = grams x 9.80665 / 1000`.

### Calibrating a cell

1. Select the cell, press **Calibrate...**.
2. **Step 1:** remove all weight, keep the cell still, press **Zero**.
3. **Step 2:** put the known weight on the cell, wait until it stops moving, type its mass **in grams**, press
   **Measure**. It discards 15 samples and averages 100 (about 12 s); keep everything still.
4. The result shows the factor and the noise (peak-to-peak, in grams).

Tips:
- Use a mass of roughly 20 to 100 % of the cell's capacity; very light weights give a noisy factor.
- The factor is only as accurate as the mass you type in. Use a weighed mass.
- A negative factor means the signal polarity is inverted. It works as is; swapping A+/A- also fixes it.
- If the HX711 input saturates or no change is detected, the wizard stops and tells you why.

### Two places hold factors

- The **app** remembers every calibration automatically and re-sends it to the board each time you connect
  (the board forgets runtime changes when it resets).
- The **firmware table** `load_cells[]` in `main.c` is what the board starts with. To make a calibration
  permanent there, press **Copy firmware line** in the wizard and replace the matching line, for example
  `{ "Load cell 2", 44.9390f },`, then re-flash.

If you re-flash a different factor for a cell, the app notices that the firmware changed and drops its own
saved value for that cell, so the newer one wins.

---

## 7. Serial protocol reference

For anyone writing another client. Text over the virtual COM port (baud rate is ignored). One command per
line, ended with Enter (`\r`, `\n` or `\r\n`), case-insensitive, space or comma between arguments, up to 63
characters per line.

### Commands

| Command | Does | Answer |
|---|---|---|
| `PING` | check the connection | `OK,PING,<firmware>` |
| `HELP` | list commands | `# ...` lines, then `OK,HELP` |
| `STREAM 1` / `STREAM 0` | start / stop readings | `OK,STREAM,<0/1>` |
| `INTERVAL <ms>` | time between readings, 100 to 60000 | `OK,INTERVAL,<ms>` |
| `TARE` | zero the force (about 2 s) | `OK,TARE` or `ERR,TARE,<reason>` |
| `AUTOZERO 1` / `AUTOZERO 0` | drift tracking on / off | `OK,AUTOZERO,<0/1>` |
| `CELLS` | list the load cells | `CELL,<n>,<name>,<counts_per_g>,<active>` per cell, then `OK,CELLS,<count>` |
| `CELL <n>` | select cell n and zero it | `OK,CELL,<n>,<name>,<counts_per_g>` |
| `SCALE <n> <counts_per_g>` | set a cell's factor until reset | `OK,SCALE,<n>,<value>` |
| `CAL ZERO` | calibration step 1 (empty cell) | `OK,CAL,ZERO` |
| `CAL <grams>` | calibration step 2 (about 12 s) | `CAL,PROGRESS,<%>` lines, then `OK,CAL,RESULT,<n>,<counts_per_g>,<noise_counts_pp>` |
| `CAL CANCEL` | abort | `OK,CAL,CANCEL` |
| `STATUS` | one-line status | `STATUS,<stream>,<ms>,<cell>,<counts_per_g>,<autozero>,<raw>,<offset>,<cal_hold>` |

### Messages from the board

| Line | Meaning |
|---|---|
| `F,<ms>,<newtons>` | a reading; `<ms>` is the board's tick |
| `W,SATURATED` | the HX711 input is at its limit; the value is meaningless |
| `W,UNCALIBRATED,<n>` | the selected cell has no factor yet |
| `W,HX711_TIMEOUT` | the HX711 stopped answering (check wiring) |
| `ERR,<command>,<reason>` | reasons: `ARG`, `RANGE`, `BAD_ARG`, `UNKNOWN`, `SATURATED`, `NO_HX711`, `NO_WEIGHT`, `ZERO_FIRST`, `TOO_LONG` |
| `# text` | human-readable text, ignore it in software |

The firmware does one thing at a time: `TARE`, `CELL` and `CAL` block it for 2 to 12 s, during which
readings pause and other commands wait. A client should send the next command only after the answer to the
previous one.

---

## 8. Troubleshooting

| Problem | Likely cause and fix |
|---|---|
| `No suitable Python runtime found` | Python 3.12 is not installed. Install it, tick "Add python.exe to PATH", open a new terminal. `py --list` shows what is installed |
| Activation script blocked in PowerShell | Don't activate; call `.venv\Scripts\python.exe` directly. Or run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once and use `.venv\Scripts\Activate.ps1` |
| `ModuleNotFoundError` (PySide6, serial...) | You ran the system Python. Use `.venv\Scripts\python.exe app.py` or the `.bat` file |
| "Cannot open the port" | Another program has it (Tera Term, another copy of the app). Close it, or replug the USB cable |
| "No answer from the board" | Wrong COM port, the old firmware (v1 does not speak this protocol), or the `usbd_cdc_if.c` edit is missing or after the `return`. Test with `PING` in Tera Term |
| Reading is `SATURATED` | The HX711 input is at its limit, with or without weight. Check that E+/E- and A+/A- are not mixed up, the wires are firm in the terminals, and the cell is a complete 4-wire bridge. With the HX711 powered, E+ to E- should show about the supply voltage, and A+ to A- with no load only a few millivolts |
| Force is about 0 or stuck with a weight on | Same cause (saturated input), or the cell is not connected to the HX711 |
| Force is wrong by a constant factor | Wrong scale factor. Recalibrate; the factor is counts per gram |
| Force reads negative | Signal polarity; calibrate (it handles the sign) or swap A+ and A- |
| Reading drifts over minutes | Let the HX711 warm up 5 to 15 minutes, add the supply capacitors, check the mounting, and keep auto-zero on |
| Plot jumps back in time | The board was reset; the app keeps the time axis continuous |
| Nothing in Tera Term except `# ... ready` lines | That is normal until you send a command: type `HELP` and press Enter |

---

## 9. Status and limitations

- The firmware and the app were developed and checked against simulators (a PC build of the firmware with a
  simulated HX711 and USB port, and the app's built-in Simulator). **Validate on the real hardware** with the
  bring-up checklist in section 3 and note the results here.
- Readings come at the HX711's rate, 10 per second by default, so the opening shock of a parachute is
  not resolved. The HX711 can run at 80 samples/s (RATE pin high on most boards), and the filter and logging
  would need to be adapted for that.
- Not implemented yet: external flash logging for drop tests, BME280 (air density), anemometer or pitot
  tube (air speed), LCD. Air speed and density are typed into the app by hand for now.
- Instructions here are written for Windows. The Python app should also run on Linux and macOS, but this
  has not been tested.

---

## 10. Adding this to GitHub

You need **Git** (git-scm.com) or **GitHub Desktop**. With Git, in a terminal:

```powershell
git clone https://github.com/<your-username>/test1.git
cd test1
# copy your files into the folder, then:
git add .
git commit -m "Load cell reader: firmware v2.0 and desktop app"
git push
```

- `.gitignore` keeps `.venv`, `__pycache__` and STM32CubeIDE build folders out of the repository. If a
  `.venv` was added by mistake before the `.gitignore`, remove it from Git with
  `git rm -r --cached <folder>/.venv`, then commit.
- The saved calibrations live in your user folder (`.loadcell_app.json`), so they are never committed.
- Add your own measurement CSVs to the repository only if you want to share them.

- **App** (`app/`): Python window with live plot, zero button, load cell selector,
  calibration wizard, CSV recording and a drag coefficient calculator.

## Contents

1. [Repository layout](#1-repository-layout)
2. [Hardware and wiring](#2-hardware-and-wiring)
3. [Firmware setup (STM32CubeIDE)](#3-firmware-setup-stm32cubeide)
4. [Quick test in a serial terminal](#4-quick-test-in-a-serial-terminal)
5. [The desktop app](#5-the-desktop-app)
6. [Using the app](#6-using-the-app)
7. [Calibration and where the factors live](#7-calibration-and-where-the-factors-live)
8. [Serial protocol reference](#8-serial-protocol-reference)
9. [Troubleshooting](#9-troubleshooting)
10. [Status and limitations](#10-status-and-limitations)
11. [Adding this to GitHub](#11-adding-this-to-github)

---

## 1. Repository layout

```
test1/
├── README.md
├── .gitignore
├── firmware/                 STM32CubeIDE project
│   ├── Core/Src/main.c       <- the file with our code
│   └── USB_DEVICE/App/usbd_cdc_if.c   <- two lines added (see section 3)
└── app/
    ├── app.py                the window (PySide6 + pyqtgraph)
    ├── core.py               protocol, serial/simulated devices, calibration, CSV, statistics
    ├── requirements.txt      Python packages
    ├── run_app.bat           double-click launcher (no console window)
    └── run_app_debug.bat     same, but keeps the console open to read errors
```

`app.py` and `core.py` must stay in the same folder.

---

## 2. Hardware and wiring

### HX711 module to the Black Pill

| HX711 pin | Black Pill pin | Notes |
|---|---|---|
| VCC | 3.3V | 3.3 V keeps DT/SCK at the STM32's logic level |
| GND | GND | |
| DT (DOUT) | PB4 | input |
| SCK | PB5 | output |

### Load cell to the HX711 (screw terminals)

| Load cell wire (typical colors) | HX711 terminal |
|---|---|
| Red, excitation + | E+ |
| Black, excitation - | E- |
| White, signal - | A- |
| Green, signal + | A+ |
| Shield (bare or yellow, if present) | leave unconnected |

- **Colors vary between manufacturers.** Check the datasheet of your cell.
- Swapping A+ and A- only flips the sign of the reading. Mixing excitation and signal wires
  saturates the HX711 (see [Troubleshooting](#9-troubleshooting)).
- Leave B+ / B- unconnected: the firmware uses channel A, gain 128.
- If the reading is noisy, add 100 nF + 10 uF capacitors across the HX711's VCC/GND, keep the cell
  cable short and away from the USB cable, and connect the shield to the HX711 GND at that end only.
- Most HX711 boards sample at 10 samples/s (RATE pin low). The firmware is tuned for that.

---

## 3. Firmware setup (STM32CubeIDE)

### 3.1 Project configuration (`.ioc`)

If you start from scratch, create a project for **STM32F411CEUx** and set:

| Where in CubeMX | Setting |
|---|---|
| System Core, RCC | High Speed Clock (HSE) = Crystal/Ceramic Resonator |
| Connectivity, USB_OTG_FS | Mode = **Device_Only** |
| Middleware, USB_DEVICE | Class = **Communication Device Class (Virtual Port Com)** |
| Pin PB4 | GPIO_Input |
| Pin PB5 | GPIO_Output (push-pull, no pull, low speed) |
| Analog, ADC1 | IN0 enabled (left over from the first version; `main.c` still contains its init code) |
| Clock Configuration | HSE 25 MHz, PLL /M=25 x N=336 /P=4 -> SYSCLK 84 MHz, /Q=7 -> USB 48 MHz |

Then **Generate Code**.

If your `.ioc` has no ADC1, delete `hadc1`, `MX_ADC1_Init` (prototype, call and function) from
`main.c` instead of enabling it.

### 3.2 Our code

1. Replace the generated `Core/Src/main.c` with `main.c` from this repo. It keeps the
   `USER CODE BEGIN/END` markers, so regenerating from the `.ioc` later preserves our code.
2. Open `USB_DEVICE/App/usbd_cdc_if.c`, find `CDC_Receive_FS` and add **two lines** between
   `USER CODE BEGIN 6` and the existing `USBD_CDC_SetRxBuffer` call:

   ```c
   static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
   {
     /* USER CODE BEGIN 6 */
     extern void App_OnUsbRx(const uint8_t *buf, uint32_t len);   // <-- add
     App_OnUsbRx(Buf, *Len);                                      // <-- add

     USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
     USBD_CDC_ReceivePacket(&hUsbDeviceFS);
     return (USBD_OK);
     /* USER CODE END 6 */
   }
   ```
   Nothing may sit after `return (USBD_OK);` except the `USER CODE END 6` comment. If you have the
   older `usb_rx_cmd` lines from the first version, **delete them**.
3. Build (hammer icon).

### 3.3 Flashing

- **With an ST-Link:** connect SWDIO, SWCLK, GND and 3V3 to the Black Pill's 4-pin SWD header and press
  Run in CubeIDE.
- **Without an ST-Link (USB DFU):** hold the **BOOT0** button, tap **NRST** (reset), release BOOT0. Open
  STM32CubeProgrammer, choose **USB**, and program the `.hex`/`.bin` from the project's `Debug` folder.
  Press NRST afterwards to run it.

After flashing, plug the board's USB cable into the PC. Device Manager, "Ports (COM & LPT)" should show
**STMicroelectronics Virtual COM Port (COMx)**. Note the `COMx` number.

### 3.4 Behaviour at power-up

The board waits 1 s, selects load cell 1 and zeroes it (about 2 s). **Keep the load cell unloaded
while powering the board.** Until it receives its first command it prints
`# Load cell reader v2.0 ready. Type HELP and press Enter.` every 2 s.

### 3.5 Things you can change in `main.c`

| Constant | Meaning |
|---|---|
| `load_cells[]` | Names and scale factors (counts per gram). Names must not contain commas |
| `DEFAULT_CELL` | Cell selected at power-up (0 = first) |
| `MEDIAN_N`, `EMA_ALPHA` | Filter: median of N samples, then smoothing (smaller alpha = smoother and slower) |
| `ZERO_DEADBAND_N` | Readings below this (newtons) are reported as exactly 0 |
| `AUTO_ZERO_*` | Slow zero tracking: window (N) and speed |
| `TARE_SAMPLES` | Samples averaged when zeroing (about 2 s) |
| `CAL_*` | Calibration: samples discarded, samples averaged, minimum change that counts as "weight detected" |

---

## 4. Quick test in a serial terminal

Do this once after flashing, before using the app.

1. Open **Tera Term**: File, New connection, **Serial**, pick the STMicroelectronics COM port, OK.
2. Optional: Setup, Terminal, tick **Local echo** to see what you type.
3. Type a command and press **Enter**:

```
HELP          list the commands
PING          -> OK,PING,2.0
STATUS        -> STATUS,0,2000,1,9.0090,1,<raw>,<offset>,0
STREAM 1      -> F,<ms>,0.000   (a reading every 2 s; empty cell reads 0.000)
STREAM 0
TARE
```

If `PING` is answered, the firmware and the USB side work. **Close Tera Term before using the app:**
only one program can open the COM port.

### First bring-up checklist

1. Board shows up as a COM port.
2. `PING` answers `OK,PING,2.0`.
3. `STREAM 1` with an empty cell shows `F,...,0.000`. If you see `W,SATURATED` instead, fix the wiring first.
4. Put a known weight on: the force should be `mass_in_kg x 9.80665` newtons (for example 500 g gives 4.903 N)
   once the cell is calibrated.
5. Run the calibration wizard for every cell you plan to use.

---

## 5. The desktop app

### 5.1 One-time setup (Windows)

1. Install **Python 3.12** from python.org. On the first installer screen tick
   **"Add python.exe to PATH"**. Python 3.13 or newer may not have packages for PySide6 yet.
2. Put the `app` folder somewhere **outside OneDrive**, for example `C:\instru_app`. The virtual
   environment has thousands of files that OneDrive would try to sync.
3. Open a terminal **in that folder**: open the folder in File Explorer, click the address bar, type
   `powershell` and press Enter.
4. Run:

```powershell
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

If `py -3.12` says "No suitable Python runtime found", Python 3.12 is not installed; check with
`py --list`. Calling `.venv\Scripts\python.exe` directly means you never have to "activate" the
environment (PowerShell often blocks the activation script).

Do **not** move or rename the folder after this: virtual environments are not relocatable. If you do, delete
`.venv` and repeat step 4.

### 5.2 Running the app

**From the terminal** (any time, same folder as step 3):

```powershell
.venv\Scripts\python.exe app.py
```

Plain `python app.py` will not work: the system Python does not have the packages, only the one in `.venv`.

**With a shortcut, no terminal:** create `run_app.bat` in the app folder:

```bat
@echo off
cd /d "%~dp0"
start "" ".venv\Scripts\pythonw.exe" app.py
```

Double-click it, or right-click it, Show more options, Send to, Desktop (create shortcut). For error
messages, use a second file `run_app_debug.bat`, which keeps the window open:

```bat
@echo off
cd /d "%~dp0"
.venv\Scripts\python.exe app.py
pause
```

### 5.3 Trying it without the board

In the **Port** list choose **Simulator (no hardware)** and press Connect. It behaves like the firmware
(same timing, same commands) and shows a panel to set the simulated weight, the cell's sensitivity and
a saturated input. Use it to learn the app or test changes.

---

## 6. Using the app

1. Pick the board's COM port and press **Connect**. The status line shows the firmware version and
   the active load cell. The last port used is remembered.
2. **Start readings** / **Stop readings** toggles the stream. **Every** sets the interval between readings
   (100 to 60000 ms, default 200). The big number is the latest force in newtons.
3. **Zero (tare)** zeroes the force. Keep the cell unloaded and still (about 2 s).
4. **Auto-zero** slowly follows drift while the reading stays within +-0.2 N of zero. It can absorb a very
   small real load that stays for a long time; turn it off if that matters.
5. **Load cell** selects which cell is plugged in. Selecting one also zeroes it, so the app asks for confirmation.
6. **Calibrate...** starts the wizard (section 7).
7. **Plot:** *Follow* scrolls with the data over the chosen window; untick it to pan and zoom with the mouse.
   *Clear plot* discards the displayed data.
8. **Selected region:** drag the shaded band over a steady segment. The app shows samples, mean, standard
   deviation, minimum and maximum. *Selection = last 10 s* snaps it to the latest data.
9. **Drag coefficient:** type air speed, air density (1.225 kg/m3 at sea level, 15 C) and reference area.
   `Cd = 2 x mean force / (density x speed^2 x area)` is computed for the selected region.
10. **Marker** adds a labelled vertical line to the plot and a row to the CSV (for example "release").
11. **Record CSV** asks for a file and writes every reading while the stream is on.

### CSV format

```
# app: loadcell_app
# started: 2026-11-15T18:30:00
# firmware: 2.0
# cell: 2 (Load cell 2)
# scale_counts_per_g: 44.9390
# interval_ms: 200
# unit: newtons
pc_time,device_ms,elapsed_s,force_N,cell,scale_counts_per_g,marker
2026-11-15T18:30:01.250,12400,0.000,0.000,2,44.9390,
```

Read it with pandas: `pd.read_csv(path, comment="#")`. `device_ms` is the board's own clock (use this for
timing); `pc_time` is the PC's clock when the line arrived.

### Where the app keeps its settings

`%USERPROFILE%\.loadcell_app.json` stores the last port, the interval and the saved calibrations. It is in your
user folder, not the app folder, so replacing `app.py` or `core.py` keeps it. Delete the file to reset.

---

## 7. Calibration and where the factors live

The **scale factor** is in **counts per gram**: `grams = (counts - zero) / factor`, and
`force_N = grams x 9.80665 / 1000`.

### Calibrating a cell

1. Select the cell, press **Calibrate...**.
2. **Step 1:** remove all weight, keep the cell still, press **Zero**.
3. **Step 2:** put the known weight on the cell, wait until it stops moving, type its mass **in grams**, press
   **Measure**. It discards 15 samples and averages 100 (about 12 s); keep everything still.
4. The result shows the factor and the noise (peak-to-peak, in grams).

Tips:
- Use a mass of roughly 20 to 100 % of the cell's capacity; very light weights give a noisy factor.
- The factor is only as accurate as the mass you type in. Use a weighed mass.
- A negative factor means the signal polarity is inverted. It works as is; swapping A+/A- also fixes it.
- If the HX711 input saturates or no change is detected, the wizard stops and tells you why.

### Two places hold factors

- The **app** remembers every calibration automatically and re-sends it to the board each time you connect
  (the board forgets runtime changes when it resets).
- The **firmware table** `load_cells[]` in `main.c` is what the board starts with. To make a calibration
  permanent there, press **Copy firmware line** in the wizard and replace the matching line, for example
  `{ "Load cell 2", 44.9390f },`, then re-flash.

If you re-flash a different factor for a cell, the app notices that the firmware changed and drops its own
saved value for that cell, so the newer one wins.

---

## 8. Serial protocol reference

For anyone writing another client. Text over the virtual COM port (baud rate is ignored). One command per
line, ended with Enter (`\r`, `\n` or `\r\n`), case-insensitive, space or comma between arguments, up to 63
characters per line.

### Commands

| Command | Does | Answer |
|---|---|---|
| `PING` | check the connection | `OK,PING,<firmware>` |
| `HELP` | list commands | `# ...` lines, then `OK,HELP` |
| `STREAM 1` / `STREAM 0` | start / stop readings | `OK,STREAM,<0/1>` |
| `INTERVAL <ms>` | time between readings, 100 to 60000 | `OK,INTERVAL,<ms>` |
| `TARE` | zero the force (about 2 s) | `OK,TARE` or `ERR,TARE,<reason>` |
| `AUTOZERO 1` / `AUTOZERO 0` | drift tracking on / off | `OK,AUTOZERO,<0/1>` |
| `CELLS` | list the load cells | `CELL,<n>,<name>,<counts_per_g>,<active>` per cell, then `OK,CELLS,<count>` |
| `CELL <n>` | select cell n and zero it | `OK,CELL,<n>,<name>,<counts_per_g>` |
| `SCALE <n> <counts_per_g>` | set a cell's factor until reset | `OK,SCALE,<n>,<value>` |
| `CAL ZERO` | calibration step 1 (empty cell) | `OK,CAL,ZERO` |
| `CAL <grams>` | calibration step 2 (about 12 s) | `CAL,PROGRESS,<%>` lines, then `OK,CAL,RESULT,<n>,<counts_per_g>,<noise_counts_pp>` |
| `CAL CANCEL` | abort | `OK,CAL,CANCEL` |
| `STATUS` | one-line status | `STATUS,<stream>,<ms>,<cell>,<counts_per_g>,<autozero>,<raw>,<offset>,<cal_hold>` |

### Messages from the board

| Line | Meaning |
|---|---|
| `F,<ms>,<newtons>` | a reading; `<ms>` is the board's tick |
| `W,SATURATED` | the HX711 input is at its limit; the value is meaningless |
| `W,UNCALIBRATED,<n>` | the selected cell has no factor yet |
| `W,HX711_TIMEOUT` | the HX711 stopped answering (check wiring) |
| `ERR,<command>,<reason>` | reasons: `ARG`, `RANGE`, `BAD_ARG`, `UNKNOWN`, `SATURATED`, `NO_HX711`, `NO_WEIGHT`, `ZERO_FIRST`, `TOO_LONG` |
| `# text` | human-readable text, ignore it in software |

The firmware does one thing at a time: `TARE`, `CELL` and `CAL` block it for 2 to 12 s, during which
readings pause and other commands wait. A client should send the next command only after the answer to the
previous one.

---

## 9. Troubleshooting

| Problem | Likely cause and fix |
|---|---|
| `No suitable Python runtime found` | Python 3.12 is not installed. Install it, tick "Add python.exe to PATH", open a new terminal. `py --list` shows what is installed |
| Activation script blocked in PowerShell | Don't activate; call `.venv\Scripts\python.exe` directly. Or run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once and use `.venv\Scripts\Activate.ps1` |
| `ModuleNotFoundError` (PySide6, serial...) | You ran the system Python. Use `.venv\Scripts\python.exe app.py` or the `.bat` file |
| "Cannot open the port" | Another program has it (Tera Term, another copy of the app). Close it, or replug the USB cable |
| "No answer from the board" | Wrong COM port, the old firmware (v1 does not speak this protocol), or the `usbd_cdc_if.c` edit is missing or after the `return`. Test with `PING` in Tera Term |
| Reading is `SATURATED` | The HX711 input is at its limit, with or without weight. Check that E+/E- and A+/A- are not mixed up, the wires are firm in the terminals, and the cell is a complete 4-wire bridge. With the HX711 powered, E+ to E- should show about the supply voltage, and A+ to A- with no load only a few millivolts |
| Force is about 0 or stuck with a weight on | Same cause (saturated input), or the cell is not connected to the HX711 |
| Force is wrong by a constant factor | Wrong scale factor. Recalibrate; the factor is counts per gram |
| Force reads negative | Signal polarity; calibrate (it handles the sign) or swap A+ and A- |
| Reading drifts over minutes | Let the HX711 warm up 5 to 15 minutes, add the supply capacitors, check the mounting, and keep auto-zero on |
| Plot jumps back in time | The board was reset; the app keeps the time axis continuous |
| Nothing in Tera Term except `# ... ready` lines | That is normal until you send a command: type `HELP` and press Enter |

---

## 10. Status and limitations

- The firmware and the app were developed and checked against simulators (a PC build of the firmware with a
  simulated HX711 and USB port, and the app's built-in Simulator). **Validate on the real hardware** with the
  bring-up checklist in section 4 and note the results here.
- Readings come at the HX711's rate, 10 per second by default, so the opening shock of a parachute is
  not resolved. The HX711 can run at 80 samples/s (RATE pin high on most boards), and the filter and logging
  would need to be adapted for that.
- Not implemented yet: external flash logging for drop tests, BME280 (air density), anemometer or pitot
  tube (air speed), LCD. Air speed and density are typed into the app by hand for now.
- Instructions here are written for Windows. The Python app should also run on Linux and macOS, but this
  has not been tested.

---

## 11. Adding this to GitHub

You need **Git** (git-scm.com) or **GitHub Desktop**. With Git, in a terminal:

```powershell
git clone https://github.com/<your-username>/test1.git
cd test1
# copy the files in using the layout from section 1, then:
git add .
git commit -m "Load cell reader: firmware v2.0 and desktop app"
git push
```

- `.gitignore` keeps `.venv`, `__pycache__` and STM32CubeIDE build folders out of the repository. If a
  `.venv` was added by mistake before the `.gitignore`, remove it from Git with
  `git rm -r --cached app/.venv`, then commit.
- The saved calibrations live in your user folder (`.loadcell_app.json`), so they are never committed.
- Add your own measurement CSVs to the repository only if you want to share them.
