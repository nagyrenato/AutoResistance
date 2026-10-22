# DomyosBridge (Linux BlueZ Edition)

Turns your Linux laptop into a **Virtual FTMS (`0x1826`) + Cycling Power (`0x1818`) Smart Trainer** intermediary between the **Decathlon Domyos Bike 900 / e-Bike 900** and **MyWhoosh** running on your **Google Pixel 9 Pro XL**.

## Key Features
- **Zero External Dependencies**: Uses your laptop's built-in **Python 3.12**, **BlueZ 5.72 D-Bus API**, and **GLib**.
- **Mouse Wheel Virtual Shifting (32 Gears)**: Scroll the **Mouse Wheel UP / DOWN** (or Right/Left click) inside the terminal window to shift **Virtual Gears 1–32** (Base Level 18) on the fly—shifting your resistance seamlessly while still tracking MyWhoosh hill gradients (`0x11`) and ERG workouts (`0x05`).
- **QZ-Calibrated Domyos Bike 900 Power Curve**: Uses the non-linear 32-step magnetic brake watt table for the Domyos Training Bike 900 whenever the console does not emit native watts.

## Quick Start

1. **Test Pairing with Your Phone First (No Bike Needed)**
   ```bash
   ./run_bridge.sh --simulate
   ```
   - Open **MyWhoosh** on your **Pixel 9 Pro XL** → **Connect Devices**.
   - Select **`DomyosBridge`** for **Power**, **Cadence**, and **Controllable**.

2. **Live Ride with Your Domyos Bike 900**
   - Turn on your **Domyos Bike 900** console (make sure your phone is **not** directly connected to the bike).
   - Start the bridge on your laptop:
     ```bash
     ./run_bridge.sh
     ```
   - **Optional CLI Parameters** (select or force hardware protocol & resistance scale):
     ```bash
     ./run_bridge.sh --protocol auto          # Default: auto-detect FTMS vs Domyos UART
     ./run_bridge.sh --protocol both          # Drive BOTH FTMS 0x1826 and Domyos UART simultaneously
     ./run_bridge.sh --protocol ftms          # Force standard FTMS (0x1826) only
     ./run_bridge.sh --protocol uart --max-level 15   # Force Domyos UART with 15-level console scale
     ```

## Mouse & Keyboard Controls
| Input | Action |
| :--- | :--- |
| **Mouse Wheel UP** / **Right Click** / `↑` / `k` | **Shift UP (+1 Virtual Gear)** — harder resistance |
| **Mouse Wheel DOWN** / **Left Click** / `↓` / `j` | **Shift DOWN (-1 Virtual Gear)** — easier resistance |
| **Middle Click** | Reset Virtual Gear to **Base Level 18** (or **8** when `--max-level 15`) |
| `-` / `+` | Decrease / Increase **Slope Sensitivity** (`+Lvl per 1% grade`, Default: `+1.25`) |
| `p` | Cycle **Hardware Protocol Mode** (`AUTO` → `FTMS` → `UART` → `BOTH`) live |
| `s` | Toggle **Simulated Rider Mode** on/off |
| `q` | Cleanly unregister BLE advertisement, disconnect, and quit |
