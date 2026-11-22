# HugBot

HugBot is an RC control system for battlebots built with inexpensive ESP32 boards.

<img width="4080" height="3072" alt="HugBot system" src="https://github.com/user-attachments/assets/2a70d29a-e5e6-4572-9285-251399df3ff9" />

## Setup
### Flashing the RX
1. Set the board to `ESP32C6 Dev Module`.
2. Enable `ACM on boot`.
3. Upload `HugRX/HugRX.ino`.
4. Open the serial monitor.
5. Copy the MAC address displayed by the receiver.

### Flashing the TX

1. Set the board to `ESP32C6 Dev Module`.
2. Enable `ACM on boot`.
3. Copy `HugTX/secrets.example.h` to `HugTX/secrets.h`.
4. Replace the address in `HugTX/secrets.h` with the receiver MAC address.
5. Upload `HugTX/HugTX.ino`.
6. Open the serial monitor and type `calibrate` to run joystick calibration.


## BOM
| Item              | Qty |
| ----------------- | --- |
| ESP32C6 supermini | 2   |
| Analog Joystick   | 2   |
| 2ch, 4in H-bridge | 1   |

An OLED screen and a case are optional.
