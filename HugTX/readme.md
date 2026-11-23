# Transmitter

This is the transmitter side of the HugBot controller.

**DO NOT SWITCH ON THE BATTERY WHILE HAVING A USB CABLE IN THE USB-C PORT!**

| Pin         | Function                        | Interface / Mode           |
| :---------- | :------------------------------ | :------------------------- |
| **5V**      | Battery                         | Power (`5V` / VBUS)        |
| **G (GND)** | Common Ground                   | Power (`GND`)              |
| **3.3V**    | Joystick, OLED VCC, Divider Top | Power (`3.3V`)             |
| **GPIO0**   | Axis Y1 - Joystick1 Axis X      | Analog (`ADC1_CH0`)        |
| **GPIO1**   | Axis X1 - Joystick1 Axis Y      | Analog (`ADC1_CH1`)        |
| **GPIO2**   | Battery Voltage                 | Analog (`ADC1_CH2`, strap) |
| **GPIO3**   | Axis Y2 - Joystick2 Axis X      | Analog (`ADC1_CH3`)        |
| **GPIO4**   | Axis X2 - Joystick2 Axis Y      | Analog (`ADC1_CH4`)        |
| **GPIO5**   | *Unconnected*                   | *N/A*                      |
| **GPIO6**   | *Unconnected*                   | *N/A*                      |
| **GPIO7**   | *Unconnected*                   | *N/A*                      |
| **GPIO8**   | I2C SDA                         | Digital (`I2C`)            |
| **GPIO9**   | I2C SCL                         | Digital (`I2C`)            |
| **GPIO10**  | Button 1                        | Digital (`INPUT_PULLUP`)   |
| **GPIO20**  | *Unconnected*                   | *N/A*                      |
| **GPIO21**  | Button 2                        | Digital (`INPUT_PULLUP`)   |

Hold both buttons for one second to open or close Settings. Navigate with the
left stick: up or right moves toward the top/previous option, down or left moves
toward the bottom/next option. Use the right stick right to select/save and left
to cancel. Motor outputs stay neutral in menus and for 500 ms after returning
to Home.

Settings, in menu order:

- **Model select:** Choose a configured receiver. The first model is the
  default; the selected model is saved.
- **Control mode:** Choose Tank (each stick controls its motor) or Mixed (left
  stick vertical controls throttle, right stick horizontal controls steering).
- **Calibrate sticks:** Recalibrate joystick centers and ranges.
- **Battery factor:** Adjust in 0.01 steps; pushing the left stick farther
  repeats changes faster. The header voltage previews the edited value.
- **Erase settings:** Restore calibration, model, control mode, and battery
  factor defaults after confirmation.
- **Back:** Return to Home.

Configure at least one receiver in the `modelList` array in `secrets.h`. Each
entry contains a friendly name and six-byte MAC address; see
[`secrets.example.h`](secrets.example.h) for the format.
