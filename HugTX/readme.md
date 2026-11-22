# Transmitter

This is the transmitter side of the HugBot controller.

**DO NOT SWITCH ON THE BATTERY WHILE HAVING A USB CABLE IN THE USB-C PORT!**

| Pin         | Function                        | Interface / Mode         |
| :---------- | :------------------------------ | :----------------------- |
| **5V**      | Battery                         | Power (`5V` / VBUS)      |
| **G (GND)** | Common Ground                   | Power (`GND`)            |
| **3.3V**    | Joystick, OLED VCC, Divider Top | Power (`3.3V`)           |
| **GPIO0**   | Joystick1 Axis X                | Analog (`ADC1_CH0`)      |
| **GPIO1**   | Joystick1 Axis Y                | Analog (`ADC1_CH1`)      |
| **GPIO2**   | *Unconnected*                   | bootstrap                |
| **GPIO3**   | Joystick2 Axis X                | Analog (`ADC1_CH3`)      |
| **GPIO4**   | Joystick2 Axis Y                | Analog (`ADC1_CH4`)      |
| **GPIO5**   | Battery Voltage                 | Analog (`ADC2_CH0`)      |
| **GPIO6**   | *Unconnected*                   | *N/A*                    |
| **GPIO7**   | *Unconnected*                   | *N/A*                    |
| **GPIO8**   | I2C SDA                         | Digital (`I2C`)          |
| **GPIO9**   | I2C SCL                         | Digital (`I2C`)          |
| **GPIO10**  | Button 1                        | Digital (`INPUT_PULLUP`) |
| **GPIO20**  | *Unconnected*                   | *N/A*                    |
| **GPIO21**  | Button 2                        | Digital (`INPUT_PULLUP`) |