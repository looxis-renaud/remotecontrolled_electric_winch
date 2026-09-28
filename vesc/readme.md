# VESC Introduction
VESC is the Open Source Electronic Speed Controler developed by Benjamin Vedder ( **V**edder **E**lectronic **S**peed **C**ontroller).

# The controller used on this winch: Trampa VESC 75/300

The winch runs on a **Trampa VESC 75/300** (hardware revision `75_300_R3`), driving the QS Motor 12kW 260 V4 hub motor in FOC mode.
It is flashed with the patched autostop firmware [vesc_75_300_auto_stop.bin](vesc_75_300_auto_stop.bin) (see "Line auto stop in VESC" below).

Manufacturer manual (PDF): [VESC-75-300-MKIV-MANUAL.pdf](VESC-75-300-MKIV-MANUAL.pdf)

Product page: https://trampaboards.com/vesc-75v-300a-black-anodised-non-conductive-cnc-housing--the-most-powerful-vedder-electronic-speed-controller-ever-p-26284.html

## What matters for the winch

| Topic | Controller spec | What it means for this winch |
|-------|-----------------|------------------------------|
| Voltage | 12V – 67.2V (4S–16S LiPo/Li-ion), spikes must never exceed **75V** | Our 16S battery is at exactly **67.2V** when fully charged, which is the upper limit. Braking (brake states, autostop) is regenerative and feeds energy back into the battery, raising the voltage. Braking hard with a freshly, fully charged battery is therefore the most critical case for overvoltage. Never charge above 4.2V/cell. |
| Current | 300A continuous, 400A burst, **depending on mounting, ambient temperature and air circulation** | The continuous rating assumes good cooling. That is why the VESC has a cooling fan (see main README), and why the enclosure needs airflow around the heat sink. |
| Regenerative braking | Energy is recovered during braking | Brake states (-1 soft / -2 hard) and autostop brake with motor current; see voltage note above. |
| Protection | Under/over voltage, over current, over temperature (motor and ESC) | If a limit is hit, the VESC reduces or cuts motor power, including **during a tow**. Keep the battery charged and the controller cool. |
| Real-time data | Motor temperature, current, voltage | The receiver reads battery voltage, motor temperature, tachometer and duty cycle via UART and shows them on the remote. |
| Aux power outputs | 12V 1A (switchable), 5V 1A, 3.3V 0.5A; **all combined max. 1A** | Everything powered from these outputs (e.g. cooling fan, relay, potentiometer, receiver, depending on the build) shares this 1A budget. |

## Technical specifications (manufacturer data)

**Voltage**
- 12V – 67.2V (safe for 4S to 16S LiPo/Li-ion). Voltage spikes may not exceed 75V
- 12V 1A switchable output for external electronics
- 5V 1A output for external electronics
- 3.3V 0.5A output for external electronics
- Combined 3.3V, 5V, 12V: no more than 1A

**Current**
- Continuous 300A, burst 400A (values depend on the mounting, ambient temperature and air/water circulation around the device)

**Motor control modes**
- DC, BLDC, FOC (sinusoidal). This winch uses **FOC**.
- Sensored, sensorless or hybrid operation
- Sensorless modes: HFI, VSS, 45 Deg V0V7 HFI (Silent), 45 Deg V0 HFI, Coupled V0V7 HFI (Silent), Coupled V0 HFI
- High ERPM drivable: 100–150K (motor/system dependent)

**Supported sensors**
- Hall sensors
- Encoders: ABI, AS5047, AS5X47U, SIN/COS, TS5700N8501 (incl. multiturn), MT6816, BISSC, TLE5102, custom encoder

**Communication**
- USB, SWD
- PWM in/out (this winch: PPM input from the receiver on the "Servo" port)
- UART x2 (this winch: telemetry to the receiver on the COMM port)
- SPI and I²C
- CAN, UAVCAN and custom CAN commands
- Wireless connectivity via accessory (WiFi/BLE)
- 2x programmable GPIO pins

**Other technical features**
- Current and voltage measurement on all phases (3 phase shunts), adjustable current and voltage filters (full phase filters)
- 3 individual gate drivers
- Accelerometer and gyro (9 axis, ±2/±4/±8/±16 g full scale)
- Hibernation with wake-up via power switch options (momentary NC), automatic hibernation with adjustable timer, 20µA consumption while hibernating (less than battery self-discharge)

**Software**
- VESC Tool: https://vesc-project.com/vesc_tool (desktop), mobile apps for Android/iOS
- Motor and input setup wizards
- Scripting support (QML and LISP)

**Housing**
- Precision CNC aluminium heat sink housing, black hard anodised
- Mounting holes for easy attachment
- Outer dimensions: 141 x 82 x 18 mm

*Source: Trampa product page (manufacturer data), retrieved September 2026.*

# VESC Software
The Vesc Software contains several sections, to setup the VESC and Motor we'll need the following sections 

## Welcome & Wizards
We will need the two wizards **Setup Input** and **Setup Motor FOC**, they are needed to set up the PWM Remote and read Some Motor Values such as Resistance, Inductance, Flux Linkage and a bunch of other values (I do understand basically none of them)

## Motor Settings
This is where you can edit your motor settings. It is very important to setup your VESC every time you connect a different motor, otherwise the VESC and/or the motor are likely to get damaged. The easiest way to set up your VESC for your motor is to use the Motor Setup Wizard. This wizard can be accessed from the welcome page, from the help menu or using the button at the bottom of this page.
The motor settings are stored in their own configuration structure. Every time you make changes to the motor configuration you have to write the configuration to the VESC in order to apply the new settings. Reading/writing the motor configuration can be done using the buttons on the toolbar to the right.

## App Settings
This is where you can edit your app settings. The VESC can run one or more apps, and the apps are used to enable different functions on the communication interfaces of the VESC. If you are going to use your VESC with USB or CAN-bus you don't have to change the app configuration since these interfaces always are active. If you want to use conventional input devices such as nunchuks, ebike throttles or RC remote controllers you have to configure the apps accordingly.
The easiest way to configure your VESC for conventional input devices is to use the Input Setup Wizard. This wizard can be accessed from the welcome page, from the help menu or using the button at the bottom of this page.
The app settings are stored in their own configuration structure. Every time you make changes to the app configuration you have to write the configuration to the VESC in order to apply the new settings. Reading/writing the app configuration can be done using the buttons on the toolbar to the right. The functions of these toolbar buttons are the following:


## Default Config for VESC
Default VESC app config is vesc_app_config.xml
Default Motor config is vesc_motor_config_12kw_260_V4.xml or vesc_motor_config_12kw_273.xml
**PLEASE NOTE:** don't just take the standard motor config and upload to your VESC. Take it as an example only.
Make sure to run the **"Setup Motor FOC"** wizard for the VESC tool to properly detect internal resistances and other values.

## Line auto stop in VESC
Line auto stop can be implemented within VESC with vesc_ppm_auto_stop.patch

For this to work properly, either connect a Potentiometer to ADC2 and GND to manually control the winch. E.g. To wind up the last meters of the line when finishing. Or to manually set a tension when used as a rewind winch. Note that the potentiometer only reduces tension/speed of the motor when it is running one of the pull programs as controlled via the transmitter!

IMPORTANT: If you do not install a Potentionmeter, connect ADC2 to GND.
