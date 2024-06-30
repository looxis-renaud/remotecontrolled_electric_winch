# VESC Introduction
VESC is the Open Source Electronic Speed Controler developed by Benjamin Vedder ( **V**edder **E**lectronic **S**peed **C**ontroller).

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
