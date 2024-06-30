# VESC Introduction
VESC is the Open Source Electronic Speed Controler developed by Benjamin Vedder ( **V**edder **E**lectronic **S**peed **C**ontroller)


## Default Config for VESC
Default VESC app config is vesc_app_config.xml
Default Motor config is vesc_motor_config_12kw_260_V4.xml or vesc_motor_config_12kw_273.xml
**PLEASE NOTE:** don't just take the standard motor config and upload to your VESC. Take it as an example only.
Make sure to run the **"Setup Motor FOC"** wizard for the VESC tool to properly detect internal resistances and other values.

## Line auto stop in VESC
Line auto stop can be implemented within VESC with vesc_ppm_auto_stop.patch

For this to work properly, either connect a Potentiometer to ADC2 and GND to manually control the winch. E.g. To wind up the last meters of the line when finishing. Or to manually set a tension when used as a rewind winch. Note that the potentiometer only reduces tension/speed of the motor when it is running one of the pull programs as controlled via the transmitter!

IMPORTANT: If you do not install a Potentionmeter, connect ADC2 to GND.
