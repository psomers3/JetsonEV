---
layout: default
---
# Choosing a Mode
JetsonEV has different modes that it may be set to upon construction. These are listed below, along with how they are used.
* [JetsonEV.xbox_mode](#xbox_mode)
* [JetsonEV.xbox_direct_mode](#xbox_direct_mode)
* [JetsonEV.xbox_forwarding_mode](#xbox_forwarding_mode)

## xbox_mode
This mode assumes the use of an xbox 360 controller and will automatically try and connect to one. Upon successfulconnection, the signals from the triggers and left joystick will be forwarded to controlling functions. The left joystick will control the steering. The right trigger will control forward movement while the left trigger controls reverse. This mode utilizes the ```set_motor_speed()``` function and relies on a very rough PD(-ish) controller to drive the vehicle at a set speed. The max speed is set by the ```max_speed_limit``` parameter in the constructor. This mode relies heavily on a good speed signal.

## xbox_direct_mode
This mode is similar to the [xbox_mode](#xbox_mode), however, instead of using the internal speed controller and the ```set_motor_speed()``` command, the signals from the triggers are sent directly as a percentage of desired motor speed. This results in a faster speed response and more consistent speed control. The maximum speed is limited by setting the ```max_duty_cycle``` parameter in the constructor. The internal speed controller will not be automatically turned on in this mode, so manually setting the ```set_motor_speed()``` will not result in any speed change.

## xbox_forwarding_mode
This mode will connect to the xbox controller, however, the user must provide functions to handle the signals recieved. The way this is done is by using the functions:

* ```set_trigger_l_func(function)```
* ```set_trigger_r_func(function)```
* ```set_l_joystick_func(function)```

where ```function``` is the function handle that will recieve an Axis object (see xbox360controller package). The speed controller is not automatically turned on in this mode.