# Table of Contents

* [JetsonEV.JetsonEV](#.JetsonEV.JetsonEV)
  * [SPEED\_PORT](#.JetsonEV.JetsonEV.JetsonEV.SPEED_PORT)
  * [CAMERA\_PORT](#.JetsonEV.JetsonEV.JetsonEV.CAMERA_PORT)
  * [IMU\_PORT](#.JetsonEV.JetsonEV.JetsonEV.IMU_PORT)
  * [LIDAR\_PORT](#.JetsonEV.JetsonEV.JetsonEV.LIDAR_PORT)
  * [ARDUINO\_CONFIG](#.JetsonEV.JetsonEV.JetsonEV.ARDUINO_CONFIG)
  * [\_\_init\_\_](#.JetsonEV.JetsonEV.JetsonEV.__init__)
  * [start\_speed\_control](#.JetsonEV.JetsonEV.JetsonEV.start_speed_control)
  * [stop\_speed\_control](#.JetsonEV.JetsonEV.JetsonEV.stop_speed_control)
  * [shutdown](#.JetsonEV.JetsonEV.JetsonEV.shutdown)
  * [start\_sockets](#.JetsonEV.JetsonEV.JetsonEV.start_sockets)
  * [set\_trigger\_l\_func](#.JetsonEV.JetsonEV.JetsonEV.set_trigger_l_func)
  * [set\_trigger\_r\_func](#.JetsonEV.JetsonEV.JetsonEV.set_trigger_r_func)
  * [set\_l\_joystick\_func](#.JetsonEV.JetsonEV.JetsonEV.set_l_joystick_func)
  * [set\_r\_joystick\_func](#.JetsonEV.JetsonEV.JetsonEV.set_r_joystick_func)
  * [set\_motor\_speed](#.JetsonEV.JetsonEV.JetsonEV.set_motor_speed)
  * [set\_motor\_percent](#.JetsonEV.JetsonEV.JetsonEV.set_motor_percent)
  * [set\_steering\_percent](#.JetsonEV.JetsonEV.JetsonEV.set_steering_percent)
  * [get\_imu\_values](#.JetsonEV.JetsonEV.JetsonEV.get_imu_values)
  * [get\_lidar\_values](#.JetsonEV.JetsonEV.JetsonEV.get_lidar_values)
  * [get\_camera\_frame](#.JetsonEV.JetsonEV.JetsonEV.get_camera_frame)
  * [get\_speed](#.JetsonEV.JetsonEV.JetsonEV.get_speed)
  * [add\_timed\_task](#.JetsonEV.JetsonEV.JetsonEV.add_timed_task)
  * [start\_all\_tasks](#.JetsonEV.JetsonEV.JetsonEV.start_all_tasks)
  * [calibrate\_imu](#.JetsonEV.JetsonEV.calibrate_imu)

<a name=".JetsonEV.JetsonEV"></a>
## JetsonEV.JetsonEV

<a name=".JetsonEV.JetsonEV.JetsonEV.SPEED_PORT"></a>
#### SPEED\_PORT

Constant: TCP port being used to send speed measurement

<a name=".JetsonEV.JetsonEV.JetsonEV.CAMERA_PORT"></a>
#### CAMERA\_PORT

Constant: TCP port being used to send camera frames

<a name=".JetsonEV.JetsonEV.JetsonEV.IMU_PORT"></a>
#### IMU\_PORT

Constant: TCP port being used to send IMU measurements

<a name=".JetsonEV.JetsonEV.JetsonEV.LIDAR_PORT"></a>
#### LIDAR\_PORT

Constant: TCP port being used to send lidar measurements

<a name=".JetsonEV.JetsonEV.JetsonEV.ARDUINO_CONFIG"></a>
#### ARDUINO\_CONFIG

Constant: Dictionary of pin configuration to use for arduino control. Default values are good to use for a MEGA.
These values may be changed before constructing JetsonEV.
* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.__init__"></a>
#### \_\_init\_\_

```python
 | __init__(mode='xbox', initialize_imu=True, imu_bus=0, initialize_camera=False, initialize_lidar=False, max_speed_limit=5, max_duty_cycle=0.3, rc_control='VESC_CONTROL', rc_communication='/dev/ttyTHS1')
```

**Arguments**:

- `mode`: Control mode for the vehicle. Possible values are:

>- JetsonEV.xbox_mode
>- JetsonEV.xbox_direct_mode
>- JetsonEV.xbox_forwarding_mode

>See [here](./modes.html) for more details on each mode.

- `initialize_imu`: Whether or not to initialize imu

- `imu_bus`: I2C bus number to communicate with the IMU

- `initialize_camera`: Whether or not to initialize camera

- `initialize_lidar`: Whether or not to initialize rplidar

- `max_speed_limit`: Max speed in m/s when using speed controller.

- `max_duty_cycle`: Max duty cycle to send when using direct duty cycle control.

- `rc_control`: Type of hardware control for rc components (motor and steering servo). Can be either
JetsonEV.VESC_CONTROL or JetsonEV.ARDUINO_CONTROL.

- `rc_communication`: Port or bus to use for rc_control. If using VESC, this should point to the UART port.
If using USB for UART control, set rc_communication = 'USB' (linux only).
If rc_control=JetsonEV.ARDUINO, this should be an integer for the I2C bus number.

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.start_speed_control"></a>
#### start\_speed\_control

```python
 | start_speed_control()
```

Starts the internal speed controller. Speed is controlled with set_motor_speed(). This may be called
automatically depending on mode.
* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.stop_speed_control"></a>
#### stop\_speed\_control

```python
 | stop_speed_control()
```

Stops the internal speed controller.
* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.shutdown"></a>
#### shutdown

```python
 | shutdown()
```

Shuts down the car. It is necessary to call this before this object goes out of scope in order to shut down
the hardware properly.
* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.start_sockets"></a>
#### start\_sockets

```python
 | start_sockets()
```

Starts all the TCP sockets in _output_sockets. This is called automatically in the constructor.
* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.set_trigger_l_func"></a>
#### set\_trigger\_l\_func

```python
 | set_trigger_l_func(function)
```

Use this function when in xbox_forwarding_mode to assign a function to handle a signal from the left trigger.

**Arguments**:

- `function`: function that takes one argument that will be an Axis object from the xbox360controller

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.set_trigger_r_func"></a>
#### set\_trigger\_r\_func

```python
 | set_trigger_r_func(function)
```

Use this function when in xbox_forwarding_mode to assign a function to handle a signal from the right trigger.

**Arguments**:

- `function`: function that takes one argument that will be an Axis object from the xbox360controller

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.set_l_joystick_func"></a>
#### set\_l\_joystick\_func

```python
 | set_l_joystick_func(function)
```

Use this function when in xbox_forwarding_mode to assign a function to handle a signal from the left joystick.

**Arguments**:

- `function`: function that takes one argument that will be an Axis object from the xbox360controller

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.set_r_joystick_func"></a>
#### set\_r\_joystick\_func

```python
 | set_r_joystick_func(function)
```

Use this function when in xbox_forwarding_mode to assign a function to handle a signal from the right joystick.

**Arguments**:

- `function`: function that takes one argument that will be an Axis object from the xbox360controller

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.set_motor_speed"></a>
#### set\_motor\_speed

```python
 | set_motor_speed(new_speed)
```

The value set here will be assigned to self.desired_speed. This is only effective in xbox_mode or if the
internal speed controller is turned on with start_speed_control.

**Arguments**:

- `new_speed`: new speed in m/s

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.set_motor_percent"></a>
#### set\_motor\_percent

```python
 | set_motor_percent(percentage)
```

This function will directly send a pwm signal to the motor controller. Do not use this while the internal
speed control is running (i.e. DO NOT use while in xbox_mode or if start_speed_control() has been called).

**Arguments**:

- `percentage`: motor duty cycle to use between -1 (reverse) and 1 (forward)

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.set_steering_percent"></a>
#### set\_steering\_percent

```python
 | set_steering_percent(percentage)
```

**Arguments**:

- `percentage`: steering servo percentage between -1 and 1

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.get_imu_values"></a>
#### get\_imu\_values

```python
 | get_imu_values()
```

:returns Latest measurement from the IMU.

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.get_lidar_values"></a>
#### get\_lidar\_values

```python
 | get_lidar_values()
```

**Returns**:

Latest measurement from the Lidar sensor.

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.get_camera_frame"></a>
#### get\_camera\_frame

```python
 | get_camera_frame()
```

**Returns**:

Latest camera frame.

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.get_speed"></a>
#### get\_speed

```python
 | get_speed()
```

**Returns**:

Latest speed measurement (m/s)

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.add_timed_task"></a>
#### add\_timed\_task

```python
 | add_timed_task(calling_function, object_to_wrap=None, sampling_rate=10, calling_func_args=[], forwarding_function=None, verbose=False)
```

This is a helper function to create a TimedTask object. The only difference between using this and making a
TimedTask directly, is that the resulting new task is also added to JetsonEV._timed_tasks which are all
automatically stopped when shutdown() is called.

**Arguments**:

- `calling_function`: The function that will be called at the set interval.
:type calling_function: function

- `object_to_wrap`: An instance of an object that is associated with the repetitive task. References to
this object can be obtained from two member attributes of TimedTask under the name "wrapped_object" and
the name of the type of the object that is passed (i.e. if an object of type "Sensor" is passed, then the
reference would be "myTimedTask.Sensor"). This can be set as None if you do not want this feature or are
not using an object.
:type object_to_wrap: object

- `sampling_rate`: The frequency in Hz of task loop.
:type sampling_rate: float

- `calling_func_args`: A list of any arguments that need to be passed to the calling_function when it is
called.
:type calling_func_args: list

- `forwarding_function`: An additional optional function that can be specified to be called every loop.
:type forwarding_function: function

- `verbose`: Whether or not to print errors from the task (i.e. task is not keeping up to desired run speed).

:returns The created TimedTask object.

* * *

<a name=".JetsonEV.JetsonEV.JetsonEV.start_all_tasks"></a>
#### start\_all\_tasks

```python
 | start_all_tasks()
```

Starts all the created TimedTasks that are being tracked by JetsonEV. This is called in the constructor.
* * *

<a name=".JetsonEV.JetsonEV.calibrate_imu"></a>
#### calibrate\_imu

```python
calibrate_imu(bus=0)
```

A function to zero out the gyros and orient the accelerometer to gravity. DOES NOT DO ANYTHING YET

**Arguments**:

- `bus`: I2C Bus to communicate with the imu

* * *

