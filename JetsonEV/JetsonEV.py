from ArduinoI2C import ArduinoI2CBus
from fastestrplidar import RPLidar
from nanocamera.NanoCam import Camera
from DataSocket import SendSocket, ReceiveSocket, JSON, NUMPY
from TimedTask import TimedTask
from xbox360controller import Xbox360Controller
from imu.mpu9250 import MPU6050
from pyvesc import VESC
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
import threading
import signal
import time
import os
import math
import numpy as np
from .utilities import run_calibration, get_calibration

MPU6050._chip_id = 113  # this is only necessary because only the MPU6050 is being used from a MPU9250

# calculate a static value to convert from motor rpm to vehicle speed in m/s
poles_on_motor = 6
wheel_radius = 0.02  # meters
__motor_rotations_per_wheel_rotation = 48 / poles_on_motor
speed_conversion = (wheel_radius * 2 * math.pi) / (60 * __motor_rotations_per_wheel_rotation)

gains = {'VESC_CONTROL': {'p': 0.005, 'i': 0.000, 'd': 0.000},
         'ARDUINO_CONTROL': {'p': 0.008, 'i': 0.000, 'd': 0.004}}

ip_address = ''


class JetsonEV(object):
    xbox_mode = 'xbox'
    xbox_direct_mode = 'xbox_direct'
    xbox_forwarding_mode = 'xbox_forwarding'
    __running_car__ = None

    VESC_CONTROL = 'VESC_CONTROL'
    ARDUINO_CONTROL = 'ARDUINO_CONTROL'

    # Default TCP ports for out-going data
    SPEED_PORT = 4020
    """Constant: TCP port being used to send speed measurement"""

    IMU_PORT = 5025
    """Constant: TCP port being used to send IMU measurements"""

    LIDAR_PORT = 5026
    """Constant: TCP port being used to send lidar measurements"""

    CAMERA_PORT = 5027
    """Constant: TCP port being used to send camera frames"""

    SLAM_PORT = 5028
    """Constant: TCP port being used to send SLAM data (x, y, theta, map)"""

    ARDUINO_CONFIG = {
        'steering_pin': 9,
        'motor_pwm_pin': 10,
        'motor_pin_a': 19,
        'motor_pin_b': 18,
        'motor_pin_c': 2
    }
    """Constant: Dictionary of pin configuration to use for arduino control. Default values are good to use for a MEGA.
    These values may be changed before constructing JetsonEV.
    * * *
    """

    def __init__(self,
                 mode='xbox',
                 initialize_imu=True,
                 imu_bus=0,
                 initialize_camera=False,
                 initialize_lidar=False,
                 slam_map_meters=15,
                 slam_map_pixels=1000,
                 max_speed_limit=5,
                 max_duty_cycle=0.2,
                 rc_control='VESC_CONTROL',
                 rc_communication='/dev/ttyTHS1'):
        """
        :param mode: Control mode for the vehicle. Possible values are:

        >- JetsonEV.xbox_mode
        >- JetsonEV.xbox_direct_mode
        >- JetsonEV.xbox_forwarding_mode

        >See [here](./modes.html) for more details on each mode.

        :param initialize_imu: Whether or not to initialize imu

        :param imu_bus: I2C bus number to communicate with the IMU

        :param initialize_camera: Whether or not to initialize camera

        :param initialize_lidar: Whether or not to initialize rplidar

        :param slam_map_meters: The distance in meters to use for the SLAM map size. A square map is
                                   made using this distance for each side.

        :param slam_map_pixels: The SLAM square map size in pixels. This controls the resolution of the map.

        :param max_speed_limit: Max speed in m/s when using speed controller.

        :param max_duty_cycle: Max duty cycle to send when using direct duty cycle control.

        :param rc_control: Type of hardware control for rc components (motor and steering servo). Can be either
                           JetsonEV.VESC_CONTROL or JetsonEV.ARDUINO_CONTROL.

        :param rc_communication: Port or bus to use for rc_control. If using VESC, this should point to the UART port.
                                 If using USB for UART control, set rc_communication = 'USB' (linux only).
                                 If rc_control=JetsonEV.ARDUINO, this should be an integer for the I2C bus number.

        * * *
        """

        JetsonEV.__running_car__ = self
        self.rc_control = rc_control

        self._timed_tasks = []
        """private list to hold all existing TimedTask objects"""

        self._output_sockets = []
        """private list to hold all existing output TCP sockets objects"""

        self.max_speed_limit = max_speed_limit
        self.duty_cycle_limit = max_duty_cycle

        self._speed = [0]
        self._speed_lock = threading.Lock()

        '''************** Initiate Steering and Motor ****************'''
        try:
            self._initialize_rc_control(rc_communication)
        except OSError as e:
            print(e)
        '''*********************************************************'''

        '''****************** Initiate Camera **********************'''
        self.camera = None
        if initialize_camera:
            print('initiating Camera... ', end='')
            try:
                camera = Camera(flip=2)
                self._latest_frame = [0]
                self._latest_frame_lock = threading.Lock()

                self.camera_socket = SendSocket(tcp_port=JetsonEV.CAMERA_PORT, tcp_ip=ip_address)
                self._output_sockets.append(self.camera_socket)

                def update_frame(frame):
                    self.latest_frame = frame
                    self.camera_socket.send_data(frame)

                # wrap the object in a TimedTask set to update at fixed rate
                self.camera = self.add_timed_task(calling_function=camera.read,
                                                  object_to_wrap=camera,
                                                  forwarding_function=update_frame,
                                                  sampling_rate=120)

            except Exception as e:
                print(e)
            print('success')
        '''*********************************************************'''

        '''******************* Initiate IMU ************************'''
        self.imu: TimedTask
        if initialize_imu:
            try:
                imu = MPU6050(bus=imu_bus, device_addr=0x68)  # create the imu device
                print('Successfully connected to IMU')
                self._imu_values = [0]
                self._imu_values_lock = threading.Lock()

                # wrap the object in a TimedTask set to update at fixed rate
                self.imu = self.add_timed_task(calling_function=self._get_imu_values,
                                               object_to_wrap=imu,
                                               sampling_rate=50)

                self.imu_socket = SendSocket(tcp_port=JetsonEV.IMU_PORT, tcp_ip=ip_address)
                self._output_sockets.append(self.imu_socket)

                imu_config_dict = get_calibration()
                self._imu_rotation = np.array(imu_config_dict['rotation_matrix'])
                self._gravity_offset = imu_config_dict['gravity']
                self._gyro_offset = np.array(imu_config_dict['gyro_offsets'])

            except Exception as e:
                print(e)
                print('ERROR: Could not connect to IMU')
                self.imu = None
                if hasattr(self, 'imu_socket'):
                    self._output_sockets.remove(self.imu_socket)
        '''*********************************************************'''

        '''****************** Initiate Lidar ***********************'''
        self.lidar = None
        self._theta_lock = threading.Lock()
        self._theta = 0
        self._x_lock = threading.Lock()
        self._x = 0
        self._y_lock = threading.Lock()
        self._y = 0
        self._map_lock = threading.Lock()
        self._map = bytearray(slam_map_pixels * slam_map_pixels)
        self._breezyslam = RMHC_SLAM(LaserModel(),
                                     map_size_meters=slam_map_meters,
                                     map_size_pixels=slam_map_pixels,
                                     hole_width_mm=500)

        if initialize_lidar:
            started = threading.Event()  # threading event to watch if lidar started

            # This is done to prevent the script from locking up if the lidar needs to be restarted
            def check_lidar():
                start_time = time.time()
                while not started.isSet():
                    time.sleep(0.2)
                    if time.time() - start_time > 10:
                        os.kill(os.getpid(), signal.SIGTERM)
                        raise Exception('Unable to start Lidar.')

            initialize_thread = threading.Thread(target=check_lidar)
            initialize_thread.start()
            try:
                print('connecting to lidar...', end='')
                lidar = RPLidar(scan_mode=0)
                started.set()
                try:
                    initialize_thread.join()
                except Exception as e:
                    raise e
            except Exception as e:
                print(e)
                print('ERROR: Unable to connect to lidar sensor.')
                os.kill(os.getpid(), signal.SIGTERM)

            self._latest_lidar_scan = lidar.get_scan_as_xy(filter_quality=True)
            self._latest_lidar_scan_lock = threading.Lock()
            print('success')

            self.lidar_socket = SendSocket(tcp_port=JetsonEV.LIDAR_PORT, tcp_ip=ip_address)
            self._output_sockets.append(self.lidar_socket)

            self.slam_socket = SendSocket(tcp_port=JetsonEV.SLAM_PORT, tcp_ip=ip_address)
            self._output_sockets.append(self.slam_socket)
            self._update_map = [True]
            self._save_criteria_dist = 100  # must move ** mm before updating map
            self._save_criteria_angle = 20  # must rotate more than ** deg before updating map

            def update_scan(scan):
                scan = np.array(scan)
                self.latest_lidar_scan = scan
                self._breezyslam.update(scan[:, 1].tolist(),
                                        scan_angles_degrees=scan[:, 0].tolist(),
                                        should_update_map=self._update_map[0])
                x, y, theta = self._breezyslam.getpos()
                dist = np.sqrt((x - self.x) ** 2 + (y - self.y) ** 2)
                del_theta = abs(theta - self.theta)
                if dist > self._save_criteria_dist or del_theta > self._save_criteria_angle:
                    self._update_map[0] = True
                else:
                    self._update_map[0] = False
                self.x = x
                self.y = y
                self.theta = theta
                with self._map_lock:
                    self._breezyslam.getmap(self._map)

                self.lidar_socket.send_data(scan)
                self.slam_socket.send_data({'x': self.x,
                                            'y': self.y,
                                            'theta': self.theta,
                                            'map': np.frombuffer(self.map, dtype=np.uint8),
                                            'meters': slam_map_meters,
                                            'pixels': slam_map_pixels})

            self.lidar = self.add_timed_task(calling_function=lidar.get_scan_as_vectors,
                                             calling_func_args=[True],
                                             object_to_wrap=lidar,
                                             forwarding_function=update_scan,
                                             sampling_rate=5)

            print("initializing SLAM map")
            for _ in range(10):
                scan = np.array(lidar.get_scan_as_vectors(True))
                self._breezyslam.update(scan[:, 1].tolist(),
                                        scan_angles_degrees=scan[:, 0].tolist(),
                                        should_update_map=True)
                time.sleep(0.05)

        '''*********************************************************'''

        '''************ Initiate XBOX Controller *******************'''
        self.xbox_controller: Xbox360Controller
        self._initialize_xbox_controller(mode)
        '''*********************************************************'''

        '''***************** Setup speed control *******************'''
        self._desired_speed = [0]
        self.last_error = 0
        self._current_duty = 0
        self._integrated_speed_error = 0

        self._desired_speed_lock = threading.Lock()
        self._speed_control_task = TimedTask(calling_function=self._speed_control, sampling_rate=20)

        if mode in [JetsonEV.xbox_mode]:
            # define an in between function to convert percentage from xbox controller to speed using max speed limit.
            def percent_to_speed(percent):
                self.set_motor_speed(percent * self.max_speed_limit)

            self._xbox_speed_func = percent_to_speed
            self.duty_cycle_limit = 1
            self.start_speed_control()
        elif mode in [JetsonEV.xbox_direct_mode]:
            self._xbox_speed_func = self.set_motor_percent
        '''*********************************************************'''

        print('Starting Vehicle...', end='')
        self.start_sockets()
        self.start_all_tasks()
        print('running.')

    def start_speed_control(self):
        """Starts the internal speed controller. Speed is controlled with set_motor_speed(). This may be called
        automatically depending on mode.
        * * *
        """
        self._speed_control_task.start()

    def stop_speed_control(self):
        """Stops the internal speed controller.
        * * *
        """
        self._speed_control_task.stop()

    def _speed_control(self):
        # NOT PROPER (OR AT LEAST GOOD) IMPLEMENTATION OF PID CONTROL
        if abs(self.speed) < 0.4 and abs(self.desired_speed) < 0.5:
            self.set_motor_percent(0)
            self._current_duty = 0
            return

        error = self.desired_speed - self.speed
        error_rate = error - self.last_error
        self.last_error = error
        self._integrated_speed_error += error

        self._current_duty = self._current_duty + \
                             error * gains[self.gain_id]['p'] + \
                             error_rate * gains[self.gain_id]['d'] + \
                             self._integrated_speed_error * gains[self.gain_id]['i']

        # print("speed: {:.3f}, desired: {:.3f}, duty:{:.3f}".format(self.speed, self.desired_speed, self._current_duty))
        self.set_motor_percent(self._current_duty)

    @property
    def x(self):
        with self._x_lock:
            return self._x

    @x.setter
    def x(self, new_x):
        with self._x_lock:
            self._x = new_x

    @property
    def y(self):
        with self._y_lock:
            return self._y

    @y.setter
    def y(self, new_y):
        with self._y_lock:
            self._y = new_y

    @property
    def theta(self):
        with self._theta_lock:
            return self._theta

    @theta.setter
    def theta(self, new_theta):
        with self._theta_lock:
            self._theta = new_theta

    @property
    def map(self):
        with self._map_lock:
            return self._map.copy()

    @property
    def desired_speed(self):
        """ The currently stored desired speed (m/s)
        * * *
        """
        with self._desired_speed_lock:
            return self._desired_speed[0]

    @desired_speed.setter
    def desired_speed(self, new_speed):
        with self._desired_speed_lock:
            self._desired_speed[0] = new_speed

    @property
    def speed(self):
        """ Holds the current vehicle speed measurement.
        * * *"""
        with self._speed_lock:
            return self._speed[0]

    @speed.setter
    def speed(self, new_speed):
        with self._speed_lock:
            self._speed[0] = new_speed

    @property
    def latest_lidar_scan(self):
        """ Holds the latest complete lidar scan.
        * * *"""
        with self._latest_lidar_scan_lock:
            return self._latest_lidar_scan

    @latest_lidar_scan.setter
    def latest_lidar_scan(self, scan):
        with self._latest_lidar_scan_lock:
            self._latest_lidar_scan = scan

    @property
    def latest_frame(self):
        """ Holds the latest camera image.
        * * *"""
        with self._latest_frame_lock:
            return self._latest_frame[0]

    @latest_frame.setter
    def latest_frame(self, frame):
        with self._latest_frame_lock:
            self._latest_frame[0] = frame

    @property
    def imu_values(self):
        """ Holds the latest imu values.
        * * *"""
        with self._imu_values_lock:
            return self._imu_values[0]

    @imu_values.setter
    def imu_values(self, values):
        with self._imu_values_lock:
            self._imu_values[0] = values

    def _initialize_rc_control(self, communication):
        """This function creates the properties to control the motor and steering.
        :param communication: either 'VESC_CONTROL' for using the VESC or 'ARDUINO_CONTROL' for using an ArduinoI2CBus

        * * *
        """

        if self.rc_control == self.VESC_CONTROL:
            self.gain_id = JetsonEV.VESC_CONTROL
            if communication == 'USB':
                try:
                    for dev in os.listdir('/dev/serial/by-id'):
                        if dev.find('ChibiOS') >= 0:
                            communication = '/dev/serial/by-id/' + dev
                except FileNotFoundError:
                    raise Exception('Unable to find VESC device. Check port and power.')
            try:
                self.vesc = VESC(serial_port=communication)
            except Exception as e:
                raise e

            self._set_servo_percent = self.vesc.set_servo

            def vesc_motor_duty(percentage):
                # percentage should be between -1 and 1
                self.vesc.set_duty_cycle(percentage)

            self._set_motor_duty = vesc_motor_duty
            self._speed_socket = SendSocket(tcp_port=JetsonEV.SPEED_PORT, tcp_ip=ip_address)
            self._output_sockets.append(self._speed_socket)
            new_conversion = speed_conversion * poles_on_motor  # poles in motor

            def set_motor_speed(new_speed):
                self.speed = new_speed * new_conversion
                self._speed_socket.send_data(self.speed)

            self.motor_speed_task = self.add_timed_task(calling_function=self.vesc.get_rpm,
                                                        sampling_rate=20,
                                                        forwarding_function=set_motor_speed,
                                                        verbose=False)

        else:  # Use arduino
            self.gain_id = JetsonEV.ARDUINO_CONTROL
            self.arduino_devices = ArduinoI2CBus(communication)

            self.StWhl = self.arduino_devices.create_servo_dev(pin=JetsonEV.ARDUINO_CONFIG['steering_pin'])
            self.StWhl.attach()
            self._set_servo_percent = self.StWhl.write_percentage

            self.Motor = self.arduino_devices.create_servo_dev(pin=JetsonEV.ARDUINO_CONFIG['motor_pwm_pin'])
            self.Motor.attach()

            self._last_duty = [0]
            self._was_reversing = [False]

            # define function to handle a percentage from -1 to 1 and convert to regular ESC percentage
            def arduino_motor_duty(percentage):
                if percentage < 0 and self._last_duty[0] >= 0 and not self._was_reversing[0]:
                    self.Motor.write_percentage(.1)
                    time.sleep(0.15)
                    self.Motor.write_percentage(.5)
                    self._current_duty = 0
                    time.sleep(0.1)
                    self._was_reversing[0] = True

                self._last_duty[0] = percentage

                if percentage >= 0:
                    if percentage > 0:
                        self._was_reversing[0] = False
                    percentage = 0.5 * percentage + 0.5  # convert to percent of pwm above 50 %
                else:
                    percentage = 0.5 - 0.5 * abs(percentage)

                self.Motor.write_percentage(percentage)

            self._set_motor_duty = arduino_motor_duty

            encoder = self.arduino_devices.create_BLDC_encoder_dev(pin_A=JetsonEV.ARDUINO_CONFIG['motor_pin_a'],
                                                                   pin_B=JetsonEV.ARDUINO_CONFIG['motor_pin_b'],
                                                                   pin_C=JetsonEV.ARDUINO_CONFIG['motor_pin_c'])

            self._speed_socket = SendSocket(tcp_port=JetsonEV.SPEED_PORT, tcp_ip=ip_address)
            self._output_sockets.append(self._speed_socket)

            def set_motor_speed(new_speed):
                self.speed = new_speed * speed_conversion
                self._speed_socket.send_data(self.speed)

            self.motor_encoder = self.add_timed_task(calling_function=encoder.get_speed,
                                                     object_to_wrap=encoder,
                                                     sampling_rate=20,
                                                     forwarding_function=set_motor_speed,
                                                     verbose=True)

    def shutdown(self):
        """Shuts down the car. It is necessary to call this before this object goes out of scope in order to shut down
        the hardware properly.
        * * *
        """
        self.stop_speed_control()
        self.set_motor_percent(0)

        try:
            print('shutting down controller')
            self.xbox_controller.close()
        except AttributeError as e:
            print(e)

        if hasattr(self, 'vesc'):
            self.vesc.set_duty_cycle(0)
            self.vesc.stop_heartbeat()

        if hasattr(self, 'arduino_devices'):
            print('shutting down arduino')
            self.arduino_devices.clear_all_devices()

        print('closing ports')
        [x.stop() for x in self._output_sockets]

        print('stopping any tasks')
        [x.stop() for x in self._timed_tasks]

        if self.lidar is not None:
            self.lidar.wrapped_object.stopmotor()

        print('shutting down camera')
        self.close_camera()

    def start_sockets(self):
        """ Starts all the TCP sockets in _output_sockets. This is called automatically in the constructor.
        * * *"""
        [x.thread.start() for x in self._output_sockets]

    def _connect_controller(self):
        try:
            self.xbox_controller = Xbox360Controller(index=1, axis_threshold=0.05)
        except Exception as e:  # this library emits a stupid general exception that makes it difficult to retry
            try:
                self.xbox_controller = Xbox360Controller(index=2, axis_threshold=0.05)
            except Exception as e:
                self.xbox_controller = Xbox360Controller(index=0, axis_threshold=0.05)

    def _initialize_xbox_controller(self, mode):
        if mode in [self.xbox_mode, self.xbox_direct_mode]:
            self._connect_controller()
            self.xbox_controller.axis_l.when_moved = self._xbox_adjust_steering
            self.xbox_controller.trigger_r.when_moved = self._xbox_control_speed
            self.xbox_controller.trigger_l.when_moved = self._xbox_control_speed
            print('sucessfully connected to controller')
        elif mode == self.xbox_forwarding_mode:
            self._connect_controller()
            print('sucessfully connected to controller.')
            print('Don\'t forget to set forwarding functions using: set_trigger_l_func, set_trigger_r_func,'
                  ' set_l_joystick_func')

    def set_trigger_l_func(self, function):
        """
        Use this function when in xbox_forwarding_mode to assign a function to handle a signal from the left trigger.
        :param function: function that takes one argument that will be an Axis object from the xbox360controller

        * * *
        """
        self.xbox_controller.trigger_l.when_moved = function

    def set_trigger_r_func(self, function):
        """
        Use this function when in xbox_forwarding_mode to assign a function to handle a signal from the right trigger.
        :param function: function that takes one argument that will be an Axis object from the xbox360controller

        * * *
        """
        self.xbox_controller.trigger_r.when_moved = function

    def set_l_joystick_func(self, function):
        """
        Use this function when in xbox_forwarding_mode to assign a function to handle a signal from the left joystick.
        :param function: function that takes one argument that will be an Axis object from the xbox360controller

        * * *
        """
        self.xbox_controller.axis_l.when_moved = function

    def set_r_joystick_func(self, function):
        """
        Use this function when in xbox_forwarding_mode to assign a function to handle a signal from the right joystick.
        :param function: function that takes one argument that will be an Axis object from the xbox360controller

        * * *
        """
        self.xbox_controller.axis_r.when_moved = function

    def _xbox_adjust_steering(self, axis):
        self.set_steering_percent(axis.x)

    def _xbox_control_speed(self, axis):
        trigger_value = axis.value
        if axis.name == "trigger_r":
            speed_percent = trigger_value
        else:
            speed_percent = -trigger_value

        self._xbox_speed_func(speed_percent)

    def set_motor_speed(self, new_speed):
        """ The value set here will be assigned to self.desired_speed. This is only effective in xbox_mode or if the
        internal speed controller is turned on with start_speed_control.
        :param new_speed: new speed in m/s

        * * *
        """
        # if self.rc_control == JetsonEV.VESC_CONTROL:
        #   self.vesc.set_rpm(int(new_speed/(speed_conversion*poles_on_motor)))
        # else:
        self.desired_speed = new_speed

    def set_motor_percent(self, percentage):
        """ This function will directly send a pwm signal to the motor controller. Do not use this while the internal
        speed control is running (i.e. DO NOT use while in xbox_mode or if start_speed_control() has been called).
        :param percentage: motor duty cycle to use between -1 (reverse) and 1 (forward)

        * * *
        """
        if percentage > 1:
            percentage = 1
        elif percentage < -1:
            percentage = -1
        # print(percentage*self.duty_cycle_limit)
        self._set_motor_duty(percentage * self.duty_cycle_limit)

    def set_steering_percent(self, percentage):
        """
        :param percentage: steering servo percentage between -1 and 1

        * * *
        """
        if percentage > 1:
            percentage = 1
        elif percentage < -1:
            percentage = -1
        steering_percent = -(percentage - 1) / 2
        self._set_servo_percent(steering_percent)

    def close_camera(self):
        if self.camera is not None:
            self.camera.wrapped_object.release()

    def _get_imu_values(self):
        self.imu_values = (self._imu_rotation @ self.imu.wrapped_object.accel.xyz,  # + [0, 0, self._gravity_offset],
                           self._imu_rotation @ (self.imu.wrapped_object.gyro.xyz - self._gyro_offset))
        # , self._imu_rotation @ self.imu.wrapped_object.mag.xyz)
        self.imu_socket.send_data({'accel': self.imu_values[0], 'gyro': self.imu_values[1]})
        return self.imu_values

    def get_x(self):
        """
        :returns Latest x position measurement in millimeters calculated from SLAM.

        * * *
        """
        return self.x

    def get_y(self):
        """
        :returns Latest y position measurement in millimeters calculated from SLAM.

        * * *
        """
        return self.y

    def get_yaw(self):
        """
        :returns Latest yaw angle measurement in degrees calculated from SLAM.

        * * *
        """
        return self.theta

    def get_imu_values(self):
        """
        :returns Latest measurement from the IMU.

        * * *
        """
        return self.imu_values

    def get_lidar_values(self):
        """
        :returns: Latest measurement from the Lidar sensor.

        * * *"""
        return self.latest_lidar_scan

    def get_camera_frame(self):
        """
        :returns: Latest camera frame.

        * * *
        """
        return self.latest_frame

    def get_speed(self):
        """
        :returns: Latest speed measurement (m/s)

        * * *"""

    def add_timed_task(self,
                       calling_function,
                       object_to_wrap=None,
                       sampling_rate=10,
                       calling_func_args=[],
                       forwarding_function=None,
                       verbose=False):
        """
        This is a helper function to create a TimedTask object. The only difference between using this and making a
        TimedTask directly, is that the resulting new task is also added to JetsonEV._timed_tasks which are all
        automatically stopped when shutdown() is called.

        :param calling_function: The function that will be called at the set interval.
        :type calling_function: function

        :param object_to_wrap: An instance of an object that is associated with the repetitive task. References to
               this object can be obtained from two member attributes of TimedTask under the name "wrapped_object" and
               the name of the type of the object that is passed (i.e. if an object of type "Sensor" is passed, then the
               reference would be "myTimedTask.Sensor"). This can be set as None if you do not want this feature or are
               not using an object.
        :type object_to_wrap: object

        :param sampling_rate: The frequency in Hz of task loop.
        :type sampling_rate: float

        :param calling_func_args: A list of any arguments that need to be passed to the calling_function when it is
               called.
        :type calling_func_args: list

        :param forwarding_function: An additional optional function that can be specified to be called every loop.
        :type forwarding_function: function

        :param verbose: Whether or not to print errors from the task (i.e. task is not keeping up to desired run speed).

        :returns The created TimedTask object.

        * * *
        """

        new_task = TimedTask(calling_function,
                             object_to_wrap,
                             sampling_rate,
                             calling_func_args,
                             forwarding_function,
                             verbose=verbose)

        self._timed_tasks.append(new_task)
        return new_task

    def start_all_tasks(self):
        """Starts all the created TimedTasks that are being tracked by JetsonEV. This is called in the constructor.
        * * *
        """
        [x.start() for x in self._timed_tasks]


def __shutdown__(signum, frame):
    JetsonEV.__running_car__.shutdown()
    print('Car shutdown.')
    # raise KeyboardInterrupt


signal.signal(signal.SIGTERM, __shutdown__)


def calibrate_imu(bus=0):
    """A function to zero out the gyros and orient the accelerometer to gravity as -Z. Running this function
    overwrites the existing calibration stored in the root folder. The new calibration will persist until this
    function is run again.

    :param bus: I2C Bus to communicate with the imu

    * * *"""
    imu = MPU6050(bus=bus, device_addr=0x68)  # create the imu device
    run_calibration(imu)
