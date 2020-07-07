from JetsonEV import JetsonEV


if __name__ == '__main__':
    Car = JetsonEV(mode=JetsonEV.xbox_direct_mode,
                   rc_communication=1,  # I2C bus for arduino
                   rc_control=JetsonEV.ARDUINO_CONTROL,
                   # rc_communication='USB',
                   # rc_control=JetsonEV.VESC_CONTROL,
                   initialize_imu=True,
                   initialize_lidar=True,
                   initialize_camera=False,
                   max_duty_cycle=0.15,
                   max_speed_limit=3)
    input('Press enter to shut down\n')
    Car.shutdown()
    exit()
