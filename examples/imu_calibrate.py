from JetsonEV import calibrate_imu, get_calibration


if __name__ == '__main__':
    calibrate_imu()
    print(get_calibration())