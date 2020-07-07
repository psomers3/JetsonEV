import os
import subprocess
import json
from TimedTask import TimedTask
import numpy as np
import threading
import time
import math

calibration_file = os.path.join(os.path.dirname(__file__), 'imu_calibration.txt')
calibration_write_script = os.path.join(os.path.dirname(__file__), 'write_calibration.py')


def get_calibration():
    if os.path.exists(calibration_file):
        with open(calibration_file) as f:
            return json.loads(f.read())
    else:
        print('No calibration file found. Use the run_calibration function to generate one.')
        my_calibration_dict = {'rotation_matrix': np.eye(3).tolist(),
                               'gravity': 0,
                               'gyro_offsets': [0, 0, 0]}
        return my_calibration_dict


def write_calibration_to_file(calibration_dict):
    subprocess.call(['sudo', 'python3', calibration_write_script, calibration_file, json.dumps(calibration_dict)])


def run_calibration(imu_object, calibrate_mag=False, over_write_old_values=True, save_to_file=None):
    # do calibration and make a dictionary of values here
    rate = 50  # hertz
    sample_time = 5  # seconds
    total_samples = rate * sample_time

    stop_recording = threading.Event()

    if calibrate_mag:
        num_sens = 3
    else:
        num_sens = 2

    samples = np.zeros((rate*sample_time, num_sens, 3))
    averages = np.zeros((num_sens, 3))
    i = [0]

    def get_imu_values():
        if calibrate_mag:
            data = (imu_object.accel.xyz, imu_object.gyro.xyz, imu_object.mag.xyz)
            return data
        else:
            data = (imu_object.accel.xyz, imu_object.gyro.xyz)
            return data

    def fill_samples(values):
        if i[0] >= rate * sample_time:
            stop_recording.set()
        else:
            samples[i[0]] = values
            i[0] = i[0] + 1

    task = TimedTask(calling_function=get_imu_values,
                     forwarding_function=fill_samples,
                     sampling_rate=rate)
    print('Starting calibration. Don\'t move the Vehicle.')

    task.start()
    while not stop_recording.isSet():
        time.sleep(0.0001)
    task.stop()

    for i in range(num_sens):
        for axis in range(3):
            averages[i][axis] = np.mean(samples[:, i, axis])

    # normalize accelerations
    offset = np.sqrt(averages[0][0]**2 + averages[0][1]**2 + averages[0][2]**2)
    averages[0] = averages[0] / offset

    gamma = math.atan2(-averages[0][1], -averages[0][2])
    beta = math.atan2(averages[0][0], math.sqrt(averages[0][1]**2 + averages[0][2]**2))
    alpha = 0

    rotation_matrix = np.array([[np.cos(alpha)*np.cos(beta),
                        np.cos(alpha)*np.sin(beta)*np.sin(gamma) - np.sin(alpha)*np.cos(gamma),
                        np.cos(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(alpha)*np.sin(gamma)],
                       [np.sin(alpha)*np.cos(beta),
                        np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma),
                        np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.cos(alpha)*np.sin(gamma)],
                       [-np.sin(beta), np.cos(beta)*np.sin(gamma), np.cos(beta)*np.cos(gamma)]])

    my_calibration_dict = {'rotation_matrix': rotation_matrix.tolist(),
                           'gravity': offset,
                           'gyro_offsets': [averages[1, 0],
                                            averages[1, 1],
                                            averages[1, 2]]}

    #print("accel: {}\ngyro: {}".format(rotation_matrix @ imu_object.accel.xyz + [0, 0, offset],
    #                                    imu_object.gyro.xyz - my_calibration_dict['gyro_offsets']))

    write_calibration_to_file(my_calibration_dict)

