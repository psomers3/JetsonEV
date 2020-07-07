import sys

if __name__ == '__main__':
    calibration_file = sys.argv[1]
    json_dict = sys.argv[2]

    with open(calibration_file, 'w') as f:
        f.write(json_dict)
