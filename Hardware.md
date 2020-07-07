---
layout: default
---
# Hardware
* [RC Car](#rc-car)
* [Brushless DC Motor](#brushless-dc-motor)
* [Electronic Speed Controller](#electronic-speed-controller)
* [Vedder Speed Controller (VESC)](#vedder-speed-controller-vesc)
* [Steering Servo](#steering-servo)
* [Battery](#battery)
* [Lidar](#lidar)
* [Inertial Measurement Unit (IMU)](#inertial-measurement-unit-imu)

## RC Car
[Link to RC Car Manual]({{"/pdfs/"|relative_url}}Team_C_TD10_EP_DRIFT_ONROAD__TD10__4WD_ROLLING.pdf)

## Brushless DC Motor
The JetsonEV is powered by an Absima brushless DC motor.

### Specs:
- Part No. 2130020
- 13.5 Turns
- RPM-V: 3040
- Power: 255 W
- Current: 35 A

[Link to Datasheet]({{"/pdfs/"|relative_url}}Absima_BRUSHLESS_MOTOR_REVENGE_CTM_13_5T_STOCK.pdf) for more information.

## Electronic Speed Controller
When controlling the JetsonEV using an arduino, this motor controller is used. The controller is driven with a PWM signal on the white wire using standard servo control signals (i.e. pulse between 1000-2000 ms). The red wire PROVIDES 5V. It should only be necessary to connect the white and black (GND) wires. 

[Link to Datasheet]({{"/pdfs/"|relative_url}}BLDC_ESC.pdf) for more information.

## Vedder Speed Controller (VESC)
The VESC is a much more advanced BLDC motor controller. Communication is done through UART using the python library [PyVESC](https://github.com/LiamBindle/PyVESC). The VESC is also used to control the steering servo. More information can be found [here](./VESC.html).

## Steering Servo
This uses a standard rc car servo.

[Link to Datasheet]({{"/pdfs/"|relative_url}}Savox SC0254MG Digital Servo.pdf) for more information.

## Battery
### Specs
- LiPo
- 20 Cells
- 5000 mAh
- 11.1 V
- 55.5 Wh

[Link to Datasheet]({{"/pdfs/"|relative_url}}Hacker_LIPO_AKKU_20C_ECO_X_500_de_en_it.pdf) for more information.

The datasheet for the battery wall charger can be found [here]({{"/pdfs/"|relative_url}}LADEGERAET_V_CHARGE_ECO_LIPO_de_en_fr_nl.pdf).

## Lidar

[Link to Datasheet]({{"/pdfs/"|relative_url}}LD108_SLAMTEC_rplidar_datasheet_A1M8_v1.0_en.pdf) for more information.

## Inertial Measurement Unit (IMU)

[Link to Datasheet]({{"/pdfs/"|relative_url}}PS-MPU-9250A-01-v1.1.pdf) for more information.
