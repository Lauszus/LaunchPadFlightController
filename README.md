# LaunchPad Flight Controller
#### Developed by Kristian Sloth Lauszus, 2015
_________
[![Build Status](https://travis-ci.org/Lauszus/LaunchPadFlightController.svg)](https://travis-ci.org/Lauszus/LaunchPadFlightController)

This is a flight controller used for a quadcopter in X-configuration.

It is written for the [Tiva C Series TM4C123G LaunchPad](http://www.ti.com/tool/EK-TM4C123GXL) running at 80 MHz.

More information can be found at the following blog posts: <http://blog.tkjelectronics.dk/2015/01/launchpad-flight-controller> and <http://blog.tkjelectronics.dk/2015/08/bachelors-thesis-launchpad-flight-controller>.

# Video demonstrations

Some video demonstrations of the flight controller can be seen at my [YouTube channel](https://www.youtube.com/playlist?list=PLRBI0ZWd8RfBnD1IZzrBdREjrzRAjWMqg).

<a href="https://www.youtube.com/watch?v=HXX-2L1hKgI&index=1&list=PLRBI0ZWd8RfBnD1IZzrBdREjrzRAjWMqg" target="_blank"><img src="http://img.youtube.com/vi/HXX-2L1hKgI/0.jpg" width="240" height="180" border="10" /></a>

# Report

The report I wrote for my Bachelor's these can be found in the [docs](docs) folder. The 3D model and Matlab code can be found in there as well.

# Features

* Rate mode, self level mode, heading hold and altitude hold
    - __AUX1:__ Use 3-POS switch for self level and heading hold. At first position both are off, at second position self level is on and at third position both are on
    - __AUX2:__ Use a 2-POS switch for altitude hold. Activated when switch is high. Note that self level mode must be activated for altitude hold to work
* Store PID values, calibration values etc. in EEPROM
* Gyro, accelerometer & magnetometer calibration routine
    - Gyro is calibrated at startup
    - Accelerometer and magnetometer calibration routine can be activated in the code or by using the Android app
    - The magnetometer turns on the Blue LED while calibrating
        + Rotate flight controller slowly along all three axis
* Arm/disarm using rudder
* Status LEDs
* Supports CPPM receivers
* Gyro & accelerometer (MPU-6500 or MPU-9250)
* Magnetometer (HMC5883L or AK8963 (inside MPU-9250))
* Barometer (BMP180)
* Ultrasound sensor aka sonar (HC-SR04)
* [Android application](https://github.com/Lauszus/LaunchPadFlightControllerAndroid)
* OneShot125 ESC support
* Buzzer feedback

# Current setup

* Motors: Dualsky 980kv
* ESC's: [Hobby King 20A ESC 3A UBEC (F-20A)](http://hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=37253) - flashed with SimonK bs_nfet with OneShot125 and COMP_PWM = 1
* Propellers: 10x4.5
* Frame: [Hobby King Q450 V3 Glass Fiber Quadcopter Frame 450mm](http://hobbyking.com/hobbyking/store/__49725__Q450_V3_Glass_Fiber_Quadcopter_Frame_450mm_Integrated_PCB_Version.html)
* LiPo: [Turnigy 3300mAh 3S 30C Lipo Pack](http://hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=35870)
* RX: [OrangeRX R615X](http://www.hobbyking.com/hobbyking/store/__46632__OrangeRx_R615X_Spektrum_JR_DSM2_DSMX_Compatible_6Ch_2_4GHz_Receiver_w_CPPM.html)
* TX: [Turnigy 9XR](http://www.hobbyking.com/hobbyking/store/__31544__Turnigy_9XR_Transmitter_Mode_2_No_Module_.html) flashed with OpenTX
* TX module: [OrangeRX 2.4GHz transmitter module](http://hobbyking.com/hobbyking/store/__24656__OrangeRX_DSMX_DSM2_2_4Ghz_Transmitter_Module_JR_Turnigy_compatible_.html)

# Pinout

| Pin  |         Connection      |
|------|-------------------------|
| PB6  |          Motor 1        |
| PB7  |          Motor 2        |
| PB4  |          Motor 3        |
| PB5  |          Motor 4        |
| PC6  |         CPPM input      |
| PA6  |           SCL           |
| PA7  |           SDA           |
| PE2  |  MPU-6500/MPU-9250 INT  |
| PC5  |        Sonar echo       |
| PE0  |       Sonar trigger     |
| PB0* |         UART1 RX        |
| PB1* |         UART1 TX        |
| PD2  |          Buzzer         |
| PE3  |       HMC5883L DRDY     |

\* UART1 is connected to a HC-06 Bluetooth module running at a baudrate of 115200. __Not 5V tolerant!__, so make sure your Bluetooth module outputs 3.3 voltage level or use logic level converter.

# Notes

Note that the motor layout follows the Naze32 in x-configuration i.e. motor 1 is bottom right, motor 2 is top right, motor 3 is bottom left and motor 4 is top left when looking from the back.

Make sure that roll increases when tilting quadcopter to the right, pitch increases when pitching quadcopter downward and yaw increases when rotation quadcopter clockwise.

It is a good idea to run the accelerometer, magnetometer and ESCs calibration routines before flying the aircraft.

__WARNING:__ Take propellers OFF when testing and calibrating ESCs!!

# Android application

Android application is available at the following repository: <https://github.com/Lauszus/LaunchPadFlightControllerAndroid>.

[![Screenshots](https://raw.githubusercontent.com/Lauszus/LaunchPadFlightControllerAndroid/master/android_screenshots.png)](https://github.com/Lauszus/LaunchPadFlightControllerAndroid)

# GUI

A simple GUI can be found inside the [GUI](GUI) directory. It can be used to visualize the orientation of the flight controller.

# Build instructions

In order to built this project you need to download Keil µVision IDE 5 or use [Make](http://www.gnu.org/software/make/).

If you are using Keil µVision IDE 5, then simply open the [project file](LaunchPadFlightController.uvprojx).

If you are using Make, then you will need to first download and install [gcc-arm-none-eabi](https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q3-update) and then install [lm4tools](https://github.com/utzig/lm4tools).

lm4tools can be installed like so:

```bash
$ git clone https://github.com/utzig/lm4tools.git
$ cd lm4tools/
$ cd lm4flash/ && make
$ sudo cp lm4flash /usr/bin/
```

If you are on a Mac, I recommend installing gcc-arm-none-eabi using [Homebrew](http://brew.sh) like so:

```bash
$ brew tap PX4/homebrew-px4
$ brew update
$ brew install gcc-arm-none-eabi
```

## Hardware debugging using OpenOCD

OpenOCD can be installed via [Homebrew](http://brew.sh) as well:

```bash
$ brew install openocd
```

Now run the following commands:

```bash
$ openocd --file /usr/local/share/openocd/scripts/board/ek-tm4c123gxl.cfg

$ arm-none-eabi-gdb gcc/LaunchPadFlightController.axf
(gdb) target extended-remote :3333
(gdb) monitor reset halt
(gdb) load
(gdb) monitor reset init
```

More information regarding hardware debugging can be found at the [Wiki](https://github.com/Lauszus/LaunchPadFlightController/wiki/Hardware-Debugging-In-Eclipse).

# Credits

A lot of the inspiration for this code came from [Cleanflight](https://github.com/cleanflight/cleanflight) and other open source flight controller projects.

For more information send me an email at <lauszus@gmail.com>.
