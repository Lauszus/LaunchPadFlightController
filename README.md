# LaunchPad Flight Controller
#### Developed by Kristian Sloth Lauszus, 2015-2020
_________
[![Build Status](https://travis-ci.com/Lauszus/LaunchPadFlightController.svg?branch=master)](https://travis-ci.com/github/Lauszus/LaunchPadFlightController)

This is a flight controller used for a quadcopter in X-configuration.

It is written for the [Tiva C Series TM4C123G LaunchPad](http://www.ti.com/tool/EK-TM4C123GXL) running at 80 MHz.

More information can be found at the following blog posts: <http://blog.tkjelectronics.dk/2015/01/launchpad-flight-controller> and <http://blog.tkjelectronics.dk/2015/08/bachelors-thesis-launchpad-flight-controller>.

# Video demonstrations

Some video demonstrations of the flight controller can be seen at my [YouTube channel](https://www.youtube.com/playlist?list=PLRBI0ZWd8RfBnD1IZzrBdREjrzRAjWMqg).

<a href="https://www.youtube.com/watch?v=pgcV-pZrptI&list=PLRBI0ZWd8RfBnD1IZzrBdREjrzRAjWMqg" target="_blank"><img src="http://img.youtube.com/vi/pgcV-pZrptI/0.jpg" width="400" height="300" border="10" /></a>

# Report

The report I wrote for my Bachelor's thesis can be found in the [docs](docs) folder. The 3D model and Matlab code can be found in there as well.

# Features

* Rate mode, self level mode, heading hold and altitude hold
    - __AUX1:__ Use 3-POS switch for self level and heading hold. At first position both are off, at second position self level is on and at third position both are on
    - __AUX2:__ Use a 3-POS switch for altitude hold. Note that self level mode must be activated for altitude hold to work! At first position altitude hold is turned off, at second position altitude hold will use the distance measured using the sonar and/or LIDAR-Lite v3 and at the third position altitude hold will be using the altitude estimated using the barometer and accelerometer
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
* [LIDAR-Lite v3](https://buy.garmin.com/en-US/US/p/557294)
    - Connect a 680 µF electrolytic capacitor from 5V to GND
* Optical flow sensor (ADNS3080)
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

| Pin  |        Connection       |   Hardware peripheral   |
|------|-------------------------|-------------------------|
| PA0  |         UART RX         |     U0RX (UART0 RX)     |
| PA1  |         UART TX         |     U0TX (UART0 TX)     |
| PA2  |         SPI CLK         |         SSI0CLK         |
| PA3  |         SPI SS          |         SSI0Fss         |
| PA4  |         SPI MISO        |         SSI0Rx          |
| PA5  |         SPI MOSI        |         SSI0Tx          |
| PA6  |           SCL           |         I2C1SCL         |
| PA7  |           SDA           |         I2C1SDA         |
| PB0* |       Bluetooth RX      |     U1RX (UART1 RX)     |
| PB1* |       Bluetooth TX      |     U1TX (UART1 TX)     |
| PB4  |         Motor 3         |         M0PWM2          |
| PB5  |         Motor 4         |         M0PWM3          |
| PB6  |         Motor 1         |         M0PWM0          |
| PB7  |         Motor 2         |         M0PWM1          |
| PC5  |        Sonar echo       |   WTimer0B (WT0CCP1)    |
| PC6  |        CPPM input       |   WTimer1A (WT1CCP0)    |
| PD2  |          Buzzer         |                         |
| PE0  |       Sonar trigger     |                         |
| PE1  |      ADNS3080 reset     |                         |
| PE2  |  MPU-6500/MPU-9250 INT  |                         |
| PE3  |       HMC5883L DRDY     |                         |
| PF0  |         Switch 1        |                         |
| PF1  |         Red LED         |                         |
| PF2  |        Blue LED         |                         |
| PF3  |        Green LED        |                         |
| PF4  |         Switch 2        |                         |

\* UART1 is connected to an HC-06 Bluetooth module running at a baudrate of 115200. __Not 5V tolerant!__, so make sure your Bluetooth module outputs 3.3 voltage level or use a logic level converter.

Furthermore WTimer1B is used to turn off motors if the connection to the RX is lost. SysTick counter is used for time basic time keeping functionality.

The MPU-6500/MPU-9250, HMC5883L, BMP180 and LIDAR-Lite v3 are connected via I<sup>2</sup>C if they are used.

All pins are defined in [src/Config.h](src/Config.h) and can be overriden by creating a file called "Config_custom.h" in the [src](src) directory allowing you to redefine the values.

# Notes

The motor layout follows the Naze32 in x-configuration i.e. motor 1 is bottom right, motor 2 is top right, motor 3 is bottom left and motor 4 is top left when looking from the back. Motor 1 and 4 should be spinning clockwise and motor 2 and 3 should be spinning counterclockwise.

Make sure that roll increases when tilting quadcopter to the right, pitch increases when pitching quadcopter upward and yaw increases when rotation quadcopter clockwise. This can be displayed using the graph menu in the [Android application](https://github.com/Lauszus/LaunchPadFlightControllerAndroid).

The flight controller is armed by having the throttle low and the rudder to the right. The flight controller is disarmed again by having the throttle low and the rudder to the left.

## Initial setup

1. Locate the line ```#define ONESHOT125 1``` in [PPM.c](src/PPM.c) and set line to 0 if your ESCs does not support OneShot125, if you are in doubt, then set the value to 0.
2. Configure the orientation of the MPU-9250/6500 and HMC5883L, so they corresponds to your setup. This is done inside ```mpu6500BoardOrientation``` and ```hmc5883lBoardOrientation``` in [MPU6500.c](src/MPU6500.c) and [HMC5883L.c](src/HMC5883L.c) respectively.
    * The x-axis should be facing forward, the y-axis should be facing to the right and the z-axis should be facing downward. Typically the axis is indicated on the breakout board for the sensor.

### Calibrating the ESCs

__WARNING:__ Take propellers OFF when testing and calibrating ESCs!!

In order calibrate the ESCs you need to do the following procedure:

1. Power up the board while holding down both hardware switches. The blue LED will come on, indicating that the ESCs will be calibrated at next power up.
2. Turn the board off by disconnecting the battery.
3. Now apply power while holding down the two hardware switches. The board will send out the maximum and minimum pulses to the ESCs and thus calibrating them. When the ESC calibration is done the blue LED will be turned off and a long beep will be played.

The calibration is canceled if the buttons are not held down during the whole procedure or the reset is not performed by a power on reset. The former allows the user to cancel the calibration at any time by releasing the switches and the latter prevents the user from accidentally resetting the board without also resetting the ESCs, for instance by pressing the reset button on the board.

### Calibrating accelerometer and magnetometer

The last step is to calibrate the accelerometer and magnetometer. The accelerometer is calibrated by putting the quadcopter horizontal and activating the calibration routine via the [Android application's](https://github.com/Lauszus/LaunchPadFlightControllerAndroid) settings menu. It is important that the quadcopter is kept still while running the accelerometer calibration routine. When the calibration is done a long beep will be played using the onboard buzzer.

The magnetometer calibration routine is activated similar to the accelerometer by using the [Android application](https://github.com/Lauszus/LaunchPadFlightControllerAndroid). The blue LED will turn on, indicating that the calibration procedure is activated. Now rotate the quadcopter along all three axis slowly in order to find the minimum and maximum values. The user has 30 seconds to rotate the quadcopter. Once the calibration is done the blue LED will be turned off and a long beep will be played.

It is a good idea to confirm that the estimated angles are all correct by using the graph menu in the [Android application](https://github.com/Lauszus/LaunchPadFlightControllerAndroid).

There is no need to calibrate the gyroscope, as this is done at every startup.

### RX

The minimum and maximum receiver values are hardcoded in [src/Config.h](src/Config.h) and might need to be adjusted as well.

# Android application

<a href="http://play.google.com/store/apps/details?id=com.lauszus.launchpadflightcontrollerandroid.app"><img src="https://play.google.com/intl/en_us/badges/images/generic/en_badge_web_generic.png" alt="Google Play" width="200px"/></a>

The Android source code is available at the following repository: <https://github.com/Lauszus/LaunchPadFlightControllerAndroid>.

<a href="https://github.com/Lauszus/LaunchPadFlightControllerAndroid"><img src="https://raw.githubusercontent.com/Lauszus/LaunchPadFlightControllerAndroid/master/android_screenshots.png" width=600/></a>

# GUI

A simple GUI can be found inside the [GUI](GUI) directory. It can be used to visualise the orientation of the flight controller.

# Build instructions

## PlatformIO

[PlatformIO](http://platformio.org) is a cross-platform build system, which makes it easy to compile and upload the code on both Windows, Linux and Mac. This is the recommended way to compile the code for novice users.

Please follow these instructions in order to install Python 2.7 and PlatformIO: <http://docs.platformio.org/en/latest/installation.html>.

On Windows be sure to select "Add python.exe to Path" when installing Python 2.7.

Open a terminal (search for ```cmd.exe``` on Windows) and navigate to the root of the project:

```bash
cd LaunchPadFlightController
```

Now compile the code:

```bash
platformio run
```

The program should now be compiled.

In order to upload the code simply type:

```bash
platformio run -t upload
```

I recommend installing the [PlatformIO IDE](http://platformio.org/get-started/ide?install) if using PlatformIO.

## Keil µVision IDE 5

In order to built this project you need to download Keil µVision IDE 5 and then simply open the [project file](LaunchPadFlightController.uvprojx).

## Manual installation

In order to built this project you will need to use [Make](http://www.gnu.org/software/make/).

First download and install [gcc-arm-none-eabi](https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q3-update) and then install [lm4tools](https://github.com/utzig/lm4tools).

lm4tools can be installed like so:

```bash
$ git clone https://github.com/utzig/lm4tools.git
$ make -C lm4tools/lm4flash all
$ sudo cp lm4tools/lm4flash/lm4flash /usr/bin/
```

If you are on a Mac, I recommend installing gcc-arm-none-eabi using [Homebrew](http://brew.sh) like so:

```bash
$ brew tap PX4/homebrew-px4
$ brew update
$ brew install gcc-arm-none-eabi
```

Now simply compile the code by navigating to the [src](src) and running the following command:

```bash
make
```

And to upload the code:

```bash
make flash
```

Some information on configuring Eclipse for use with this project can be the [Wiki](https://github.com/Lauszus/LaunchPadFlightController/wiki).

### Hardware debugging using OpenOCD

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
