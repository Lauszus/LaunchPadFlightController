# LaunchPad Flight Controller
#### Developed by Kristian Lauszus, 2015

The code is released under the GNU General Public License.
_________

This is a flight controller used for a quadcopter in X-configuration.

It is written for the [Tiva C Series TM4C123G LaunchPad](http://www.ti.com/tool/EK-TM4C123GXL).

In order to built this project you need to download Keil µVision IDE 5. Then open the [project file](LaunchPadFlightController.uvprojx).

More information can be found at the following blog post: <http://blog.tkjelectronics.dk/2015/01/launchpad-flight-controller/>.

# Android application

Android application is available at the following repository: <https://github.com/Lauszus/LaunchPadFlightController>.

# Video demonstrations

Some video demonstrations of the flight controller can be seen at my [YouTube channel](https://www.youtube.com/user/kslauszus).

# GUI

A simple GUI can be found inside the [GUI](GUI) directory. It can be used to visualize the orientation of the flight controller.

# Features

* Store PID values, calibration values etc. in EEPROM
* Gyro & accelerometer calibration routine
* Arm/disarm using rudder
* Status LEDs
* Supports CPPM receivers
* Gyro & accelerometer (MPU-6500)
* Magnetometer (HMC5883L)
* Ultrasound sensor aka sonar (HC-SR04)
* Rate mode, Self level mode
* [Android application](https://github.com/Lauszus/LaunchPadFlightControllerAndroid)
* OneShot125 ESC support
* Buzzer feedback

# Current setup

* Motors: Dualsky 980kv
* ESC's: [Hobby King 20A ESC 3A UBEC (F-20A)](http://hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=37253) - flashed with SimonK bs_nfet with OneShot125 and COMP_PWM = 1
* Props: 10x4.5
* Frame: [Hobby King Q450 V3 Glass Fiber Quadcopter Frame 450mm](http://hobbyking.com/hobbyking/store/__49725__Q450_V3_Glass_Fiber_Quadcopter_Frame_450mm_Integrated_PCB_Version.html)
* LiPo: [Turnigy 3300mAh 3S 30C Lipo Pack](http://hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=35870)
* RX: [OrangeRX R615X](http://www.hobbyking.com/hobbyking/store/__46632__OrangeRx_R615X_Spektrum_JR_DSM2_DSMX_Compatible_6Ch_2_4GHz_Receiver_w_CPPM.html)
* TX: [Turnigy 9XR](http://www.hobbyking.com/hobbyking/store/__31544__Turnigy_9XR_Transmitter_Mode_2_No_Module_.html) flashed with OpenTX
* TX module: [OrangeRX 2.4GHz transmitter module](http://hobbyking.com/hobbyking/store/__24656__OrangeRX_DSMX_DSM2_2_4Ghz_Transmitter_Module_JR_Turnigy_compatible_.html)

# Pinout

| Pin  |    Connection  |
|------|----------------|
| PB6  |     Motor 1    |
| PB7  |     Motor 2    |
| PB4  |     Motor 3    |
| PB5  |     Motor 4    |
| PC6  |    CPPM input  |
| PA6  |      SCL       |
| PA7  |      SDA       |
| PE3  |  MPU-6500 INT  |
| PC4  |   Sonar echo   |
| PC5  |  Sonar trigger |
| PB0* |    UART1 RX    |
| PB1* |    UART1 TX    |
| PA5  |     Buzzer     |

\* Not 5V tolerant

UART1 is connected to a HC-06 Bluetooth module running at a baudrate of 115200.

# Notes

Note that the motor layout follow the Naze32 in x-configuration i.e. motor 1 is bottom right, motor 2 is top right, motor 3 is bottom left and motor 4 is top left when looking from the back.

Make sure that roll increases when tilting quadcopter to the right, pitch increases when pitching quadcopter downward and yaw increases when rotation quadcopter clockwise.

It is a good idea to run the accelerometer, magnetometer and ESCs calibration routines before flying the aircraft.

__WARNING:__ Take propellers OFF when testing and calibrating ESCs!!

For more information send me an email at <kristianl@tkjelectronics.dk>.