# LaunchPad Flight Controller
#### Developed by Kristian Lauszus, 2015

The code is released under the GNU General Public License.
_________

This is a flight controller used for a quadcopter in X-configuration.

It is written for the [Tiva C Series TM4C123G LaunchPad](http://www.ti.com/tool/EK-TM4C123GXL).

In order to built this project you need to download Keil µVision IDE 4. Then open the [project file](LaunchPadFlightController.uvproj).

More information can be found at the following blog post: <http://blog.tkjelectronics.dk/2015/01/launchpad-flight-controller/>.

# Android application

Android application is available at the following repository: <https://github.com/Lauszus/LaunchPadFlightController>.

# Video demonstrations

Some video demonstrations of the flight controller can be seen at my [YouTube channel](https://www.youtube.com/user/kslauszus).

# Features

* Store PID values, calibration values etc. in EEPROM
* Calibrate gyro and accelerometer
* Arm/disarm using rudder
* Status LEDs
* Ultrasound sensor aka sonar (HC-SR04)
* Supports CPPM receiver
* Gyro & accelerometer (MPU-6500)
* Kalman filtered IMU data
* Self level mode
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

| Pin  |   Connection  |
|------|---------------|
| PB6  |    Motor 1    |
| PB7  |    Motor 2    |
| PB4  |    Motor 3    |
| PB5  |    Motor 4    |
| PC6  |   CPPM input  |
| PA6  |     SCL       |
| PA7  |     SDA       |
| PE3  |     INT       |
| PC4  |  Sonar echo   |
| PC5  | Sonar trigger |
| PB0* |   UART1 RX    |
| PB1* |   UART1 TX    |
| PA5  |    Buzzer     |

\* Not 5V tolerant

Note that the motor layout follow the Naze32 in x-configuration i.e. motor 1 is bottom right, motor 2 is top right, motor 3 is bottom left and motor 4 is top left when looking from the back.

Also make sure that pitch increase when pitching quadcopter downward and roll should increase when tilting quadcopter clockwise.

It is a good idea to run the accelerometer and ESCs calibration routines before flying the aircraft.

__WARNING:__ Take propellers OFF when testing and calibrating ESCs!!

For more information send me an email at <kristianl@tkjelectronics.dk>.