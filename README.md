# LaunchPad Flight Controller
#### Developed by Kristian Lauszus, 2015

The code is released under the GNU General Public License.
_________

This is a flight controller used for a quadcopter in X-configuration.

It is written for the [Tiva C Series TM4C123G LaunchPad](http://www.ti.com/tool/EK-TM4C123GXL).

In order to built this project you need to download Keil µVision IDE 4. Then open the [project file](LaunchPadFlightController.uvproj).

More information can be found at the following blog post: <http://blog.tkjelectronics.dk/2015/01/launchpad-flight-controller/>.

# Video demonstrations

Some video demonstrations of the flight controller can be seen at my [YouTube channel](https://www.youtube.com/user/kslauszus).

# Current setup

* Microcontroller: Tiva C Series TM4C123G LaunchPad
* IMU: MPU-6500
* Motors: Sunnysky X2204 2300kv
* ESC's: Blue Series 12A RapidESC (SimonK v2013-05-15)
* Props: Gemfan 5x3
* Frame: ZMR250 FPV Quadcopter (Bought on eBay)
* LiPo: Turnigy nano-tech 1500 mAh
* RX: OrangeRX R615X
* TX: Turnigy 9XR
* TX module: OrangeRX 2.4GHz transmitter module

# Pinout

| Pin |   Connection  |
|-----|---------------|
| PB6 |    Motor 1    |
| PB7 |    Motor 2    |
| PB4 |    Motor 3    |
| PB5 |    Motor 4    |
| PC6 |   CPPM input  |
| PA6 |     SCL       |
| PA7 |     SDA       |
| PE3 |     INT       |
| PC4 |  Sonar echo   |
| PC5 | Sonar trigger |

Note that the motor layout follow the Naze32 in x-configuration i.e. motor 1 is bottom right, motor 2 is top right, motor 3 is bottom left and motor 4 is top left when looking from the back.

Also make sure that pitch increase when pitching quadcopter downward and roll should increase when tilting quadcopter clockwise.

It is a good idea to run the accelerometer and ESCs calibration routines before flying the aircraft.

__WARNING:__ Take propellers OFF when testing and calibrating ESCs!!

For more information send me an email at <kristianl@tkjelectronics.dk>.