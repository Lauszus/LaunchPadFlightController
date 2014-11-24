# LaunchPad Flight Controller
#### Developed by Kristian Lauszus, 2014

The code is released under the GNU General Public License.
_________

This is a flight controller used for a quadcopter in X-configuration.

It is written for the [Tiva C Series TM4C123G LaunchPad](http://www.ti.com/tool/EK-TM4C123GXL).

In order to built this project you need to download Keil µVision IDE 4. Then open the [project file](LaunchPadFlightController.uvproj).

The code is still in progress. I will update this readme with more information later on.

# Pinout

| Pin | Connection |
|-----|------------|
| PB6 |   Motor 1  |
| PB7 |   Motor 2  |
| PB4 |   Motor 3  |
| PB5 |   Motor 4  |
| PC6 | CPPM input |
| PA6 |     SCL    |
| PA7 |     SDA    |
| PE3 |     INT    |

Note that the motor layout follow the Naze32 in x-configuration i.e. motor 1 is bottom right, motor 2 is top right, motor 3 is bottom left and motor 4 is top left when looking from the back.

Also make sure that pitch increase when pitching quadcopter downward and roll should increase when tilting quadcopter clockwise.

For more information send me an email at <kristianl@tkjelectronics.dk>.