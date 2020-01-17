
# UDP Client with BNO055

The application creates UDP socket, gets data from BNO055 sensor from predefined I2C port and sends message to the predefined port and IP address in frequency of 100Hz. 

## Hardware Required

This example can be run on any commonly available ESP32 development board with BNO055 sensor attached to I2C port.

## Configure the project

```
make menuconfig
```

Set following parameter under Serial Flasher Options:

* Set `Default serial port`.

Set following parameters under Example Configuration Options:

* Set `WiFi SSID` of the Router (Access-Point).

* Set `WiFi Password` of the Router (Access-Point).

* Set `IP version` of example to be IPV4 or IPV6.

* Set `IPV4 Address` in case your chose IP version IPV4 above.

* Set `IPV6 Address` in case your chose IP version IPV6 above.

* Set `Port` number that represents remote port the example will send data and receive data from.

* Set `I2C Port` number (`Port 0` or `Port 1`).

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
make -j4 flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

## Data description

BNO055 sensor outputs its frame represented by quaternions (stored in `bno055_quaternion_t` structure). For more details see www.github.com/rebrik/esp32-bno055.

Data is read each 10ms by default and sent as a string in UDP packet. For more details about UDP see https://erg.abdn.ac.uk/users/gorry/course/inet-pages/udp.html 


## Troubleshooting

Start server first, to receive data sent from the client (application).
