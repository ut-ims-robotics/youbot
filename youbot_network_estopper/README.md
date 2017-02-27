# youbot_network_estopper
A ROS node which sends stop commands to the KUKA youBot when it does not receive a heartbeat

RValner: Modified the heartbeat node to support external stop button. When the button is pressed, a controller sends a "stop" signal over serial port.

NB: You need to have libserial installed: http://libserial.sourceforge.net/
```
$ sudo apt-get install libserial-dev
```
