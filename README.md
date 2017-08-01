# CNT-723 driver

This is a module that provides the ROS driver for the CNT-723 rotary encoder.

## Dependencies

ROS modules that this package depends on:

- serial

## Usage

Clone, build, and install using catkin:

```
$ git clone https://github.com/naoki-mizuno/cnt723 ~/ros/catkin_ws/src/cnt723
$ cd ~/ros/catkin_ws
$ catkin_make
$ source devel/setup.zsh
$ roslaunch cnt723 cnt723_node.launch
```

## Parameters

- `port`: the port name where the CNT-723 is connected to
- `baudrate`: the baud rate
- `frequency`: the control loop frequency
- `wheel_diameter`: the diameter of the wheel where the transducer is attached
  to
- `counts_per_rotation`: how many counts there are in one rotation

## Caveats

Program mode is currently not supported.

## Author

Naoki Mizuno

## License

MIT License
