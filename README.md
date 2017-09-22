# CNT-723 driver

ROS driver for the [CNT-723 rotary encoder](http://www.cocores.co.jp/english/cnt723.htm) and [peiseler 1283 RS](http://www.sensores-de-medida.es/uploads/motion_transducer_mt_kfz_engl.pdf).

## Dependencies

ROS modules that this package depends on:

- [serial](http://wiki.ros.org/serial)
- [coms_msgs](https://github.com/tado-aev/coms_msgs)

## Usage

Clone, build, and install using catkin:

```
$ git clone https://github.com/tado-aev/cnt723 ~/ros/catkin_ws/src/cnt723
$ cd ~/ros/catkin_ws
$ catkin_make
$ source devel/setup.zsh
$ roslaunch cnt723 cnt723_node.launch
```

## Parameters

- `port`: the port name where the CNT-723 is connected to
- `baudrate`: the baud rate
- `frequency`: the rate at which to publish the pulse counts

## Published Topics

- `cnt723/count`: raw pulse count from the encoder

## Caveats

Program mode is currently not supported.

## License

MIT License

## Author

Naoki Mizuno
