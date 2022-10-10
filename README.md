# ROS integration for Franka Emika research robots

[![CI](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml)


See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs

## How to test a controller in gazebo

To test a controller's performance in gazebo, run the command

```console
roslaunch franka_gazebo panda.launch controller:=<name-of-controller>
```

Note that the controller cannot use `franka_hw::FrankaPoseCartesianInterface` and `franka_hw::FrankaVelocityCartesianInterface` because `franka_hw` does not support them.
