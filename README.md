# ROS integration for Franka Emika research robots

[![CI](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros/actions/workflows/ci.yml)


See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs

## How to use conda installed packages

First activate the conda environment, then when running `catkin_make`, either add a flag `-DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX` or add

```make
set(CMAKE_INSTALL_PREFIX $ENV{CONDA_PREFIX})
```

to your `CMakeLists.txt` file. Remember to link the libraries!

## Files that need to changed if adding another controller

In the `franka_example_controller` package, the following files need to be created

```
include/franka_example_controllers/new_controller.h
launch/new_controller.launch
src/new_controller.cpp
```

the content of the following files need to be altered

```
config/franka_example_controllers.yaml
CMakeLists.txt
franka_example_controllers_plugin.xml
```

To enable simulation of the new controller, in `franka_gazebo`, the content in the file

```
config/sim_controllers.yaml
```

needs to be altered.

## How to test a controller in gazebo

To test a controller's performance in gazebo, run the command

```console
roslaunch franka_gazebo panda.launch controller:=<name-of-controller>
```

Note that the controller cannot use `franka_hw::FrankaPoseCartesianInterface` and `franka_hw::FrankaVelocityCartesianInterface` because `franka_hw` does not support them.

## How to setup realtime kernel

Disclaimer: this worked for my setting, it might not work on yours.

First, follow the steps in the [Franka documentation](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel). When deciding which linux kernel version to choose, for Ubuntu 20.04, just choose `v5.13`, which is the closest version to my original kernel. One issue when installing the realtime kernel is that directly install `v5.13` will not work in an OEM kernel, which is the default kernel Dell Precision 3650 Tower ships with. One way to deal with this issue is to first install the realtime kernel `v5.9.1` (this has been tested to work on the OEM kernel), then install realtime kernel `v5.13` after booting into realtime kernel `v5.9.1`. The command to download the source files for realtime kernel `v5.9.1` are 

```console
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.sign
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.sign
```

The command to download the source files for realtime kernel `v5.13` are 

```console
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.13.tar.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.13.tar.sign
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.13/patch-5.13-rt1.patch.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.13/patch-5.13-rt1.patch.sign
```

The reason realtime kernel `v5.13` is preferred over `v5.9.1` is that the WiFi driver for Intel AX100 only works for realtime kernels after `v5.13`. Before running

```console
make -j$(nproc) deb-pkg
```

install `fakeroot` using

```console
sudo apt install fakeroot
```

Then, finish the remaining steps in the [Franka documentation](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel). Make sure to NOT install the `.deb` file that contains `dbg`, this is very IMPORTANT! 

```console
sudo dpkg -i linux-headers-5.13.0-rt1_5.13.0-rt1-1_amd64.deb
sudo dpkg -i linux-image-5.13.0-rt1_5.13.0-rt1-1_amd64.deb
```

To ensure the GRUB menu appears when starting the computer, open `/etc/default/grub` using

```console
sudo nano /etc/default/grub
```

and set

```config
GRUB_TIMEOUT_STYLE=menu
GRUB_TIMEOUT=10
```

save the file, and apply the new settings using

```console
sudo update-grub
```

Finally, reboot the machine, go to `UEFI firmware settings > Boot Configuration > Enable Secure Boot` and disable secure boot. Now, you should be able to choose the realtime kernel after selecting `Advanced options for Ubuntu`.
