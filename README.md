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

## How to setup realtime kernel

Disclaimer: this worked for my setting, it might not work on yours.

First, follow the steps in the [Franka documentation](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel). When deciding which linux kernel version to choose, for Ubuntu 20.04, just choose `v5.11.4`, which is the closest version to my original kernel that I found works. The command to download the source files are

```console
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.11.4.tar.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.11.4.tar.sign
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.11/patch-5.11.4-rt11.patch.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.11/patch-5.11.4-rt11.patch.sign
```

Before running

```console
make -j$(nproc) deb-pkg
```

install `fakeroot` using

```console
sudo apt install fakeroot
```

Then, finish the remaining steps in the [Franka documentation](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel). Make sure to NOT install the `.deb` file that contains `dbg`, this is very IMPORTANT! 

```console
sudo dpkg -i linux-headers-5.11.4-rt11_5.11.4-rt11-1_amd64.deb
sudo dpkg -i linux-image-5.11.4-rt11_5.11.4-rt11-1_amd64.deb
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
