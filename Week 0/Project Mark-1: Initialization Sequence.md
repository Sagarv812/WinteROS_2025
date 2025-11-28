# ROS Installation in Native UBUNTU - No Docker (Proceed at your own risk)

The docker container provides plenty of functionalities all of which will be more than enough for this bootcamp. However, if you wish to pursue ROS further in the future, it is advisable to download it natively on Ubuntu. Follow the given steps to install:

## Ubuntu Installation (Ubuntu 24.04)
> NOTE: Skip this part if you already have **Ubuntu 24.04** installed on your computer.

We will be using ROS2 Jazzy. This is only compatible from Ubuntu 24.04 onwards so please upgrade your Ubuntu if it's an older version.

### Dual boot
Follow this [Tutorial](https://youtu.be/alFosqQ1ang?si=47hL29TceYc_6h89) to install Ubuntu 24.04 on your Windows PC.  
For macOS, follow this [Tutorial](https://youtu.be/jbUulXVZIBI?si=XTMyoI4yP6OC0Jc5).  
 **WARNING**: Dual booting can be quite risky. Especially on **macOS**. Do at your own risk! We will not be responsible if you lose your data. **Follow instructions carefully and make backups before you start!**

 > Note: For absolute beginners, we recommend going for the Docker installation as dual booting can be quite scary. You can always opt for dual-booting once you're comfortable with linux.

 ### Get Familiar with Linux
 Here are a few additional resources that you can refer to familiarize yourselves with Linux:
 
 - [Video Based Tutorial](https://www.youtube.com/watch?v=IVquJh3DXUA)
 - [Text-based Tutorial](https://ryanstutorials.net/linuxtutorial/)
 - [Document containing useful linux commands](https://docs.google.com/document/d/1aroDJBIP-mqYovI8sVYYjGrn_1ugpN5NBauLLihvEjM/edit?usp=sharing)

 ## Clearing out older versions before ROS Installation:
 We wll be using ROS2 Jazzy for the bootcamp, because it has support for the latest packages that we will be using in this bootcamp. So before you install ROS2 Jazzy on your system, make sure you don't have any previous installations of ROS (even faulty installs). You can run the following commands in your Ubuntu to clear it out:  

 - To check which version of ROS is installed 
    ```bash
    dpkg -l | grep ros-*
    ```
- If you get any version other than **Jazzy**, run the following command to remove the installation:
    ```bash
    sudo apt remove ~nros-<your-ros-version>-*
    ```
    For example, if you have ROS2 Humble installed, then run the command:
    ```bash
    sudo apt remove ~nros-humble-*
    ```
- Then run this for cleanup:
    ```bash
    sudo apt autoremove
    ```

## ROS2 Jazzy Installation

This is the [Tutorial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) that we will be following for the installation. Since the official documentation may not be as easy to follow, we will be listing out the proper steps to do here for smooth installation:

### Step 1
Make sure you have a locale which supports `UTF-8`.
```bash
locale # check for UTF-8


sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale # verify settings
```
Enter your password if prompted.

### Step 2
You will need to add the ROS2 apt repository to your system.  
First ensure that the Ubuntu Universe repository is enabled.
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe # Click [ENTER] after running command
```
The ros-apt-source packages provide keys and apt source configuration for the various ROS repositories. Installing the ros2-apt-source package will configure ROS 2 repositories for your system. Updates to repository configuration will occur automatically when new versions of this package are released to the ROS repositories.  
Run the following commands to install the repository onto your system:
```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### Step 3
Update your apt repository caches
```bash
sudo apt update
sudo apt upgrade
```

Installation command:
```bash
sudo apt install ros-jazzy-desktop
```

To enable ROS2 on you terminal, run:
```bash
source /opt/ros/jazzy/setup.bash
```

If you don't want to have to source the setup every time you open a new terminal, then run:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

And VOILA!! You have installed ROS2 Jazzy on your system. Great job!  
Let's test it out with an example

### Example
Open a terminal and run:
```bash
ros2 run demo_nodes_cpp talker
```

In another terminal, run:
```bash
ros2 run demo_nodes_py listener
```

If ROS2 Jazzy has been installed correctly on your machine, then you should see the `talker` saying that it's "Publishing messages" and the `listener` saying "I heard" those messages. This verifies that both the C++ and Python APIs are working properly. Hooray!

