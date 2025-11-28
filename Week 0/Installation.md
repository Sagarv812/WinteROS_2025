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

