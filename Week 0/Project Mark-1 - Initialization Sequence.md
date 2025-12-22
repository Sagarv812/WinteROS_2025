# Docker Installation and Running ROS Containers  
*Project Mark-1: Initialization Sequence*

Before Tony Stark could weld armor plates or fire up a repulsor, he needed one thing above all:  
**a working cave terminal** — a system capable of running ROS, simulations, sensors, and code.

This guide is that first spark — powering up your own cave setup by installing Docker and preparing the environment where every part of the Mark-1 (Weeks 0–5) will be built.

Once everything is ready, your ROS container will run like this:  
![12secfinal (1)](https://github.com/user-attachments/assets/0893ef22-5425-4e2c-a8d7-759f8319c895)

# ❄️ Week 0 Begins  
Before building intelligence, mobility, vision, or manipulation for the Mark-1, you need the **base system**.

This installation is the **System Boot Protocol** — the foundational step Tony had to take before creating anything else.  
Once your ROS container runs, you’re ready to begin forging the digital nervous system of Stark’s escape machine.

---

## Index  
- [Windows Installation](#for-windows)  
- [macOS Installation](#for-macos)  
- [Ubuntu Installation](#for-ubuntu)  

---


# For Windows  

### Step 1: Install WSL (Windows Subsystem for Linux)  

1. Open **PowerShell** as Administrator.  
2. Run this command:  
   ```bash
   wsl --install
   ```
3. Once the installation is complete, reboot your system.  

> **Note**: If you already have WSL installed, you can skip this step and move to the next one.You can check if WSL is installed in your system by running `wsl --list --verbose` in the powershell. It will show the list of distros installed in WSL , if you have it already or else it will tell command not found. 

### Step 2: Install Docker Desktop  

1. Visit the [Docker installation page for Windows](https://docs.docker.com/desktop/setup/install/windows-install/).  
2. Download the `.exe` file for Docker Desktop.  
3. Run the installer and follow the on-screen instructions to complete the installation.  
4. After installation, reboot your system.  

### Step 3: Running the ROS Container  

1. Open **Docker Desktop** after logging into your system.  
2. In the bottom-right corner, open the **Docker Terminal**.
     
<img width="960" height="574" alt="1" src="https://github.com/user-attachments/assets/303908cb-6d15-41fa-97cf-e5eab3757f2e" />

   
4. Run the following command to start the ROS Docker container:  
   ```bash
   docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m adtyp/winteros_docker:jazzy
   ```
5. Wait for the container to initialize. This may take around 30 minutes for first-time users.  

> **Note**: First launch takes 20–30 minutes as Docker assembles the system — just like Tony’s first cave prototype.

<img width="960" height="564" alt="2" src="https://github.com/user-attachments/assets/6fcfdc7d-d531-46fc-9d55-c6166867d8aa" />


### Step 4: Access ROS in the Browser  

1. Open your web browser.  
2. In the address bar, type `localhost:6080` and press Enter.  
3. You should now see your ROS environment running inside Docker.
4. To stop it , run `contol + C` in the terminal and to run the container again , just toggle the play button in the docker desktop , no need to folow the command line code again :))
   

<img width="960" height="564" alt="3" src="https://github.com/user-attachments/assets/1b882d23-e12b-407e-9f7f-970df12029c5" />


> **Note**: you have to only do this once (pasting the commad in the terminal) from the next time just toggle the play button in the containers section of docker desktop has shown in the above pic 


---

## For macOS  

### Step 1: Install XQuartz — External HUD Renderer 

Tony built his HUD from scraps.
You're installing yours with a DMG file.

1. Download and install XQuartz from [here](https://www.xquartz.org/releases/XQuartz-2.8.1.html).  
2. Follow the on-screen instructions to complete the installation.  

### Step 2: Install Docker Desktop  

1. Visit the [Docker installation page for macOS](https://docs.docker.com/desktop/setup/install/mac-install/).  
2. Download the `.dmg` file for Docker Desktop.  
3. Run the `.dmg` file and follow the on-screen instructions.  

### Step 3: Running the ROS Container  

1. Open **Docker Desktop** after logging into your system.  
2. In the bottom-right corner, open the **Docker Terminal**.  
<img width="1372" alt="SCR-20241129-hdwg" src="https://github.com/user-attachments/assets/5f6891df-5274-4f3f-8b52-e6afcd59663d">

3. Run the following command to start the ROS Docker container:  
   ```bash
      docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m adtyp/winteros_docker:jazzy
   ```
4. Wait for the container to initialize. This may take around 30 minutes for first-time users.  

> **Note**: The first run may take some time to download necessary files and set up the environment.

<img width="1372" alt="image" src="https://github.com/user-attachments/assets/0cac3b54-d77f-4c06-b1e7-a021ba9349b6">



### Step 4: Access ROS in the Browser  (HUD Access)

1. Open your web browser.  
2. In the address bar, type `localhost:6080` and press Enter.  
3. You should now see your ROS environment running inside Docker.
4. To stop it , run `contol + C` in the terminal and to run the container again , just toggle the play button in the docker desktop , no need to folow the command line code again :))
   <img width="1372" alt="image" src="https://github.com/user-attachments/assets/e68e95d1-2dc6-4fc3-807c-c8f91ab67086">

> **Note**: you have to only do this once (pasting the commad in the terminal) from the next time just toggle the play button in the containers section of docker desktop has shown in the above pic 
---

## For Ubuntu  

> **Note**: Ubuntu is the closest thing to Tony’s raw Linux cave setup. If you’re not using GNOME Terminal: 
> ```bash
> sudo apt install gnome-terminal
> ```


### Step 1: Download Docker Desktop  

1. Download the `.deb` file for Ubuntu from the [Docker installation page](https://docs.docker.com/desktop/setup/install/linux/ubuntu/).  

### Step 2: Install Docker Desktop  

1. Open the terminal and enter the following commands:  
   ```bash
   sudo apt-get update
   sudo apt-get install ./docker-desktop-amd64.deb
   ```
2. Reboot your system.  

### Step 3: Running the ROS Container  

1. Open the **Terminal**.  
2. Run the following command to start the ROS Docker container:  
   ```bash
      docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m adtyp/winteros_docker:jazzy
   ```
3. Wait for the container to initialize. This may take around 30 minutes for first-time users.  

> **Note**: The first run may take some time to download necessary files and set up the environment.

<img width="1372" alt="image" src="https://github.com/user-attachments/assets/0cac3b54-d77f-4c06-b1e7-a021ba9349b6">


### Step 4: Access ROS in the Browser  

1. Open your web browser.  
2. In the address bar, type `localhost:6080` and press Enter.  
3. You should now see your ROS environment running inside Docker.
4. To stop it , run `contol + C` in the terminal and to run the container again , just toggle the play button in the docker desktop , no need to folow the command line code again :))
   <img width="1372" alt="image" src="https://github.com/user-attachments/assets/e68e95d1-2dc6-4fc3-807c-c8f91ab67086">

> **Note**: you have to only do this once (pasting the commad in the terminal) from the next time just toggle the play button in the containers section of docker desktop has shown in the above pic 
---




# ROS Installation in Native UBUNTU - No Docker (Proceed at your own risk)

**WARNING — This is pure arc reactor work.**
Raw, exposed, no safety suit.

Recommended only for users who plan to work with ROS long-term.. Follow the given steps to install:

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
sudo apt install ros-dev-tools
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

> If the examples nodes don't run or if you get an error saying `Package 'demo_nodes_cpp' not found` then run the command `sudo apt install ros-jazzy-desktop` again and try re-running the examples.

# ⚡  SYSTEMS ONLINE
<p align="center">
  <img src="https://github.com/user-attachments/assets/9be17ef2-fbf3-4b5d-b540-536e87a4cf18" width="450">
</p>

Your cave terminal is active.
Docker is running.
ROS is online.
This is the moment Tony Stark looked at his glowing reactor and said:

**“Yeah… I can work with this.”**

**You’ve built the furnace. Next week, we forge the mind inside it.**
