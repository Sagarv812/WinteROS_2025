# Image Processing with OpenCV

We'll learn how to implement our own node for image processing using ROS and OpenCV. To start, let's create a package alongside our pre-existing package `erc_gazebo_sensors` that we made in the previous week:

## Create new package
Ensure you have finished `erc_gazebo_sensors` and all its content from **Week 2**. 
Let's now create a new package `erc_gazebo_sensors_py` to run our python scripts from:

```bash
cd ~/erc_ws/src

ros2 pkg create --build-type=ament_python erc_gazebo_sensors_py
```

This will create the package inside your `src` folder.

## Creating new node
Open your workspace in VS Code/VS Codium and navigate to the `erc_gazebo_sensors_py` folder.

The directory will mostly look like this:
```bash
erc_gazebo_sensors_py/
├── erc_gazebo_sensors_py
│   └── __init__.py
├── package.xml
├── resource
│   └── erc_gazebo_sensors_py
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

In this, navigate to the inner folder `erc_gazebo_sensors_py` which has the file `__init__.py`. 

Create a new file of the name `chase_the_ball.py`. Yes, this will make our robot chase a red ball around. Exciting, isn't it!!!

Let's do this step by step

## Step 1
Making the node which subscribes to `/camera/image` topic as that contains the camera feed. It then converts it to OpenCV compatible frame and displays it using OpenCV.

Paste this in `chase_the_ball.py`

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import threading

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Create a subscriber with a queue size of 1 to only keep the last frame
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1  # Queue size of 1
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Variable to store the latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()  # Lock to ensure thread safety
        
        # Flag to control the display loop
        self.running = True

        # Start a separate thread for spinning (to ensure image_callback keeps receiving new frames)
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    def spin_thread_func(self):
        """Separate thread function for rclpy spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        with self.frame_lock:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()

    def display_image(self):
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                self.process_image(self.latest_frame)

                # Show the latest frame
                cv2.imshow("frame", self.latest_frame)
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False

    def process_image(self, img):
        """Image processing task."""
        return

def main(args=None):

    print("OpenCV version: %s" % cv2.__version__)

    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
> In OpenCV, each frame is handled individually and we have to perform operations such as converting it to a CV2 compatible frame for all the frames we get.

Run 
```bash
chmod +x chase_the_ball.py
```
So we can make it an executable file.

Don't forget to add the entry point in `setup.py`
```python
entry_points={
     'console_scripts': [
         'chase_the_ball = erc_gazebo_sensors_py.chase_the_ball:main'
     ],
 },
```

Build and source your workspace.

Run
```bash
ros2 launch erc_gazebo_sensors spawn_robot.launch.py
```
In another terminal sourcing your workspace run
```bash
ros2 run erc_gazebo_sensors_py chase_the_ball
```

You can run this after launching your robot as you did in the previous week and you should be able to see a camera feed. 
> To end the `chase_the_ball` program, just clicking **X** on the window isn't enough. Close it with `Ctrl + C` in the terminal you opened it in.

Don't worry too much about the threads part of the code. It just ensures that the image processing happens on a separate thread as it is quite an intensive task.

## Step 2

As you may have noticed, the `process_image` function doesn't really do anything right now. Let's fix that

Change your `process_image` function to the one given below:

```python
def process_image(self, img):
    """Image processing task."""
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    rows,cols = img.shape[:2]

    R,G,B = self.convert2rgb(img)

    redMask = self.threshold_binary(R, (220, 255))
    stackedMask = np.dstack((redMask, redMask, redMask))
    contourMask = stackedMask.copy()
    crosshairMask = stackedMask.copy()

    # return value of findContours depends on OpenCV version
    (contours, hierarchy) = cv2.findContours(redMask.copy(), 1, cv2.CHAIN_APPROX_NONE)

    # Find the biggest contour (if detected)
    if len(contours) > 0:
        
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        # Make sure that "m00" won't cause ZeroDivisionError: float division by zero
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = 0, 0

        # Show contour and centroid
        cv2.drawContours(contourMask, contours, -1, (0,255,0), 10)
        cv2.circle(contourMask, (cx, cy), 5, (0, 255, 0), -1)

        # Show crosshair and difference from middle point
        cv2.line(crosshairMask,(cx,0),(cx,rows),(0,0,255),10)
        cv2.line(crosshairMask,(0,cy),(cols,cy),(0,0,255),10)
        cv2.line(crosshairMask,(int(cols/2),0),(int(cols/2),rows),(255,0,0),10)

    # Return processed frames
    return redMask, contourMask, crosshairMask
```

Don't run it quite yet though as we have used some helper functions in the above code that we need to define

#### `convert2RGB` and `threshold_binary`:

Write this function above the `process_image` function.
> Please don't write it inside the process image function. They are two fully separate functions which have no overlap.

```python
def convert2rgb(self, img):
    R = img[:, :, 2]
    G = img[:, :, 1]
    B = img[:, :, 0]

    return R, G, B

def threshold_binary(self, img, thresh=(200, 255)):
    binary = np.zeros_like(img)
    binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

    return binary*255
```

## How does it work?
The essence behind OpenCV is that it dissects each frame of the video into three different photo channels - **Red, Blue and Green**.  
We then perform operations on each of the channels separately using the intensity values of the different colours.

### Change your `display_image` function to show all the different frames that `process_image` returns

```python
def display_image(self):
    """Main loop to process and display the latest frame."""
    # Create a single OpenCV window
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("frame", 800,600)

    while rclpy.ok():
        # Check if there is a new frame available
        if self.latest_frame is not None:

            # Process the current image
            mask, contour, crosshair = self.process_image(self.latest_frame)

            # Show the latest frame
            cv2.imshow("frame", self.latest_frame)
            cv2.imshow("mask", mask)
            cv2.imshow("contour", contour)
            cv2.imshow("crosshair", crosshair)
            self.latest_frame = None  # Clear the frame after displaying

        # Check for quit key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False
            break

    # Close OpenCV window after quitting
    cv2.destroyAllWindows()
    self.running = False
```

Okay let's run it once

This time the node will open 4 OpenCV windows and try to find the red ball on the image. Let's add a red ball to the simulation first using the `Resource Spawner` plugin of Gazebo:

<img width="2558" height="1336" alt="image" src="https://github.com/user-attachments/assets/2056ae45-2a0c-4ef5-95d9-99af93cb8815" />

The 4 windows

<img width="2130" height="1156" alt="image" src="https://github.com/user-attachments/assets/0f6d103e-5d61-4bd5-a468-3bf1df941ccd" />

Handling many OpenCV windows can be uncomfortable, so before we start following the ball, let's overlay the output of the image processing on the camera frame:

```python
    # Add small images to the top row of the main image
    def add_small_pictures(self, img, small_images, size=(160, 120)):
    
        x_base_offset = 40
        y_base_offset = 10
    
        x_offset = x_base_offset
        y_offset = y_base_offset
    
        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))
    
            img[y_offset: y_offset + size[1], x_offset: x_offset + size[0]] = small
    
            x_offset += size[0] + x_base_offset
    
        return img
```
Let's modify `display_image()` function so we can use the above function

```python
    def display_image(self):
        """Main loop to process and display the latest frame."""
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                mask, contour, crosshair = self.process_image(self.latest_frame)

                # Add processed images as small images on top of main image
                result = self.add_small_pictures(self.latest_frame, [mask, contour, crosshair])

                # Show the latest frame
                cv2.imshow("frame", result)
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False
```

Now, on running it, we can see that all images are in one window, making it much easier to handle.

<img width="801" height="635" alt="image" src="https://github.com/user-attachments/assets/f3dbe9e2-b18c-49c1-bd9a-7e1d971f6ee5" />

Wait but all of this is just to recognize the ball. Where is the code to follow the ball ? 

Add this code snippet in `proccess_image()` function right after creating the crosshair image

```python
            cv2.line(crosshairMask, (cx, 0), (cx, rows), (0, 0, 255), 10)
            cv2.line(crosshairMask, (0, cy), (cols, cy), (0, 0, 255), 10)
            cv2.line(
                crosshairMask,
                (int(cols / 2), 0),
                (int(cols / 2), rows),
                (255, 0, 0),
                10,
            )
            # Chase the ball
            if abs(cols / 2 - cx) > 20:
                msg.linear.x = 0.0
                if cols / 2 > cx:
                    msg.angular.z = 0.2
                else:
                    msg.angular.z = -0.2

            else:
                msg.linear.x = 0.2
                msg.angular.z = 0.0

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
```

The second else is for the  
```python
    if len(countors) > 0:
```

And now the robot should be able to follow the red ball!!
<img width="1281" height="720" alt="image" src="https://github.com/user-attachments/assets/3480e70c-a69b-45de-9c80-23c71bc62e57" />


# Autonomous Navigation

In this lesson we'll learn how to map the robot's environment, how to do localization on an existing map and we'll learn to use ROS2's navigation stack.

## Download ROS Package
To download the package, go to `Downloads` and clone the repo with the following command:
```bash
git clone https://github.com/Sagarv812/winteros_week3
```

This will make a folder named `winteros_week3` in your Downloads. In that folder there will be three more folders named:
- `erc_ros2_navigation`
- `erc_ros2_navigation_py`
- `erc_trajectory_server`

Copy all these folders and paste them in `~/erc_ws/src`, i.e., inside the `src` folder of you ROS2 workspace. 

Now **build** and **source** your workspace.

You can now test it by running:
```bash
ros2 launch erc_ros2_navigation spawn_robot.launch.py
```

## Mapping
Let's learn how to create the map of the robot's surrounding. In practice we are using SLAM algorithms, SLAM stands for Simultaneous Localization and Mapping. It is a fundamental technique in robotics (and other fields) that allows a robot to:

1. Build a map of an unknown environment (mapping).
2. Track its own pose (position and orientation) within that map at the same time (localization).

Usually SLAM algorithms consists of 4 core functionalities:
1. Sensor inputs
SLAM typically uses sensor data (e.g., LIDAR scans, camera images, or depth sensor measurements) to detect features or landmarks in the environment.
2. State estimation
An internal state (the robot’s pose, including x, y, yaw) is estimated using algorithms like Extended Kalman Filters, Particle Filters, or Graph Optimization.
3. Map building
As the robot moves, it accumulates new sensor data. The SLAM algorithm integrates that data into a global map (2D grid map, 3D point cloud, or other representations).
4. Loop closure
When the robot revisits a previously mapped area, the SLAM algorithm detects that it’s the same place (loop closure). This knowledge is used to reduce accumulated drift and refine both the map and pose estimates.

For doing all this, we will use the `slam_toolbox` package that has to be installed first:
```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox
```

Now we will make a new launch file for mapping.

Let's also move the RViz related functions into the new launch file from `spawn_robot.launch.py`. Go to that file and comment out:
```python
# launchDescriptionObject.add_action(rviz_launch_arg)
# launchDescriptionObject.add_action(rviz_config_arg)
launchDescriptionObject.add_action(world_arg)
launchDescriptionObject.add_action(model_arg)
launchDescriptionObject.add_action(x_arg)
launchDescriptionObject.add_action(y_arg)
launchDescriptionObject.add_action(yaw_arg)
launchDescriptionObject.add_action(sim_time_arg)
launchDescriptionObject.add_action(world_launch)
# launchDescriptionObject.add_action(rviz_node)
launchDescriptionObject.add_action(spawn_urdf_node)
launchDescriptionObject.add_action(gz_bridge_node)
launchDescriptionObject.add_action(gz_image_bridge_node)
launchDescriptionObject.add_action(relay_camera_info_node)
launchDescriptionObject.add_action(robot_state_publisher_node)
launchDescriptionObject.add_action(trajectory_node)
launchDescriptionObject.add_action(ekf_node)
```
> Note: You can add a comment or make a pre-existing a line a comment in python by adding a '#' before it.

Also change the `reference_frame_id` of `erc_trajectory_server` from `odom` to `map` becase this will be our new reference frame when we have a map!

```python
trajectory_node = Node(
    package='erc_trajectory_server',
    executable='erc_trajectory_server',
    name='erc_trajectory_server',
    parameters=[{'reference_frame_id': 'map'}] # Change in this line
)
```

Let's create `mapping.launch.py`. Create the file inside the `launch` folder of `erc_ros2_navigation`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_erc_ros2_navigation = get_package_share_directory('erc_ros2_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_erc_ros2_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='mapping.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Path to the Slam Toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_params_path = os.path.join(
        get_package_share_directory('erc_ros2_navigation'),
        'config',
        'slam_toolbox_mapping.yaml'
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_erc_ros2_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(slam_toolbox_launch)

    return launchDescriptionObject
```

Build the workspace and open two terminals. Make sure to **source** your workspace in both. 

In one terminal,
```bash
ros2 launch erc_ros2_navigation spawn_robot.launch.py
```

And in another,
```bash
ros2 launch erc_ros2_navigation mapping.launch.py
```

You will be able to see an additional frame `map` over the `odom` odometry frame. This can also be visualized in RViz.
<img width="2560" height="1337" alt="image" src="https://github.com/user-attachments/assets/d93eea20-f820-4a9e-bfc8-91bb2717a066" />

With SLAM Toolbox we can also save the maps, we have two options:
1) `Save Map`: The map is saved as a `.pgm` file and a `.yaml` file. This is a black and white image file that can be used with other ROS nodes for localization as we will see later. Since it's only an image file it's impossible to continue the mapping with such a file because SLAM Toolbox handles the map in the background as a graph that cannot be restored from an image.
2) `Serialize Map`: With this feature we can serialize and later deserialize SLAM Toolbox's graph, so it can be loaded and the mapping can be continued. Although other ROS nodes won't be able to read or use it for localization.

<img width="2560" height="1339" alt="image" src="https://github.com/user-attachments/assets/cd117ed9-ef20-4f09-a543-f0fa66617e15" />

After saving a serialized map next time we can load (deserialize it):

<img width="2558" height="1337" alt="image" src="https://github.com/user-attachments/assets/e948762b-5e72-4343-9af8-b9a323242bb6" />


And we can also load the map that is in the starter package of this lesson:

<img width="2560" height="1600" alt="image" src="https://github.com/user-attachments/assets/bc7b2163-ffe7-4f4b-9dd2-ecb04d6d5809" />

The paths provided are relative, assuming that your packages are located in the src directory and you are executing mapping.launch.py from the workspace root. If the launch fails, verify your current working directory and ensure the relative path to the maps is correct from that location.

Best Practice: When saving files, specify the full directory path rather than just the filename to ensure data is stored in the intended location.


## Localization

While mapping was the process of creating a representation (a map) of an environment. Localization is the process by which a robot determines its own position and orientation within a known environment (map). In other words:

- The environment or map is typically already available or pre-built.
- The robot’s task is to figure out “Where am I?” or “Which direction am I facing?” using sensor data, often by matching its current perceptions to the known map.


### Localization with AMCL

AMCL (Adaptive Monte Carlo Localization) is a particle filter–based 2D localization algorithm. The robot’s possible poses (position + orientation in 2D) are represented by a set of particles. It adaptively samples the robot’s possible poses according to sensor readings and motion updates, converging on an accurate estimate of where the robot is within a known map.

AMCL is part of the ROS2 navigation stack, let's install it first:
```bash
sudo apt install ros-jazzy-nav2-bringup 
sudo apt install ros-jazzy-nav2-amcl
```

And then let's create a new launch file `localization.launch.py`:
```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_erc_ros2_navigation = get_package_share_directory('erc_ros2_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_erc_ros2_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='localization.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Path to the Slam Toolbox launch file
    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    localization_params_path = os.path.join(
        get_package_share_directory('erc_ros2_navigation'),
        'config',
        'amcl_localization.yaml'
    )

    map_file_path = os.path.join(
        get_package_share_directory('erc_ros2_navigation'),
        'maps',
        'my_map.yaml'
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_erc_ros2_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

 
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': localization_params_path,
                'map': map_file_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(localization_launch)

    return launchDescriptionObject
```

Following the same procedure as earlier, this file should be made in the `launch` folder. Now **build** the workspace and **source** it in both the terminals.

In one terminal,
```bash
ros2 launch erc_ros2_navigation spawn_robot.launch.py
```

And in another,
```bash
ros2 launch erc_ros2_navigation localization.launch.py
```

To start using AMCL, we have to provide an initial pose to the `/initialpose` topic. It's basically like telling the robot that hey i'm starting here at these coordinates. We can use RViz's built in tool for that.

<img width="2560" height="1334" alt="image" src="https://github.com/user-attachments/assets/a0b689dd-34c0-43d3-8772-e60b4538ae15" />

This will initialize AMCL's particles around the initial pose which can be displayed in RViz as a particle cloud where each particle rerpresnts a pose (position + orientation in 2D).

<img width="2560" height="1334" alt="image" src="https://github.com/user-attachments/assets/b65f3e6c-4d08-4b2b-a814-0a1f83198f4d" />

The main purpose of the localization algorithm is establishing the transformation between the fixed map and the robot's odometry frame based on real time sensor data. We can visualize this in RViz as we saw it during mapping:

<img width="2560" height="1334" alt="image" src="https://github.com/user-attachments/assets/9ba25a40-2275-48d8-a7a2-ed75e917af2e" />

## Localization with SLAM toolbox

It's possible to use SLAM toolbox in localization mode, it requires a small adjustment on the parameters which is already part of this lesson, `slam_toolbox_localization.yaml`. This requires the path to the serialized map file.

> Make sure the `map_file_name` parameter is changed to the path on your machine to the serialized map file!

Let's create the launch file for localization, `localization_slam_toolbox.launch.py`, it's very similar to the SLAM toolbox mapping launch file:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_erc_ros2_navigation = get_package_share_directory('erc_ros2_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_erc_ros2_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='mapping.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Path to the Slam Toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'localization_launch.py'
    )

    slam_toolbox_params_path = os.path.join(
        get_package_share_directory('erc_ros2_navigation'),
        'config',
        'slam_toolbox_localization.yaml'
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_erc_ros2_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(slam_toolbox_launch)

    return launchDescriptionObject

```

Rebuild the workspace and try it, we'll see that the map keeps updating unlike with AMCL, this is the normal behavior of the localization with SLAM toolbox. According to [this paper](https://joss.theoj.org/papers/10.21105/joss.02783), the localization mode does continue to update the pose-graph with new constraints and nodes, but the updated map expires over some time. SLAM toolbox describes it as "elastic", which means it holds the updated graph for some amount of time, but does not add it to the permanent graph.

<img width="2560" height="1338" alt="image" src="https://github.com/user-attachments/assets/a0f39aa7-acf7-4e4b-8bbb-607287ef4a49" />

# Navigation

Navigation in robotics is the overall process that enables a robot to move from one location to another in a safe, efficient, and autonomous manner. It typically involves:
1.	Knowing where the robot is (localization or SLAM),
2.	Knowing where it needs to go (a goal pose or waypoint),
3.	Planning a path to reach that goal (path planning), and
4.	Moving along that path while avoiding dynamic and static obstacles (motion control and obstacle avoidance).

ROS's nav2 navigation stack implements the above points 2 to 4, for the first point we already met several possible solutions.

Let's create a launch file that uses both AMCL and the nav2 navigation stack, `navigation.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_erc_ros2_navigation = get_package_share_directory('erc_ros2_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_erc_ros2_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Path to the Slam Toolbox launch file
    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    localization_params_path = os.path.join(
        get_package_share_directory('erc_ros2_navigation'),
        'config',
        'amcl_localization.yaml'
    )

    navigation_params_path = os.path.join(
        get_package_share_directory('erc_ros2_navigation'),
        'config',
        'navigation.yaml'
    )

    map_file_path = os.path.join(
        get_package_share_directory('erc_ros2_navigation'),
        'maps',
        'my_map.yaml'
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_erc_ros2_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': localization_params_path,
                'map': map_file_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(localization_launch)
    launchDescriptionObject.add_action(navigation_launch)

    return launchDescriptionObject
```

We'll need 2 terminals as before, one for the simulation:
```bash
ros2 launch erc_ros2_navigation spawn_robot.launch.py
```

And in another terminal we launch the new `navigation.launch.py`:

```bash
ros2 launch erc_ros2_navigation navigation.launch.py
```

We are using AMCL, so first we'll have to publish an initial pose, then we have to tell the pose goal to the navigation stack. For that we can also use RViz's other built-in feature:

<img width="2560" height="1335" alt="image" src="https://github.com/user-attachments/assets/870a8b60-ae1d-43e6-b3b3-24bb0ba6a164" />


As soon as the pose goal is received the navigation stack plans a global path to the goal and the controller ensures locally that the robot follows the global path while it avoids dynamic obstacles. The controller calculates a cost map around the robot that determines the ideal trajectory of the robot. If there aren't any obstacles around the robot this cost map weighs the global plan. 

<img width="2560" height="1336" alt="image" src="https://github.com/user-attachments/assets/b2b9a6d4-cf0c-407a-b1d4-a6f480d21d63" />


If obstacles are detected around the robot those can be visualized as a cost map too:

<img width="2560" height="1335" alt="image" src="https://github.com/user-attachments/assets/0d7df78c-1749-4a61-a619-f0a2b9bc32c8" />

## Waypoint navigation

We can use the navigation stack for waypoint navigation, this can be done through the GUI of RViz or writing a custom node. Let's start with the first one.

We'll need 2 terminals as before, one for the simulation:
```bash
ros2 launch erc_ros2_navigation spawn_robot.launch.py
```

And in another terminal we launch the `navigation.launch.py`:

```bash
ros2 launch erc_ros2_navigation navigation.launch.py
```
First, we have to make sure that the `Nav2 Goal` toolbar is added to RViz! If not, we can add it under the `+` sign.
<img width="1459" height="1229" alt="image" src="https://github.com/user-attachments/assets/36b45023-921c-4e44-9411-c6034bcaf488" />


Then we have to switch nav2 to waypoint following mode:
<img width="715" height="513" alt="image" src="https://github.com/user-attachments/assets/a2dfcb48-34a0-4156-b710-67038f527a92" />


Using the `Nav2 Goal` tool we can define the waypoints:
<img width="2560" height="1334" alt="image" src="https://github.com/user-attachments/assets/d38af408-bc20-4b0e-a2d5-19e40b5cdd08" />


And when we are done with the waypoints we can start the navigation through them:
<img width="2560" height="1337" alt="image" src="https://github.com/user-attachments/assets/79b9b18d-6f6e-4ee7-be90-45e6fa9e8a06" />

<img width="2560" height="1332" alt="image" src="https://github.com/user-attachments/assets/90c1a146-dee7-4340-8a32-859cb5426346" />

It's possible to run multiple loops through the waypoints and it's also possible to save and load the waypoints. One example is already in the `config` folder: `waypoints.yaml`. You can see a video about it here(optional):

<a href="https://youtu.be/ED6AXnAR2sc"><img width="1280" height="719" alt="image" src="https://github.com/user-attachments/assets/5ee9bf49-567a-41f1-b036-dc926dc25889" /></a> 

It's also possible to follow waypoints through the nav2 navigation stack's API with a custom node. Let's create `follow_waypoints.py` in the `erc_ros2_navigation_py` package:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

    def define_waypoints(self):
        waypoints = []

        # Waypoint 1
        wp1 = PoseStamped()
        wp1.header.frame_id = 'map'
        wp1.pose.position.x = 6.0
        wp1.pose.position.y = 1.5
        q = quaternion_from_euler(0, 0, 0)
        wp1.pose.orientation.x = q[0]
        wp1.pose.orientation.y = q[1]
        wp1.pose.orientation.z = q[2]
        wp1.pose.orientation.w = q[3]
        waypoints.append(wp1)

        return waypoints

    def send_goal(self):
        waypoints = self.define_waypoints()
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_waypoint = feedback.current_waypoint
        self.get_logger().info(f'Navigating to waypoint {current_waypoint}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Waypoint following completed.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    waypoint_follower.send_goal()
    rclpy.spin(waypoint_follower)

if __name__ == '__main__':
    main()
```

The definition of the waypoints are identical to the config file we have, we define the `x`, `y` and `z` position and the orientation in a quaternion.

Let's add the entry point to the `setup.py`:
```python
    entry_points={
        'console_scripts': [
            'send_initialpose = erc_ros2_navigation_py.send_initialpose:main',
            'slam_toolbox_load_map = erc_ros2_navigation_py.slam_toolbox_load_map:main',
            'follow_waypoints = erc_ros2_navigation_py.follow_waypoints:main',
        ],
    },
```

Build the workspace, and we'll need 3 terminals this time, one for the simulation:
```bash
ros2 launch erc_ros2_navigation spawn_robot.launch.py
```

Another terminal to launch the `navigation.launch.py`:

```bash
ros2 launch erc_ros2_navigation navigation.launch.py
```

And in the third one:
```bash
ros2 run erc_ros2_navigation_py follow_waypoints
```

We can add more waypoints easily:
```python
...
        # Waypoint 2
        wp2 = PoseStamped()
        wp2.header.frame_id = 'map'
        wp2.pose.position.x = -2.0
        wp2.pose.position.y = -8.0
        q = quaternion_from_euler(0, 0, 1.57)
        wp2.pose.orientation.x = q[0]
        wp2.pose.orientation.y = q[1]
        wp2.pose.orientation.z = q[2]
        wp2.pose.orientation.w = q[3]
        waypoints.append(wp2)

        # Waypoint 3
        wp3 = PoseStamped()
        wp3.header.frame_id = 'map'
        wp3.pose.position.x = 0.0
        wp3.pose.position.y = 0.0
        q = quaternion_from_euler(0, 0, 0)
        wp3.pose.orientation.x = q[0]
        wp3.pose.orientation.y = q[1]
        wp3.pose.orientation.z = q[2]
        wp3.pose.orientation.w = q[3]
        waypoints.append(wp3)

        # Add more waypoints as needed
...
```

<a href="https://youtu.be/3OhAyDFqBIs"><img width="1280" height="719" alt="image" src="https://github.com/user-attachments/assets/138b4a0f-0b9d-4937-9bf9-ccf708b6900b" /></a> 

## Navigation with SLAM

As we saw to navigate a mobile robot we need to know 2 things:
1.	Knowing where the robot is (localization or SLAM),
2.	Knowing where it needs to go (a goal pose or waypoint)

In the previous examples we used localization on a know map, but it's also possible to navigate together with an online SLAM. It means we don't know the complete environment around the robot but we can already navigate in the known surrounding.

Let's create a `navigation_with_slam.launch.py` launch file where we start both the SLAM toolbox and the navigation stack:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_erc_ros2_navigation = get_package_share_directory('erc_ros2_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_erc_ros2_navigation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='navigation.rviz',
        description='RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    navigation_params_path = os.path.join(
        get_package_share_directory('erc_ros2_navigation'),
        'config',
        'navigation.yaml'
    )

    slam_toolbox_params_path = os.path.join(
        get_package_share_directory('erc_ros2_navigation'),
        'config',
        'slam_toolbox_mapping.yaml'
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_erc_ros2_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Path to the Slam Toolbox launch file
    slam_toolbox_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'slam_params_file': slam_toolbox_params_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': navigation_params_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(slam_toolbox_launch)
    launchDescriptionObject.add_action(navigation_launch)

    return launchDescriptionObject
```

Build the workspace and let's try it!

We'll need 2 terminals as before, one for the simulation:
```bash
ros2 launch erc_ros2_navigation spawn_robot.launch.py
```

And in another terminal we launch the new `navigation_with_slam.launch.py`:

```bash
ros2 launch erc_ros2_navigation navigation_with_slam.launch.py
```

<img width="2560" height="1333" alt="image" src="https://github.com/user-attachments/assets/905cc6c3-6059-4061-81d1-929e2bf3cf3d" />

<a href="https://youtu.be/gZrYEP2ctfY"><img width="1281" height="719" alt="image" src="https://github.com/user-attachments/assets/611a4391-51e1-46e6-81ac-f473b881cd70" /></a> 


