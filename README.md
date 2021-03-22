# set_up_livox_for_mapping

### This is an unofficial guide to help you set up a *computer* or a *raspberry pi 4 model B* from zero, in order to be able to achieve mapping results with livox horizon lidar. 
## 1) Download and install **Ubuntu 18.04 LTS** operating system. You can get it from here: https://ubuntu.com
I advise you to keep a back up of your important files before installing a new operating system.
Be carefull of the compatibilities of ubuntu images --> computer (x86 version), ---> raspberry pi (arm64).
If you are using a raspberry pi, you can use Raspberry Pi Imager software https://www.raspberrypi.org/software/.
Perhaps the full-desktop version of Ubuntu 18.04 LTS is not an option of the Raspberry Pi Imager, but you can add it like an image very easily.

---

## 2) Install **ROS Melodic** (Robot Operating System)
*(Careful, it is compatible only with Ubuntu 18.04 or else "Bionic")*

Get the instractions from here http://wiki.ros.org/melodic/Installation/Ubuntu

**Or copy from here and paste in your terminal:**
1) ***sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'***

2) ***sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654***

3) ***sudo apt update***

4) ***sudo apt install ros-melodic-desktop-full***

Before you can use **ROS** you need to initialize *rosdep* (allows you to easily install system dependencies for sources you want to compile and it is required to run some components of **ROS**). So type in your terminal:

5) ***sudo rosdep init***

6) ***rosdep update***

Congratulations, you now have **ROS** up and running.
To enable **ROS** in your system you need to **source the setup.bash of the OS workspace**, so type in your terminal:

7) ***gedit .bashrc***

Now in the .bashrc file that opened, in the end,  you need to add the command: **source /opt/ros/melodic/setup.bash** so you do not have to manually type it every time. Please save the .bashrc file and then type in your terminal:

8) ***source .bashrc***

The last step is to install tools and other dependencies to build ROS packages. Type in your terminal:

9) ***sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential***

After finishing the installation, you should create a workspace, where to put your projects and ros packages.
For this you need to create a new workspace in the home directory, and this workspace will be used later to put all your ROS projects and packages.
To do so, type in tour terminal:

10) ***mkdir -p ~/catkin_ws/src***

11) ***cd catkin_ws/***

Inside this folder, create a source file where to put your projects, then compile this new workspace. Type in your terminal inside the catkin_ws folder:

12) ***catkin_make***

---SOS---if you have any problems with the last command, especially when using the raspberry pi, you can run the command like this: *catkin_make -j2* or *catkin_make -j1* in order to use less cores, and avoid a crush. Perhaps it will take more time, but it will probably be successful ---SOS---

To enable your workspace as the default workspace you have to source the setup.bash of this workspace in the .bashrc file. So open .bashrc file with the command:

13) ***gedit .bashrc***

and add at the end of the file the following command:

14) ***source /home/ubuntu/catkin_ws/devel/setup.bash*** (be careful, "ubuntu" here is just a name, in your case it will be different)

Save it. Type in the terminal, just to be sure, the command:

15) ***source .bashrc***

Now, in order to make sure that *~/catkin_ws/devel* is the default workspace, you can type 

16) ***ros cd***

**Congratulations, now you have ROS melodic in your computer.**

---

## 3) Install Livox-SDK
You can get the instructions from here: https://github.com/Livox-SDK/Livox-SDK

**Or you can copy from here and paste to your terminal:**

1) ***sudo apt install cmake***

2) ***git clone https://github.com/Livox-SDK/Livox-SDK.git***
   ***cd Livox-SDK***
   ***cd build && cmake ..***
   ***make***
   ***sudo make install***

Installation complete.

If you want to capture and save a file in .lvx format type:
1) cd build
2) cd Livox-SDK
3) cd build
4) cd sample
5) cd lidar_lvx_file
6) ./lidar_lvx_sample -l -t 10 (read the Program Options for more info)

Program Options:

[-t] Time to save point cloud to the lvx file.(unit: s)

[-p] Get the extrinsic parameter from standard extrinsic.xml file (The same as viewer) in the executable file's directory.

Here is the example:

./lidar_lvx_sample -c "00000000000002&00000000000003&00000000000004" -l -t 10 -p

Note: .lvx files can be opened through livox_viewer, you can play them like a video, and if you want you can export **just a frame** in .las or in .csv format.

---

## 4) Install Livox ROS Driver
You can get the instruction from here: https://github.com/Livox-SDK/livox_ros_driver

**or you can copy from here and paste in your terminal:**

1) ***git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src***

2) ***cd ws_livox***
 
3) ***catkin_make***

4) ***source ./devel/setup.sh***

You are ready. 

If you want you can run it, for example with the command: ***roslaunch livox_ros_driver livox_lidar_rviz.launch***

It is important to read the instructions and the info of the livox_ros_driver command options from here: https://github.com/Livox-SDK/livox_ros_driver


Also, extremelly important is the **json** file from where you can pre-set some important parameters.

The json configuration file is in the ***ws_livox/src/livox_ros_driver/config*** directory.

---

## 5) Install the livox_mapping package
Livox_mapping is a mapping package for Livox LiDARs. The package currently contains the basic functions of low-speed mapping.

You can follow the instructions from here: https://github.com/Livox-SDK/livox_mapping
**or copy from here and paste in your terminal:**

1) ***sudo apt install libpcl-dev***

2) ***sudo apt-get update***

3) ***sudo apt-get install libeigen3-dev***

4) ***sudo apt update***

5) ***sudo apt install python3-opencv***

6) ***cd ~/catkin_ws/src***

7) ***git clone https://github.com/Livox-SDK/livox_mapping.git***

8) ***cd ..***

9) ***catkin_make***

10) ***source ~/catkin_ws/devel/setup.bash***

Installation complete.



For Livox Horizon, run the next two commands in **defferent terminals**:


terminal_1) ***roslaunch livox_mapping mapping_horizon.launch***

terminal_2) ***roslaunch livox_ros_driver livox_lidar_msg.launch***


If you want to **save the pcd** file, before you run those two commands above, please add a path at map_file_path parameter in launch file of the *mapping_horizon.launch*.


Try to keep livox horizon steady, perhaps in a tripod, and move it slowly to cover all your region of interest.

When you stop those 2 commands, you will find a .pcd file in the path that you hαve typed in the *map_file_path*.

You can open this .pcd file with **Cloud Compare** or with any other similar point-cloud software that supports .pcd file format.

---

Some notes that came through many hours of struggle:

1) Time sync with livox horizon, from my personal exprerience, is not an easy thing to achive. Until this day i am not able to achive PPS & GPRMC synchronization.
2) Livox Viewer, is a nice way to open .lvx files and have access to the internal information of the points and sensors in a .csv format. Unfortunatelly this works for only one frame at a time. Actually .lvx file recording is more like a video capturing. You can see the info of all recorded points separetely, but you cannot get a complete .csv file with every single point that you recorded.
3) Even if you achieve time sync with Livox_ros_driver, when you use the livox_mapping i believe that it is useless, because the package can only give you a well-registered point cloud in a .pcd format. This means that through the **Cloud Compare** the info that you can actually have access to, is the x,y,z info of all the points in a reference system with 0,0,0 at the beginning of the lidar. Nothing else.
4) When you record data with livox_mapping, you have about 2-3 minutes of capturing because in my case if you keep recording for more time, it will crash and the system will not be able to save a .pcd file. This happens more often when i try to use livox_mapping from my raspberry pi 4.
5) Livox Horizon lidar with livox_mapping package CANNOT BE USED in in-door situations, it is impossible to give you back accurate results. It must be used only outside, because the out-door environment enables the program to get many different and clear points in order to achieve good mapping results.
6) In my humble and non-professional opinion you can use livox horizon in two different ways.
   - scenario-A) You can use it as a static Scanner. Put livox in a tripod, and through the Livox-SDK get an .lvx file without moving the lidar. Next you move the tripod in a different location, with good overlapping of points and repeat the procedure. when you have covered your area of interest, you can put the .lvx files in the livox viewer and extract them as .las files. Then through **Cloud Compare** software you can try to force them in one registered pointcloud. I HAVE NOT tested his senario yet.
   - scenario-B) You can use livox_mapping package. With 2 commands in 2 different terminals, you can start the lidar and with ctrl-c in each terminal you can stop the procedure. When you stop it, you will find a .pcd file in the path that you chosed. Next you can open the .pcd file at **Cloud Compare** to examine the mapping results. I HAVE tested this scenario with great success.
---
Remember, everything written here is just my opinion and my personal experience.

Always double-check everything from the official instructions of each package.

All i wanted to do, was to write down my experience in order to remember the procedure in the future, and help the lidar community.

In the near future, i will try to improve more this guide.


Keep walking fellows.
