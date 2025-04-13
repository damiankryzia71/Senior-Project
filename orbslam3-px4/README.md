# Use ORB-SLAM3 with PX4 using GStreamer and a modified Gazebo Classic plugin on Ubuntu 20


## Prerequisites
Follow the instructions [here](https://github.com/damiankryzia71/Senior-Project/tree/main/orbslam3-px4-qgc-setup) to install PX4, QGroundControl, and ORB-SLAM3.

## 1. Run the Gazebo simulation
Build PX4 and run a Gazebo-simulated drone.

By default, only the Typhoon H480 model streams video, so we will use it here.

I tested in the Warehouse world.
```bash
cd PX4-Autopilot/
make px4_sitl gazebo-classic_typhoon_h480__warehouse
```
NOTE: If you get an error about `gst.h`, try re-running PX4 setup with:
```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
OR making sure GStreamer dependencies are installed with:
```bash
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```

## 2. View the camera feed inside QGroundControl
Run the QGroundControl app image.
```bash
./QGroundControl.AppImage
```
Inside QGroundControl, navigate to Application Settings -> General -> Video Settings and enable streaming from UDP h.264 Video Stream.
Leave the other options as default.

![QGC Settings](https://github.com/damiankryzia71/Senior-Project/blob/1ed7adff260b4aad2c58c51544a91dab3e8e70bd/screenshots/Screenshot%20from%202025-04-01%2018-06-33.png)

Now you should see the video feed from your Gazebo simulation inside QGroundControl.

## 3. Modify the Gazebo Gst plugin to stream to multiple ports.
By default, the Typhoon H480 model streams video to UDP port 5600. QGroundControl reads from this port and displays the video.

We will need to enable streaming on multiple ports to use the video stream in multiple programs, such as ORB-SLAM3.

The Typhoon H480 uses a Gazebo plugin that creates a GStreamer pipeline. This repository contains a modified version of that plugin in `modified_gst_plugin.cpp`.

Put the `modified_gst_plugin.cpp` source file in the `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/` directory.

Put the `modified_gst_plugin.h` header file in the `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/include/` directory.

Open the `CMakeLists.txt` file in the `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/` directory.

Locate the line:
```cmake
if (GSTREAMER_FOUND)
```
Under it, add this line:
```cmake
add_library(modified_gst_plugin SHARED src/modified_gst_plugin.cpp)
```
Modifiy the statment below it to contain the new plugin:
```cmake
set(plugins
  ${plugins}
  gazebo_gst_camera_plugin
  modified_gst_plugin
)
```
Finally, locate the SDF Typhoon H480 model at `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/typhoon_h480`.
It should contain the original plugin like so:
```sdf
<plugin name="GstCameraPlugin" filename="libgazebo_gst_camera_plugin.so">
  <robotNamespace></robotNamespace>
  <udpHost>127.0.0.1</udpHost>
  <udpPort>5600</udpPort>
</plugin>
```
Replace it with the new plugin:
```sdf
<plugin name="GstCameraPlugin" filename="libmodified_gst_plugin.so">
  <robotNamespace></robotNamespace>
  <udpHost>127.0.0.1</udpHost>
  <udpPorts>5600,5601</udpPorts>
</plugin>
```
NOTE: You may also use this plugin to stream video from other drones, such as the ones that use the depth camera. To do so, add it to `depth_camera.sdf` located at `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/depth_camera/`.

## 4. Confirm stream on multiple ports.
Rebuild the PX4 simulation.
```bash
make px4_sitl gazebo-classic_typhoon_h480__warehouse
```
Run QGroundControl and again confirm that it's receiving the video stream at UDP port 5600.
```bash
./QGroundControl.AppImage
```
Open a new terminal and, while QGroundControl is running, run the following command to confirm that the video stream can be read simultaneously from UDP port 5601.
```bash
gst-launch-1.0 -v \
  udpsrc port=5601 caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! \
  rtph264depay ! \
  avdec_h264 ! \
  videoconvert ! \
  autovideosink
```
If everything works correctly, a new window should open in which you can view the video. Both this window and the one inside QGroundControl should be working properly in parallel.

If everything is set up correctly at this point, we are able to use ORB-SLAM3 with the simulated camera.

## 5. ORB-SLAM3 Setup
Put the `px4_video_mono.cpp` file in the `ORB-SLAM3/Examples/Monocular` directory.

In the `CMakeList.txt` file, under the line:
```cmake
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
```
Add the following:
```cmake
add_executable(px4_video_mono
        Examples/Monocular/px4_video_mono.cpp)
target_link_libraries(px4_video_mono ${PROJECT_NAME})
```
Rebuild ORB-SLAM3 (in the ORB-SLAM3 directory).
```bash
./build.sh
```
The last step needed is to configure the camera. In the `ORB-SLAM3/Examples/Monocular` directory, create the configuration file.

You may also use the `px4_camera_mono.yaml` file from this repository.
```bash
touch px4_camera_mono.yaml
```
NOTE: The `px4_camera_mono.yaml` file in this repository has not yet been correctly configured, which results in bad quality in the ORB-SLAM3 viewer.

With the simulation and QGroundControl running, run the program with the ORB-SLAM3 vocabulary file and the camera configuration file as the arguments
```bash
./Examples/Monocular/px4_video_mono ./Vocabulary/ORBvoc.txt ./Examples/Monocular/px4_camera_mono.yaml
```
You might need to move the simulated drone, for example, by running the takeoff command in QGroundControl, for the ORB-SLAM3 viewer to load properly.

You should see ORB-SLAM3 running within the simulated environment in the ORB-SLAM3 viewer.

![ORB-SLAM3 Viewer Mono](https://github.com/damiankryzia71/Senior-Project/blob/2903567ff8f3655bf74684da16c5f6eac06fe27b/screenshots/Screenshot%202025-04-04%20170550.png)
