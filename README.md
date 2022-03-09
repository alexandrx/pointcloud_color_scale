# Color scale from pointcloud
-----

Generates a color scale image from the specified channel of a pointcloud.

![Color scale](docs/image-overlay.png?raw=true "Color scale") 


## Install
```
mkdir <ROS-WORKSPACE>/pointcloud_scale/src -p
cd <ROS-WORKSPACE>/pointcloud_scale/src
git clone <THIS-CODE-REPOSITORY>
cd ../
catkin build  # <-- you can also use: colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```


## Usage
```
source install/setup.bash
rosrun  ptcldcolorscale ptcldcolorscale.py _pointcloud_topic:=<POINTLCOUD_TOPIC> _channel:=<POINTCLOUD_FIELD> _image_topic:=<OUTPUT_IMAGE_TOPIC>
```

Available arguments are as follows:
- **pointcloud_topic** Name of the input pointcloud topic to use, default "/points_raw"
- **image_topic** Name of the output image topic, default "/ptcld_scale"
- **channel** Name of the channel (field) to use for generating the scale, default "intensity"
- **palette** Color palette name from matplotlib to use, default (for RVIZ) "hsv"
- **min** Minimum value in the scale, default: np.NaN which means autocompute
- **max** Maximum value in the scale, default: np.NaN which means autocompute
- **inverted** Invert the color scale, default False.

After the color scale image topic is being published, you can visualize it with the pointcloud using a JSK overlay image, and define the image topic.

![JSK overlay image](docs/rviz-add.png?raw=true "JSK overlay image") 

## TODO 
- [ ] Sometimes crashes when replaying a ROSBAG file (timestamp/sync related?)
- [ ] Support other color palettes
- [ ] It would be better to patch RVIZ's pointcloud visualizer and add the color scale there...
