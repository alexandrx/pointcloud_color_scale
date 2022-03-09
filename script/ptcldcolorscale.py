import sys
import rospy
import numpy as np
import cv2
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField, Image
from ros_numpy import numpy_msg

from cv_bridge import CvBridge
import matplotlib as mpl
mpl.use('TkAgg')
mpl.rcParams.update({'font.size': 12})
from matplotlib import pyplot as plt
from matplotlib import colors as colors
from matplotlib.ticker import FixedLocator, FixedFormatter
import rospkg
import os

__author__ = "Alexander Carballo"
__email__ = "alexander@g.sp.m.is.nagoya-u.ac.jp"

def package(path = ""):
    rospack = rospkg.RosPack()
    return os.path.join(rospack.get_path("ptcldcolorscale"), path)

class PtCldColorScale(object):
    def __init__(self):
        self.ros_init()

    def ros_init(self):
        rospy.init_node('ptcldcolorscale', anonymous=True)
        self.ptcld_topic = rospy.get_param("~pointcloud_topic", "/points_raw")
        self.img_topic = rospy.get_param("~image_topic", "/ptcld_scale")
        self.palettename = rospy.get_param("~palette", "hsv")
        self.channel = rospy.get_param("~channel", "intensity")
        self.minval = rospy.get_param("~min", np.NaN)
        self.maxval = rospy.get_param("~max", np.NaN)
        self.inverted = rospy.get_param("~inverted", False)
        self.bridge = CvBridge()
        self.cmap = self.truncate_colormap(plt.get_cmap(self.palettename), 0, 0.83333, 256) # According to RVIZ's implementation
        if (self.inverted):
            self.gradient = np.linspace(1.0, 0.0, 256)
        else:
            self.gradient = np.linspace(0.0, 1.0, 256) 
        self.gradient = np.vstack((self.gradient, self.gradient))
        self.pub_scale = rospy.Publisher(self.img_topic, Image, queue_size=10)
        rospy.Subscriber(self.ptcld_topic, PointCloud2, self.ptcld_callback)

    def truncate_colormap(self, cmap, minval=0.0, maxval=1.0, n=100):
        new_cmap = colors.LinearSegmentedColormap.from_list('trunc({n},{a:.2f},{b:.2f})'.format(n=cmap.name, a=minval, b=maxval),
            cmap(np.linspace(minval, maxval, n)))
        return new_cmap

    def make_scale(self, minval, maxval):
        # Plot the color scale
        fig, ax = plt.subplots(nrows=1, figsize=(10, 1))
        ax = fig.add_axes([0, 0, 1, 1])
        ax.imshow(self.gradient, aspect='auto', cmap=self.cmap)
        ax.text(0, 0, str(minval), color='white', fontsize=20, rotation=-90, fontweight='bold')
        ax.text(246, 0, str(maxval), color='white', fontsize=20, rotation=-90, fontweight='bold')
        fig.patch.set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['left'].set_visible(False)
        fig.canvas.draw()

        # Convert it to image
        #img = cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba()), cv2.COLOR_RGBA2BGR) # did not work    
        # convert canvas to image
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        plt.close(fig)
        img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
        img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return img

    def checkchannel(self, pointcloud_msg):
        return any(f for f in pointcloud_msg.fields if f.name == self.channel)

    def getcolorscale(self, pointcloud_msg):
        """Finds the requested channel in the point cloud and create the color scale."""
        if (self.checkchannel(pointcloud_msg)):
            points_arr = ros_numpy.numpify(pointcloud_msg)
            points_view = points_arr.view(np.recarray)
            minval = self.minval
            maxval = self.maxval
            if (np.isnan(minval)):
                minval = points_view[self.channel].min()
            if (np.isnan(maxval)):
                maxval = points_view[self.channel].max()
            
            return self.make_scale(minval, maxval)

    def ptcld_callback(self, pointcloud_msg):
        self.publish(self.getcolorscale(pointcloud_msg))

    def publish(self, image):
        self.pub_scale.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ptcldcolorscale = PtCldColorScale()
    ptcldcolorscale.run()
