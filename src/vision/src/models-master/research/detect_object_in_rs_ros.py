#!/usr/bin/env python

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import message_filters
import rospy
from std_msgs.msg import String

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
import zipfile

from distutils.version import StrictVersion
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
#from PIL import Image

from scipy import stats

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")
from object_detection.utils import ops as utils_ops

if StrictVersion(tf.__version__) < StrictVersion('1.9.0'):
  raise ImportError('Please upgrade your TensorFlow installation to v1.9.* or later!')

waiting_for_detect = False

def callback(data):
    waiting_for_detect = data.data

# ROS Integration
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

from vision.msg import DetectedClass

pub = rospy.Publisher('object_params', DetectedClass, queue_size=100)
state_pub = rospy.Publisher('state_of_detection', String, queue_size=100)

# rospy.Subscriber("start_detecting", Bool, callback)

object_detected = DetectedClass()

class DetectedObject:

    def __init__(self, object_index, x, y, z, width, height):
        self.object_index = object_index
        self.x = x
        self.y = y
        self.z = z
        self.height = height
        self.width = width


object_buffer = []

from object_detection.utils import label_map_util

from object_detection.utils import visualization_utils as vis_util

# initialising medians
median_x = 0
median_y = 0
median_z = 0
median_height = 0
median_width = 0

confidence_threshold = 90

# defining keys
key_esc = 27
key_q = ord('q')

MODEL_NAME = 'inference_graph'
PATH_TO_FROZEN_GRAPH = '/home/prl4/Documents/EE4-Human-Centered-Robotics/src/vision/src/models-master/research/frozen_inference_graph_teddy.pb'
PATH_TO_LABELS = '/home/prl4/Documents/EE4-Human-Centered-Robotics/src/vision/src/models-master/research/labelmap_teddy.pbtxt'

detection_graph = tf.Graph()
with detection_graph.as_default():
  print(tf.__version__)
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')

category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

class ObjectDetection():

    def __init__ (self):
        self.cv_bridge = CvBridge()

        # tensorflow specific variables
        self.sess = tf.Session(graph=detection_graph)
        self.classNames = ["teddy", "remote", "book"]

        self.confidence_threshold=90

        with detection_graph.as_default():
            # Get handles to input and output tensors
            ops = tf.get_default_graph().get_operations()
            all_tensor_names = {output.name for op in ops for output in op.outputs}
            self.tensor_dict = {}
            for key in [
            'num_detections', 'detection_boxes', 'detection_scores',
            'detection_classes', 'detection_masks'
            ]:
                tensor_name = key + ':0'
                if tensor_name in all_tensor_names:
                    self.tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                tensor_name)

        # Want to synchronise the colour and depth topics
        color_msg = message_filters.Subscriber("/camera/color/image_raw", Image)
        depth_msg = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        ts = message_filters.ApproximateTimeSynchronizer([color_msg, depth_msg], 10, 0.1, allow_headerless=False)
        ts.registerCallback(self.start_detecting)
        
    def start_detecting(self, color_msg, depth_msg):
        if rospy.has_param('start_detecting'):
            global_name = rospy.get_param("/start_detecting")
            if global_name == True:
                self.callback(color_msg, depth_msg)
            else:
                print('Waiting for detection from movement')
        else:
            print('No such parameter exists')

    def dimensionHasNan(self, dimensions):
        for dimension in dimensions:
            if np.isnan(dimension):
                return True
        return False

    def callback(self, color_msg, depth_msg):

        color_image_pixels = self.cv_bridge.imgmsg_to_cv2(color_msg, desired_encoding="passthrough")
        depth_image_pixels = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        color_image_pixels = cv2.cvtColor(color_image_pixels, cv2.COLOR_BGR2RGB)

        height, width = color_image_pixels.shape[:2]
        expected = 300
        aspect = width/height
        resized_color_image = cv2.resize(color_image_pixels, (int(round(expected * aspect)), expected))
        resized_depth_image = cv2.resize(depth_image_pixels, (int(round(expected * aspect)), expected))
        crop_start = round(expected*(aspect-1)/2)
        crop_color_img = resized_color_image[0:expected, int(crop_start):int(crop_start+expected)]
        crop_depth_img = resized_depth_image[0:expected, int(crop_start):int(crop_start+expected)]

        if 'detection_masks' in self.tensor_dict:
            # The following processing is only for single image
            detection_boxes = tf.squeeze(self.tensor_dict['detection_boxes'], [0])
            detection_masks = tf.squeeze(self.tensor_dict['detection_masks'], [0])
            # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
            real_num_detection = tf.cast(self.tensor_dict['num_detections'][0], tf.int32)
            detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
            detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
            detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                detection_masks, detection_boxes, crop_color_img.shape[0], crop_color_img.shape[1])
            detection_masks_reframed = tf.cast(
                tf.greater(detection_masks_reframed, 0.5), tf.uint8)
            # Follow the convention by adding back the batch dimension
            self.tensor_dict['detection_masks'] = tf.expand_dims(
                detection_masks_reframed, 0)

        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

        # Run inference
        # Slows down feed
        output_dict = self.sess.run(self.tensor_dict, feed_dict={image_tensor: np.expand_dims(crop_color_img, 0)})

        # all outputs are float32 numpy arrays, so convert types as appropriate
        output_dict['num_detections'] = int(output_dict['num_detections'][0])
        output_dict['detection_classes'] = output_dict[
            'detection_classes'][0].astype(np.uint8)
        output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
        output_dict['detection_scores'] = output_dict['detection_scores'][0]
        if 'detection_masks' in output_dict:
            output_dict['detection_masks'] = output_dict['detection_masks'][0]

        confidence = output_dict['detection_scores'][0]*100
        print('Confidence %s' %confidence)

        xmin = output_dict['detection_boxes'][0][1]
        xmax = output_dict['detection_boxes'][0][3]
        ymin = output_dict['detection_boxes'][0][0]
        ymax = output_dict['detection_boxes'][0][2]

        width = xmax - xmin
        height = ymax - ymin

        object_index = output_dict['detection_classes'][0]-1

        detectedClass = self.classNames[object_index] 

        if object_index == 0: # teddy
            self.confidence_threshold = 50
        elif object_index == 1: #remote
            self.confidence_threshold = 60
        else: #book
            self.confidence_threshold = 60

        if not self.dimensionHasNan([xmin, xmax, ymin, ymax]) and confidence > confidence_threshold:
            cv2.rectangle(crop_color_img, (int(xmin*expected), int(ymin*expected)),
            (int((xmax)*expected), int((ymax)*expected)), (255, 255, 255), 2)
            cv2.putText(crop_color_img, detectedClass, (int(xmin*expected), int(ymin*expected)),
            cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255))

        scale=1
        xmin_depth = int((xmin * expected) * scale)
        ymin_depth = int((ymin * expected) * scale)
        xmax_depth = int((xmax * expected) * scale)
        ymax_depth = int((ymax * expected) * scale)

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(crop_depth_img, alpha = 0.03), cv2.COLORMAP_JET)

        if not self.dimensionHasNan([xmin_depth, xmax_depth, ymin_depth, ymax_depth]):
            cv2.rectangle(depth_colormap, (int(xmin_depth), int(ymin_depth)),
                (int(xmax_depth), int(ymax_depth)), (255, 255, 255), 2)
            cv2.putText(depth_colormap, detectedClass, (int(xmin_depth), int(ymin_depth)),
            cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255))

        # find depth value
        bounding_box_depth_img = depth_image_pixels
        bounding_box_depth_img = bounding_box_depth_img[xmin_depth:xmax_depth, ymin_depth:ymax_depth].astype(float)

        z = (cv2.mean((bounding_box_depth_img)/3)[0])/1000

        object_detected = DetectedClass()

        object_detected.object_index = object_index
        object_detected.x = xmin
        object_detected.y = ymin
        object_detected.z = z
        object_detected.height = height
        object_detected.width = width

        if not rospy.is_shutdown():
            state_pub.publish('Detection started')
            pub.publish(object_detected)
            detect = False

        images = np.hstack((crop_color_img, depth_colormap))
        cv2.imshow('Camera streams', images)              

        key = cv2.waitKey(1)

if __name__ == '__main__':
    try:
        rospy.init_node('listener', anonymous=True)
        object_detection = ObjectDetection()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print('Error')
    except rospy.ROSException as e:
        if str(e) == "Publish() to a closed topic":
            print('Error')
        else:
            raise e 
    except KeyboardInterrupt:
        print('Exiting node')
    print()