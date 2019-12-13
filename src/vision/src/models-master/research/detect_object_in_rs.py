#!/usr/bin/env python
# coding: utf-8

# # Detection Objects in a webcam image stream

# <table align="left"><td>
#   <a target="_blank"  href="https://colab.research.google.com/github/TannerGilbert/Tutorials/blob/master/Tensorflow%20Object%20Detection/detect_object_in_webcam_video.ipynb">
#     <img src="https://www.tensorflow.org/images/colab_logo_32px.png" />Run in Google Colab
#   </a>
# </td><td>
#   <a target="_blank"  href="https://github.com/TannerGilbert/Tutorials/blob/master/Tensorflow%20Object%20Detection/detect_object_in_webcam_video.ipynb">
#     <img width=32px src="https://www.tensorflow.org/images/GitHub-Mark-32px.png" />View source on GitHub</a>
# </td></table>

# This notebook is based on the [official Tensorflow Object Detection demo](https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb) and only contains some slight changes. Make sure to follow the [installation instructions](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md) before you start.

# # Imports

# In[8]:


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
from PIL import Image

from scipy import stats

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")
from object_detection.utils import ops as utils_ops

if StrictVersion(tf.__version__) < StrictVersion('1.9.0'):
  raise ImportError('Please upgrade your TensorFlow installation to v1.9.* or later!')

waiting_for_detect = False

def callback(data):
    waiting_for_detect = data.data

# def image_received(image):
#    converter = CvBridge()
#    img = converter.cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

# ROS Integration
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

from vision.msg import DetectedClass

pub = rospy.Publisher('object_params', DetectedClass, queue_size=10)
rospy.init_node('object_detect', anonymous=True)
rospy.Subscriber("start_detecting", Bool, callback)
#rospy.Subscriber("image_topic", Image, image_received)
rate = rospy.Rate(10)

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

# ## Env setup

# In[9]:


# This is needed to display the images.
#get_ipython().run_line_magic('matplotlib', 'inline')


# ## Object detection imports
# Here are the imports from the object detection module.

# In[10]:


from object_detection.utils import label_map_util

from object_detection.utils import visualization_utils as vis_util


# # Model preparation

# ## Variables
#
# Any model exported using the `export_inference_graph.py` tool can be loaded here simply by changing `PATH_TO_FROZEN_GRAPH` to point to a new .pb file.
#
# By default we use an "SSD with Mobilenet" model here. See the [detection model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) for a list of other models that can be run out-of-the-box with varying speeds and accuracies.

# In[4]:

MODEL_NAME = 'inference_graph'
PATH_TO_FROZEN_GRAPH = '/home/joe/Documents/EE4-Human-Centered-Robotics/src/vision/src/models-master/research/frozen_inference_graph.pb'
PATH_TO_LABELS = '/home/joe/Documents/EE4-Human-Centered-Robotics/src/vision/src/models-master/research/labelmap_teddy.pbtxt'

# What model to download.
#MODEL_NAME = 'ssd_mobilenet_v1_coco_2017_11_17'
#MODEL_FILE = MODEL_NAME + '.tar.gz'
#DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'

# Path to frozen detection graph. This is the actual model that is used for the object detection.
#PATH_TO_FROZEN_GRAPH = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
#PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')


# ## Download Model

# In[5]:


#opener = urllib.request.URLopener()
#opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
#tar_file = tarfile.open(MODEL_FILE)
#for file in tar_file.getmembers():
#  file_name = os.path.basename(file.name)
#  if 'frozen_inference_graph.pb' in file_name:
#    tar_file.extract(file, os.getcwd())


# ## Load a (frozen) Tensorflow model into memory.

# In[11]:


detection_graph = tf.Graph()
with detection_graph.as_default():
  print(tf.__version__)
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')


# ## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`, we know that this corresponds to `airplane`.  Here we use internal utility functions, but anything that returns a dictionary mapping integers to appropriate string labels would be fine

# In[18]:

category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)


# # Detection

# In[19]:

import pyrealsense2 as rs
import cv2

def dimensionHasNan(dimensions):
    for dimension in dimensions:
        if np.isnan(dimension):
            return True
    return False

pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

profile = pipeline.start(config)

# defining keys
key_esc = 27
key_q = ord('q')

# initialising medians
median_x = 0
median_y = 0
median_z = 0
median_height = 0
median_width = 0

confidence_threshold = 90


for _ in range(10):
    pipeline.wait_for_frames()

def run_inference_for_single_image(image, graph):
    if 'detection_masks' in tensor_dict:
        # The following processing is only for single image
        detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
        detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
        # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
        real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
        detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
        detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
            detection_masks, detection_boxes, image.shape[0], image.shape[1])
        detection_masks_reframed = tf.cast(
            tf.greater(detection_masks_reframed, 0.5), tf.uint8)
        # Follow the convention by adding back the batch dimension
        tensor_dict['detection_masks'] = tf.expand_dims(
            detection_masks_reframed, 0)
    image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

    # Run inference
    output_dict = sess.run(tensor_dict,
                            feed_dict={image_tensor: np.expand_dims(image, 0)})

    # all outputs are float32 numpy arrays, so convert types as appropriate
    output_dict['num_detections'] = int(output_dict['num_detections'][0])
    output_dict['detection_classes'] = output_dict[
        'detection_classes'][0].astype(np.uint8)
    output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
    output_dict['detection_scores'] = output_dict['detection_scores'][0]
    if 'detection_masks' in output_dict:
        output_dict['detection_masks'] = output_dict['detection_masks'][0]

    return output_dict

try:
    with detection_graph.as_default():
        with tf.Session() as sess:
                # Get handles to input and output tensors
                ops = tf.get_default_graph().get_operations()
                all_tensor_names = {output.name for op in ops for output in op.outputs}
                tensor_dict = {}
                for key in [
                  'num_detections', 'detection_boxes', 'detection_scores',
                  'detection_classes', 'detection_masks'
                ]:
                    tensor_name = key + ':0'
                    if tensor_name in all_tensor_names:
                        tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                      tensor_name)

                classNames = ["teddy", "remote", "book"]

                while True:# and waiting_for_detect:

                    frameset = pipeline.wait_for_frames()
                    color_frame = frameset.get_color_frame()
                    depth_frame = frameset.get_depth_frame()
                    if not depth_frame: continue

                    # get pixel values
                    color_image_pixels = np.asanyarray(color_frame.get_data())
                    depth_image_pixels = np.asanyarray(depth_frame.get_data())

                    # align frames
                    align = rs.align(rs.stream.color)
                    frameset = align.process(frameset)

                    # update color and depth frames
                    aligned_depth_frame = frameset.get_depth_frame()
                    depth_image_pixels = np.asanyarray(aligned_depth_frame.get_data())

                    # pre-process image
                    height, width = color_image_pixels.shape[:2]
                    expected = 300
                    aspect = width/height
                    resized_color_image = cv2.resize(color_image_pixels, (int(round(expected * aspect)), expected))
                    resized_depth_image = cv2.resize(depth_image_pixels, (int(round(expected * aspect)), expected))
                    crop_start = round(expected*(aspect-1)/2)
                    crop_color_img = resized_color_image[0:expected, int(crop_start):int(crop_start+expected)]
                    crop_depth_img = resized_depth_image[0:expected, int(crop_start):int(crop_start+expected)]

                    # ret, image_np = cap.read()
                    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                    # image_np_expanded = np.expand_dims(image_np, axis=0)
                    # Actual detection.
                    output_dict = run_inference_for_single_image(crop_color_img, detection_graph)
                    # Visualization of the results of a detection.
                    #vis_util.visualize_boxes_and_labels_on_image_array(
                    #    crop_color_img,
                    #    output_dict['detection_boxes'],
                    #    output_dict['detection_classes'],
                    #    output_dict['detection_scores'],
                    #    category_index,
                    #    instance_masks=output_dict.get('detection_masks'),
                    #    use_normalized_coordinates=True,
                    #    line_thickness=5)

                    confidence = output_dict['detection_scores'][0]*100
                    print('Confidence %s' %confidence)

                    # detection_box = output_dict['detection_boxes'][0]
                    # print(detection_box)

                    xmin = output_dict['detection_boxes'][0][1]
                    xmax = output_dict['detection_boxes'][0][3]
                    ymin = output_dict['detection_boxes'][0][0]
                    ymax = output_dict['detection_boxes'][0][2]

                    width = xmax - xmin
                    height = ymax - ymin

                    x_for_depth = xmin + 0.5*width
                    y_for_depth = ymin + 0.5*height

                    # need to check how to avoid fitting long inside index sized array
                    detectedClass = classNames[output_dict['detection_classes'][0]-1]

                    #scale = height/expected
                    scale=1
                    xmin_depth = int((xmin * expected) * scale)
                    ymin_depth = int((ymin * expected) * scale)
                    xmax_depth = int((xmax * expected) * scale)
                    ymax_depth = int((ymax * expected) * scale)



                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(crop_depth_img, alpha = 0.03), cv2.COLORMAP_JET)

                    if not dimensionHasNan([xmin_depth, xmax_depth, ymin_depth, ymax_depth]):
                        cv2.rectangle(depth_colormap, (int(xmin_depth), int(ymin_depth)),
                          (int(xmax_depth), int(ymax_depth)), (255, 255, 255), 2)
                        cv2.putText(depth_colormap, detectedClass, (int(xmin_depth), int(ymin_depth)),
                        cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255))

                    # find depth value
                    bounding_box_depth_img = np.asanyarray(aligned_depth_frame.get_data())
                    bounding_box_depth_img = bounding_box_depth_img[xmin_depth:xmax_depth, ymin_depth:ymax_depth].astype(float)

                    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
                    bounding_box_depth_img = bounding_box_depth_img * depth_scale

                    # 3 was a depth correction - take 1st index (not RGB)
                    z = cv2.mean((bounding_box_depth_img)/3)[0]

                    # show detected objects in the live frame (top corners)
                    # print('  x: %f' %xmin)
                    # print('  y: %f' %ymin)
                    # print('  z: %f' %z)

                    # print('  Width: %f' %(width))
                    # print('  Height: %f' %(height))

                    if (len(object_buffer) < 10):
                    #     # store in buffer

                        objectIdx = output_dict['detection_classes'][0]-1

                        object_buffer.append(DetectedObject(objectIdx, xmin, ymin, z, width, height))

                    else:
                         # read from buffer (print)
                        #object_index_array  = object_buffer[:]
                        object_index_array = [object_buffer[i].object_index for i in range(10)]

                        #for i in range(10):
                        #  object_index_array[i] = object_buffer[i].object_index

                        modal_label = stats.mode(np.asarray(object_index_array)).mode[0]

                        #buffer = np.asarray(object_buffer.object_index)

                        #filtered_array = np.where(buffer == modal_label)

                        filtered_array = []
                        for i in range(10):
                            if object_buffer[i].object_index == modal_label:
                                filtered_array.append(object_buffer[i])

                        filtered_array = np.asarray(filtered_array)

                        median_x = np.median([element.x for element in filtered_array])
                        median_y = np.median([element.y for element in filtered_array])
                        median_z = np.median([element.z for element in filtered_array])
                        median_height = np.median([element.height for element in filtered_array])
                        median_width = np.median([element.width for element in filtered_array])

                        #print(modal_label.mode[0])
                        # print('  x: %f' %median_x)
                        # print('  y: %f' %median_y)
                        # print('  z: %f' %median_z)

                        # print('  Width: %f' %median_width)
                        # print('  Height: %f' %median_height)

                        object_buffer = []


                        object_detected.object_index = modal_label
                        object_detected.x = median_x
                        object_detected.y = median_y
                        object_detected.z = median_z
                        object_detected.height = median_height
                        object_detected.width = median_width

                        if not rospy.is_shutdown():
                            rospy.loginfo(object_detected)
                            pub.publish(object_detected)
                            detect = False
                            #rate.sleep()

                        if modal_label == 0: # teddy
                            confidence_threshold = 50
                        elif modal_label == 1: #remote
                            confidence_threshold = 40
                        else: #book
                            confidence_threshold = 45

                    # ensure that the dimensions can be converted to Int (ie not NaN)
                    if not dimensionHasNan([median_x, median_x+median_width, median_y, median_y+median_height]) and confidence>confidence_threshold:
                        cv2.rectangle(crop_color_img, (int(median_x*expected), int(median_y*expected)),
                        (int((median_x+median_width)*expected), int((median_y+median_height)*expected)), (255, 255, 255), 2)
                        cv2.putText(crop_color_img, detectedClass, (int(median_x*expected), int(median_y*expected)),
                        cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255))
                        print('Detected: %s' %detectedClass)


                    images = np.hstack((crop_color_img, depth_colormap))
                    cv2.imshow('Camera streams', images)

                    key = cv2.waitKey(1)
                    if key == key_esc or key & 0xFF == key_q:
                        cv2.destroyAllWindows()
                        break

finally:
    pipeline.stop()
