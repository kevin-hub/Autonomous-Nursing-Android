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

pub = rospy.Publisher('object_params', DetectedClass, queue_size=10)
rospy.Subscriber("start_detecting", Bool, callback)

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

def dimensionHasNan(dimensions):
    for dimension in dimensions:
        if np.isnan(dimension):
            return True
    return False

# initialising medians
median_x = 0
median_y = 0
median_z = 0
median_height = 0
median_width = 0

confidence_threshold = 90

def run_inference_for_single_image(image, graph, sess, tensor_dict):
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

# defining keys
key_esc = 27
key_q = ord('q')

def object_detction(color_frame, depth_frame):  
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

                color_image_pixels = color_frame
                depth_image_pixels = depth_frame

                height, width = color_image_pixels.shape[:2]
                expected = 300
                aspect = width/height
                resized_color_image = cv2.resize(color_image_pixels, (int(round(expected * aspect)), expected))
                resized_depth_image = cv2.resize(depth_image_pixels, (int(round(expected * aspect)), expected))
                crop_start = round(expected*(aspect-1)/2)
                crop_color_img = resized_color_image[0:expected, int(crop_start):int(crop_start+expected)]
                crop_depth_img = resized_depth_image[0:expected, int(crop_start):int(crop_start+expected)]

                output_dict = run_inference_for_single_image(crop_color_img, detection_graph, sess, tensor_dict)

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

                # need to check how to avoid fitting long inside index sized array
                detectedClass = classNames[output_dict['detection_classes'][0]-1] 

                # ensure that the dimensions can be converted to Int (ie not NaN)
                if not dimensionHasNan([xmin, xmax, ymin, ymax]):
                    cv2.rectangle(crop_color_img, (int(xmin*expected), int(ymin*expected)),
                          (int(xmax*expected), int(ymax*expected)), (255, 255, 255), 2)
                    cv2.putText(crop_color_img, detectedClass, (int(xmin*expected), int(ymin*expected)),
                    cv2.FONT_HERSHEY_COMPLEX, 1.0, (255, 255, 255))                     

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
                bounding_box_depth_img = depth_frame
                bounding_box_depth_img = bounding_box_depth_img[xmin_depth:xmax_depth, ymin_depth:ymax_depth].astype(float)
    
                # depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
                # bounding_box_depth_img = bounding_box_depth_img * depth_scale
                
                # 3 was a depth correction - take 1st index (not RGB)
                z = cv2.mean((bounding_box_depth_img)/3)[0]
                

                object_detected.object_index = detectedClass
                object_detected.x = xmin
                object_detected.y = ymin
                object_detected.z = z
                object_detected.height = height
                object_detected.width = width

                if not rospy.is_shutdown():
                    rospy.loginfo(object_detected)

                '''
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

                    if modal_label == 0: # bottle
                        confidence_threshold = 65
                    elif modal_label == 1: #remote
                        confidence_threshold = 60
                    else: #book
                        confidence_threshold = 60

                '''

                # show detected objects in the live frame (top corners)
                # print('  x: %f' %xmin)
                # print('  y: %f' %ymin)
                print('  z: %f' %z)

                # print('  Width: %f' %(width))
                # print('  Height: %f' %(height))

                images = np.hstack((crop_color_img, depth_colormap))
                cv2.imshow('Camera streams', images)              

                key = cv2.waitKey(1)
    
    finally:
        rospy.loginfo('Done')

def process_images(color_msg, depth_msg):
    bridge = CvBridge()
    color_frame = bridge.imgmsg_to_cv2(color_msg, desired_encoding="passthrough")
    depth_frame = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

    object_detction(color_frame, depth_frame)

def listener():
    rospy.init_node('listener', anonymous=True)
    
    # Subscribes to chatter topic
    # Type: std_msg.msgs.String
    # When new messages are recieved callback is invoked with message as first argument
    color_msg = message_filters.Subscriber("/camera/color/image_raw", Image)
    depth_msg = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
    ts = message_filters.ApproximateTimeSynchronizer([color_msg, depth_msg], 10, 0.1, allow_headerless=False)
    ts.registerCallback(process_images)

    print('Init done')

    # spin() simply keeps python from 
    rospy.spin()

if __name__ == '__main__':
    listener()
