#!/usr/bin/env python

import numpy as np
import cv2
import rospy
import time
import tensorflow as tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def load_graph(model_file):
  graph = tf.Graph()
  graph_def = tf.GraphDef()

  with open(model_file, "rb") as f:
    graph_def.ParseFromString(f.read())
  with graph.as_default():
    tf.import_graph_def(graph_def)
  return graph


def load_labels(label_file):
  label = []
  proto_as_ascii_lines = tf.gfile.GFile(label_file).readlines()
  for l in proto_as_ascii_lines:
    label.append(l.rstrip())
  return label
    
    
M = cv2.getRotationMatrix2D((150,150), 90, 1)
output_img = np.zeros((1,299,299,3)).astype(float)
new_image = False

def imagecallback(msg):
    global output_img
    global new_image
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    img = img[100:300, 300:500]
    img = cv2.warpAffine(img, M, (299,299))
    img = np.asarray(img).astype(float)
    img -= input_mean
    img /= input_std
    output_img = np.expand_dims(img, axis=0)
    new_image = True



def main():
    global output_img
    global new_image
    with tf.Session(graph=graph) as sess:
      previous_value = 0
      previous_label = "nosign"
      while not rospy.is_shutdown():
        if rospy.get_param('/uturn_mode') ==0 and rospy.get_param('/park_mode')==0:
          if new_image:
            
            init_time = time.time()
            results = sess.run(output_operation.outputs[0],
                                {input_operation.outputs[0]: output_img})
            end=time.time()
            results = np.squeeze(results)
            top_k = results.argsort()[-5:][::-1]
            
            top_label = labels[top_k[0]]
            top_value = results[top_k[0]]

            if top_value>0.9:
              print("Detected!!!!!! ", top_label)
              if top_label == 'uturn':
                rospy.set_param('/uturn_mode',1)
              if top_label == 'park':
                rospy.set_param('/park_mode',1)

            elif top_value >=0.85:
              if previous_label == top_label and previous_value >= 0.85:
                if top_label == 'uturn':
                  rospy.set_param('/uturn_mode',1)
                if top_label == 'park':
                  rospy.set_param('/park_mode', 1)
            else:
              print("Low Confidence!")
            
            previous_value = top_value
            previous_label = top_label
            
            for i in top_k:
                print(labels[i], results[i])

            print("Time taken: ", time.time()-init_time) 
            new_image = False


if __name__=="__main__":
  bridge = CvBridge()
  rospy.init_node('traffic_cam_node', anonymous=True)
  current_time = str(time.time())
  label_file = "/home/snuzero/traffic_labels.txt"
  input_height = 299
  input_width = 299
  input_mean = 128
  input_std = 128
  input_layer = "Mul"
  output_layer = "final_result"
  graph = load_graph('/home/snuzero/final_retrained_graph_traffic.pb')

  labels = load_labels(label_file)
  input_name = "import/" + input_layer
  output_name = "import/" + output_layer
  input_operation = graph.get_operation_by_name(input_name)
  output_operation = graph.get_operation_by_name(output_name)
  sub = rospy.Subscriber('traffic_image', Image, imagecallback, queue_size=1, buff_size=2**24)

  main()