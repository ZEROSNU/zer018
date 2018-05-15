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
#img = np.zeros((864,480,3), dtype=np.uint8)


def imagecallback(msg):
    global output_img
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # cv2.imshow('img', img)
    img = img[100:400, 300:600]
    img = cv2.warpAffine(img, M, (299,299))
    img = np.asarray(img).astype(float)
    img -= input_mean
    img /= input_std
    output_img = np.expand_dims(img, axis=0)
    # cv2.waitKey(1)


def main():
    global output_img
    
    with tf.Session(graph=graph) as sess:
      while not rospy.is_shutdown():
        if rospy.get_param('/uturn_mode') ==0 and rospy.get_param('/park_mode')==0:

          
          init_time = time.time()
          results = sess.run(output_operation.outputs[0],
                              {input_operation.outputs[0]: output_img})
          end=time.time()
          results = np.squeeze(results)
          top_k = results.argsort()[-5:][::-1]
          

          if results[top_k[0]]>0.85:
            traffic_sign = labels[top_k[0]]
            print('\nEvaluation time: {}, Sign: {}'.format((end-init_time), traffic_sign))
            if traffic_sign == 'uturn':
              rospy.set_param('/uturn_mode',1)
            if traffic_sign == 'park':
              rospy.set_param('/park_mode',1)
          else:
            print('\nEvaluation time: {}, Sign: Low Confidence!'.format((end-init_time)))
          
          for i in top_k:
              print(labels[i], results[i])

          print("Time taken: ", time.time()-init_time) 
        cv2.waitKey(1)


if __name__=="__main__":
  bridge = CvBridge()
  rospy.init_node('traffic_cam_node', anonymous=True)
  current_time = str(time.time())
  label_file = "traffic_labels.txt"
  input_height = 299
  input_width = 299
  input_mean = 128
  input_std = 128
  input_layer = "Mul"
  output_layer = "final_result"
  graph = load_graph('final_retrained_graph_traffic.pb')

  labels = load_labels(label_file)
  input_name = "import/" + input_layer
  output_name = "import/" + output_layer
  input_operation = graph.get_operation_by_name(input_name)
  output_operation = graph.get_operation_by_name(output_name)
  sub = rospy.Subscriber('traffic_image', Image, imagecallback, queue_size=1, buff_size=2**24)

  main()