#!/usr/bin/env python
import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import os
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.current_light = TrafficLight.UNKNOWN
        file_path = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(file_path, 'models', 'ssd_mobilenet_sim.pb')
        rospy.logwarn("model_path={}".format(model_path))
        
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
        
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.sess = tf.Session(graph=self.detection_graph, config=config)
        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_rescale = cv2.resize(src=image_rgb, dsize=(256, 256))
        # Expand dimension since the model expects image to have shape [1, None, None, 3].
        img_expanded = np.expand_dims(image_rescale, axis=0) 
        with self.detection_graph.as_default():
            (boxes, scores, classes, num) = self.sess.run(
                [self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                feed_dict={self.image_tensor: img_expanded})

        # boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        
        if scores[0] > 0.6: 
            if classes[0] == 1:
                rospy.loginfo(str(scores[0])+' : GREEN Light')
                return TrafficLight.GREEN
            elif classes[0] == 2:
                rospy.loginfo(str(scores[0])+' : RED Light')
                return TrafficLight.RED
            elif classes[0] == 3:
                rospy.loginfo(str(scores[0])+' : YELLOW Light')
                return TrafficLight.YELLOW
        else:
            return TrafficLight.UNKNOWN