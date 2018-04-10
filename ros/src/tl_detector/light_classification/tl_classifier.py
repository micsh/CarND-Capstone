import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
from os.path import join
import numpy as np
import cv2, time

class TLClassifier(object):
    def __init__(self):
        # Loading TF Model
        model_path = '../../../../final_mobilenet_frozen_sim/frozen_inference_graph.pb'
        detection_graph = tf.Graph()

        with detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(model_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')
        rospy.loginfo("[CSChen] loaded model.pb")

        self._image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        self._detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        self._detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self._detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        #self._num_detections = detection_graph.get_tensor_by_name('num_detections:0')
        #config = tf.ConfigProto(device_count={"CPU":64}, inter_op_parallelism_threads=0,intra_op_parallelism_threads=0)
        self._sess = tf.Session(graph=detection_graph)
        rospy.loginfo("[CSChen] Get tensor variables and create session")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # Should get the image data and it's shape first
        image_h_original, image_w_original, c_num = image.shape  # for simulator, 600, 800, 3
        state = TrafficLight.UNKNOWN
        image_expanded = np.expand_dims(image, axis=0)

        # Actual detection.
        (boxes,scores,classes) = self._sess.run(
                    [self._detection_boxes,self._detection_scores, self._detection_classes],
                    feed_dict={self._image_tensor: image_expanded})

        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes)
        scores = np.squeeze(scores)

        #rospy.loginfo('%s  %s',classes, scores)
        #rospy.loginfo('%s', classes[np.argmax(scores)])
        #return TrafficLight.UNKNOWN
        final_boxes = None
        final_boxes = boxes[scores>0.6]


        for i in range(len(final_boxes)):
                ymin, xmin, ymax, xmax = final_boxes[i]
                xmin, xmax = xmin*800, xmax*800
                ymin, ymax = ymin*600, ymax*600
        
                box_im=None
                box_im = image[int(ymin):int(ymax),int(xmin):int(xmax),:]
                red_Im = cv2.cvtColor(box_im, cv2.COLOR_BGR2HSV)
                lowerRed = np.array([0,50,50])
                upperRed = np.array([10,255,255])
                redSim = cv2.inRange(red_Im, lowerRed, upperRed)
                lowerRed = np.array([170,50,50])
                upperRed = np.array([180,255,255])
                redReal = cv2.inRange(red_Im, lowerRed, upperRed)
        
                combinedRedImage = cv2.addWeighted(redSim, 1.0, redReal, 1.0, 0.0)
                blurredRedImage = cv2.GaussianBlur(combinedRedImage, (15,15),0)
        
                redCircles = cv2.HoughCircles(blurredRedImage, cv2.HOUGH_GRADIENT, 0.5, 41, param1=70, param2 = 30, minRadius = 5, maxRadius = 150)
        
        
                if redCircles is not None:
                      state = TrafficLight.RED
                      rospy.loginfo('red')
                      return state
                else:
                      rospy.loginfo('other')
                      return TrafficLight.UNKNOWN
        
