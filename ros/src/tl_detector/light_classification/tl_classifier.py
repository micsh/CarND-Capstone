import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import helper
from os.path import join

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        model_dir = '/home/student/finalProject/Udacity-term3-final'
        vgg_path = join(model_dir,'vgg')
        tl_path = join(model_dir,'tl_model/semantic_model_epoch_10')
        self._num_classes = 2
        
        # vgg_path = './vgg'
        self._sess = tf.Session()
        self._input_image, self._keep_prob, l3out, l4out, l7out = helper.load_vgg(self._sess,vgg_path)
        global_encoder = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES)
        rospy.loginfo('[CSChen] len(global_encoder)={}'.format(len(global_encoder)))
        with tf.variable_scope('decoder'):
            layer_output = helper.layers(l3out, l4out, l7out, self._num_classes)
        
        vars_decoder = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES,scope='decoder')
        rospy.loginfo('[CSChen] len(vars_decoder)={}'.format(len(vars_decoder)))
        saver = tf.train.Saver(var_list = vars_decoder)
        saver.restore(self._sess, tl_path)
        rospy.loginfo("[CSChen] Model loaded")

        logits = tf.reshape(layer_output,(-1,self._num_classes))
        self._out_softmax = tf.nn.softmax(logits) # batch_idx x all_pixel x num_classes


    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction
        # Should get the image data and it's shape first

        image_h, image_w = 600, 800

        out_softmax = sess.run([self._out_softmax],{self._keep_prob: 1.0, self._input_image: [image]})
        im_softmax = out_softmax[0][:, 1].reshape(image_h, image_w) # image_h, image_w
        # those points whoes prob>0.5 are our target pixels
        # segmentation = (im_softmax > 0.5).reshape(image_shape[0], image_shape[1], 1)
        return TrafficLight.UNKNOWN
