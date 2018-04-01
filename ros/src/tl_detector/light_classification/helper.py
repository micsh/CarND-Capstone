import re
import numpy as np
import os.path
import tensorflow as tf



def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    # TODO: Implement function
    #   Use tf.saved_model.loader.load to load the model and weights
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'
    
    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)
    graph = tf.get_default_graph()
    img_in = graph.get_tensor_by_name(vgg_input_tensor_name)
    keep = graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    l3out = graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    l4out = graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    l7out = graph.get_tensor_by_name(vgg_layer7_out_tensor_name)

    return img_in, keep, l3out, l4out, l7out


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer3_out: TF Tensor for VGG Layer 3 output, (?, 76, 100, 256)
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output, (?, 38, 50, 512)
    :param vgg_layer7_out: TF Tensor for VGG Layer 7 output, (?, 19, 25, 4096)
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    # TODO: Implement function
    # conv_1x1: (?, 19, 25, 1024)
    conv_1x1 = tf.layers.conv2d(vgg_layer7_out, 1024, 1, padding='same',
                                activation=tf.nn.relu, kernel_initializer=tf.truncated_normal_initializer(stddev=0.01),
                                kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
#    print('conv_1x1.shape={}'.format(conv_1x1.shape))
    # conv2d_transpose(input, out-dim, kernel_size, strides, ...)
    # output will have 2 times of size comparing to input
    # output: (?, 38, 50, 512)
    output = tf.layers.conv2d_transpose(conv_1x1, 512, 4, 2, padding='same',
                                        activation=tf.nn.relu, kernel_initializer=tf.truncated_normal_initializer(stddev=0.01),
                                        kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
#    print('output.shape={}'.format(output.shape))
    
    # skip Connections
    vgg_layer4_out_scaled = tf.multiply(vgg_layer4_out, 0.01, name='vgg_layer4_out_scaled')
    output = tf.add(output,vgg_layer4_out_scaled)
    
    
    # output will have 2 times of size comparing to input
    # output: (?, 76, 100, 256)
    output = tf.layers.conv2d_transpose(output, 256, 4, 2, padding='same',
                                        activation=tf.nn.relu, kernel_initializer=tf.truncated_normal_initializer(stddev=0.01),
                                        kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
#    output = output[:,:-1,:,:]
#    print('output.shape={}'.format(output.shape))

    # skip Connections
    # vgg_layer3_out_scaled: (?, 76, 100, 256)
    vgg_layer3_out_scaled = tf.multiply(vgg_layer3_out, 0.0001, name='vgg_layer3_out_scaled')
#    print('vgg_layer3_out_scaled.shape={}'.format(vgg_layer3_out_scaled.shape))
    output = tf.add(output,vgg_layer3_out_scaled)
#    print('output.shape={}'.format(output.shape))
    
    # input, out-dim, kernel_size, strides, padding,
    # output will have 8 times of size comparing to input
    # output: (?, 160, 576, num_classes)
    # this layer of output tensor should suppose to have same shape with original input image
    output = tf.layers.conv2d_transpose(output, num_classes, 16, 8, padding='same',
                                        activation=tf.nn.relu, kernel_initializer=tf.truncated_normal_initializer(stddev=0.01),
                                        kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
#    print('output.shape={}'.format(output.shape))
    return output