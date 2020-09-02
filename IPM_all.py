import os
import glob
import argparse
import numpy as np
import sys

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '5'
from keras.models import load_model
from layers import BilinearUpSampling2D
from utils import predict, load_images, display_images
from matplotlib import pyplot as plt
from IPM_1 import IPM

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2 as cv


class _DictObjHolder(object):
    def __init__(self, dct):
        self.dct = dct

    def __getattr__(self, name):
        return self.dct[name]

def rotate(image, angle, center = None, scale = 1.0):

    (h, w) = image.shape[:2]

    if center is None:
        center = (w / 2, h / 2)

    M = cv.getRotationMatrix2D(center, angle, scale)
    rotated = cv.warpAffine(image, M, (w, h))

    return rotated

if __name__ == "__main__":
    import os
#    import sys


  #  sys.path.insert(0, os.path.expanduser("~/caffe/python/"))

    import matplotlib

#    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import gc

#    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

#    import cv2 as cv


    camera_info = _DictObjHolder({

        "f_x": 718.856,  # focal length x 85000
        "f_y": 718.856,  # focal length y
        "u_x": 607.1928,  # optical center x
        "u_y": 185.2157,  # optical center y

        "camera_height": 0.0007959042,  # camera height in `m`
        "camera_x": 0.0004248599,
        "camera_y": -0.06286299,
        "pitch": 1.487732,  # rotation degree around y
        "roll": 0.04961946,  # rotation degree around x
        "yaw": -1.631226, # rotation degree around z

    })
    ipm_info = _DictObjHolder({

        "input_width": 1242,
        "input_height": 375,
        "left": 0,
        "right": 1242,
        "top": 210,
        "bottom": 375
    })

    for i in range(50, 51):

        print('\nLoaded ({0}) image.'.format(i))

        img = cv.imread('raw_data/data/{:010d}.png'.format(i))
        # img = np.uint8(np.clip((1.5 * img + 10), 0, 255))
        img = cv.resize(img, (1280, 384))
        cv.imwrite("raw_data/resize/{:010d}.png".format(i), img)

        # Argument Parser
        parser = argparse.ArgumentParser(description='High Quality Monocular Depth Estimation via Transfer Learning')
        parser.add_argument('--model', default='kitti.h5', type=str, help='Trained Keras model file.')
        parser.add_argument('--input', default='raw_data/resize/{:010d}.png'.format(i), type=str, help='Input filename or folder.')
        args = parser.parse_args()

        # Custom object needed for inference and training
        custom_objects = {'BilinearUpSampling2D': BilinearUpSampling2D, 'depth_loss_function': None}

        print('Loading model...')

        # Load model into GPU / CPU
        model = load_model(args.model, custom_objects=custom_objects, compile=False)

        print('\nModel loaded ({0}).'.format(args.model))

        # Input images
        inputs = load_images(glob.glob(args.input))
        print('\nLoaded ({0}) images of size {1}.'.format(inputs.shape[0], inputs.shape[1:]))

        # Compute results
        outputs = predict(model, inputs)

        plt.axis('off')
        viz = display_images(outputs.copy())
    #    np.save('dep_kitti_3_bright.npy', viz)


    #    dep = np.load('/home/eliaswyq/DenseDepth/dep_kitti_3.npy')
    #    dep = dep[:, :, 1]
#        dep = np.dot(80, dep)
        dep = outputs[0, :, :, 0] * 80

        dep = cv.resize(dep, (1242, 375))

        np.save('50.npy', dep)

        plt.axis('off')
        np.save('out_dep_{:010d}.png'.format(i), dep)
        # viz = np.clip(viz, 0, 0.2)
        # viz = np.dot(viz, 5)
        fig = plt.gcf()
        fig.set_size_inches(12.42 / 10, 3.75 / 10)
        plt.gca().xaxis.set_major_locator(plt.NullLocator())
        plt.gca().yaxis.set_major_locator(plt.NullLocator())
        plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)
        plt.margins(0, 0)
        plt.imshow(viz)
        fig.savefig('out_dep_{:010d}.png'.format(i), transparent=True, dpi=1000, pad_inches=0, cmap='plasma')

        seg = cv.imread('raw_data/label/{:010d}.png'.format(i))



        img = img[:, :, ::-1]


        if len(img.shape) == 3:
            img = np.dot(img, [0.299, 0.587, 0.114])

    # IPM_process
        ipm = IPM(camera_info, ipm_info, dep, seg)
        out_img = ipm()

        out_img = cv.flip(out_img, 1)
        out_img = rotate(out_img, -90, None, 1.0)

    #    fig = plt.figure()

    #    ax = fig.add_subplot(111)
    #    ax.imshow(out_img)
    #    plt.savefig("./kitti_video/IPM_kitti_3_offset_1.png")
        cv.namedWindow('image')
        out_img = out_img[:, :, ::-1]
        cv.imshow('image', out_img)
     #   cv.imwrite("./kitti_video_offset/{:010d}.png".format(i), out_img)
