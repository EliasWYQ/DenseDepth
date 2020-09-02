import os
import glob
import argparse
import numpy as np
import matplotlib
import sys
# Keras / TensorFlow
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '5'
from keras.models import load_model
from layers import BilinearUpSampling2D
from utils import predict, load_images, display_images
from matplotlib import pyplot as plt

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2 as cv



# Argument Parser
parser = argparse.ArgumentParser(description='High Quality Monocular Depth Estimation via Transfer Learning')
parser.add_argument('--model', default='kitti.h5', type=str, help='Trained Keras model file.')
parser.add_argument('--input', default='examples/0000000050.png', type=str, help='Input filename or folder.')
args = parser.parse_args()

# Custom object needed for inference and training
custom_objects = {'BilinearUpSampling2D': BilinearUpSampling2D, 'depth_loss_function': None}

print('Loading model...')

# Load model into GPU / CPU
model = load_model(args.model, custom_objects=custom_objects, compile=False)

print('\nModel loaded ({0}).'.format(args.model))

# Input images
inputs = load_images( glob.glob(args.input) )
print('\nLoaded ({0}) images of size {1}.'.format(inputs.shape[0], inputs.shape[1:]))

# Compute results
outputs = predict(model, inputs)

#matplotlib problem on ubuntu terminal fix
#matplotlib.use('TkAgg')

#import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

#import cv2 as cv

#src = cv.imread('/home/eliaswyq/DenseDepth/Segmentation/1579006203141429000.png')
#seg = load_images('/home/eliaswyq/DenseDepth/Segmentation/1579006203141429000.png')
#plt.savefig('segmentation.png')


#plt.figure(figsize=(76.8,19.2))
#viz = display_images(outputs.copy())
#plt.imshow(viz)
#plt.xticks([])
#plt.yticks([])
#plt.axis('off')
#plt.savefig('depth1.png', bbox_inches='tight',pad_inches=0.0)
#plt.show()



plt.axis('off')
distance = outputs.copy()
distance = distance[0, :, :, 0] * 80
distance = cv.resize(distance, (1242, 375))
viz = display_images(outputs.copy())
np.save('examples/DD_dep_50.npy', distance)
#viz = np.clip(viz, 0, 0.2)
#viz = np.dot(viz, 5)
fig = plt.gcf()
fig.set_size_inches(12.42/10, 3.75/10)
plt.gca().xaxis.set_major_locator(plt.NullLocator())
plt.gca().yaxis.set_major_locator(plt.NullLocator())
plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
plt.margins(0,0)
plt.imshow(viz)
fig.savefig('examples/dep_1.png', transparent=True, dpi=1000, pad_inches = 0)

# Display results
#viz = display_images(outputs.copy(), inputs.copy())
#plt.figure(figsize=(10,5))
#plt.imshow(viz)
#plt.savefig('test.png')
#plt.show()
