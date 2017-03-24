# Training the behavioral cloning project

# import libraries 
import tensorflow as tf
import numpy as np

from keras.models import Sequential
from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.models import model_from_json, Model
from keras.layers import merge, Input

# Load data
import pickle

print('------------------- Loding the data -------------------')
pkl_file = open('data1.pkl', 'rb')
# pkl_file = open('data0.pkl', 'rb')
data = pickle.load(pkl_file)
pkl_file.close()

X_data = data['X']
Y_data = data['Y']

n_data = len(Y_data)
n_test = int(n_data * 0.05)
n_train = n_data - n_test

# Shuffle the data 
from sklearn.utils import shuffle
X_data, Y_data = shuffle(X_data, Y_data)

X_train = X_data[:n_train, :, :, :]
X_test = X_data[n_train:, :, :, :]
Y_train = Y_data[:n_train, :]
Y_test = Y_data[n_train:, :]

print('X_train: ' + str(X_train.shape))
print('X_test: ' + str(X_test.shape))
print('Y_train: ' + str(Y_train.shape))
print('Y_test: ' + str(Y_test.shape))

# Build a model 
input_img = Input(shape = (32, 64, 3))

conv1 = Convolution2D(64, 3, 3, border_mode = 'same')(input_img)
conv1 = Convolution2D(16, 1, 1, border_mode = 'same')(conv1)
act1 = Activation('relu')(conv1)
pool1 = MaxPooling2D((3, 3))(act1)
# drop1 = Dropout(0.5)(pool1)


conv2 = Convolution2D(128, 3, 3, border_mode = 'same')(pool1)
conv2 = Convolution2D(32, 1, 1, border_mode = 'same')(conv2)
act2 = Activation('relu')(conv2)
pool2 = MaxPooling2D((2, 2))(act2)
# drop2 = Dropout(0.5)(pool2)


conv3 = Convolution2D(256, 3, 3, border_mode = 'same')(pool2)
conv3 = Convolution2D(64, 1, 1, border_mode = 'same')(conv3)
act3 = Activation('relu')(conv3)
pool3 = MaxPooling2D((2, 2))(act3)
# drop3 = Dropout(0.5)(act3)

conv4 = Convolution2D(512, 3, 3, border_mode = 'same')(pool3)
conv4 = Convolution2D(128, 1, 1, border_mode = 'same')(conv4)
act4 = Activation('relu')(conv4)
pool4 = MaxPooling2D((2, 2))(act4)

conv5 = Convolution2D(512, 3, 3, border_mode = 'same')(pool4)
conv5 = Convolution2D(128, 1, 1, border_mode = 'same')(conv5)
act5 = Activation('relu')(conv5)

conv6 = Convolution2D(512, 5, 5, border_mode = 'same')(act5)
conv6 = Convolution2D(128, 1, 1, border_mode = 'same')(conv6)
act6 = Activation('relu')(conv6)

# f4 = Flatten()(pool4)
# f5 = Flatten()(act5)
# f6 = Flatten()(act6)

# flat = merge([f5, f6], mode = 'concat', concat_axis = 1)

# f3_1 = Flatten()(act3_1)
# f3_2 = Flatten()(conv3_2)
# f3_3 = Flatten()(conv3_3)
# f3_4 = Flatten()(conv3_4)

# flat = merge([f3_1, f3_2, f3_3, f3_4], mode = 'concat', concat_axis = 1)

flat = Flatten()(act6)

den1 = Dense(1024)(flat)
act_d1 = Activation('relu')(den1)
drop_d1 = Dropout(0.5)(act_d1)

den2 = Dense(256)(drop_d1)
act_d2 = Activation('relu')(den2)
drop_d2 = Dropout(0.5)(act_d2)

den3 = Dense(64)(act_d2)
act_d3 = Activation('relu')(den3)

output = Dense(1)(act_d3)

model = Model(input= input_img, output = [output])

# Compile and train the model
import keras.optimizers

opt = keras.optimizers.Adam(lr=0.0001, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.0)
model.compile(optimizer = opt, loss = 'mean_squared_error')

history = model.fit(X_train, Y_train, batch_size = 256, nb_epoch = 16, validation_split = 0.1)

# Serialize model to json
model_json = model.to_json()
with open("model.json", "w") as json_file:
	json_file.write(model_json)

# Serialize weights to hdf5
model.save_weights("model.h5")

print("\nModel is saved!!\n")

# Evaluate model on test data
metrics = model.evaluate(X_test, Y_test)
print('test Results: ' + str(metrics))

