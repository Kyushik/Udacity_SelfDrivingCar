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
pkl_file = open('data0.pkl', 'rb')
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
input_img = Input(shape = (64, 64, 3))

conv1 = Convolution2D(32, 3, 3, border_mode = 'same')(input_img)
pool1 = MaxPooling2D((3, 3))(conv1)
act1 = Activation('relu')(pool1)

conv2 = Convolution2D(64, 3, 3, border_mode = 'same')(act1)
pool2 = MaxPooling2D((3, 3))(conv2)
act2 = Activation('relu')(pool2)

conv3 = Convolution2D(128, 2, 2, border_mode = 'same')(act2)
pool3 = MaxPooling2D((2, 2))(conv3)
act3 = Activation('relu')(pool3)

conv4 = Convolution2D(128, 2, 2, border_mode = 'same')(act3)
pool4 = MaxPooling2D((2, 2))(conv4)
act4 = Activation('relu')(pool4)

flat = Flatten()(act4)

den1 = Dense(1000)(flat)
act_d1 = Activation('relu')(den1)
drop_d1 = Dropout(0.5)(act_d1)

den2 = Dense(500)(drop_d1)
act_d2 = Activation('relu')(den2)

den3 = Dense(100)(act_d2)
act_d3 = Activation('relu')(den3)

output = Dense(1)(act_d3)

model = Model(input= input_img, output = [output])

# Load the saved weights
Is_save = input('Is there any save data? (yes: 1, no: 2) : ')

if Is_save == '1':
	model.load_weights("model.h5")
	print('\n---------------- Successfully load the weights ----------------\n')

# Compile and train the model
import keras.optimizers

opt = keras.optimizers.Adam(lr=0.0001, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.0)
model.compile(optimizer = opt, loss = 'mean_squared_error')

history = model.fit(X_train, Y_train, batch_size = 256, nb_epoch = 32, validation_split = 0.1)

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

