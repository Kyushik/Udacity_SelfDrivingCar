# coding: utf-8

# Load pickled data
import pickle
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf
import cv2
# TODO: Fill this in based on where you saved the training and testing data

training_file = './train.p'
testing_file = "./test.p"

with open(training_file, mode='rb') as f:
    train = pickle.load(f)
with open(testing_file, mode='rb') as f:
    test = pickle.load(f)
    
X_train, y_train = train['features'], train['labels']
X_test, y_test = test['features'], test['labels']


# TODO: Number of training examples
n_train = len(X_train)

# TODO: Number of testing examples.
n_test = len(X_test)

# TODO: What's the shape of an traffic sign image?
image_shape = X_train[0].shape

# TODO: How many unique classes/labels there are in the dataset.
n_classes = max(y_train) + 1

print("Number of training examples =", n_train)
print("Number of testing examples =", n_test)
print("Image data shape =", image_shape)
print("Number of classes =", n_classes)

X_train_gray = []
for i in list(range(n_train)):
    X_train_gray.append(cv2.cvtColor(X_train[i], cv2.COLOR_RGB2GRAY))

X_test_gray = []
for i in list(range(n_test)):
    X_test_gray.append(cv2.cvtColor(X_test[i], cv2.COLOR_RGB2GRAY))


sess = tf.InteractiveSession()

def weight_variable(shape):
	initial = tf.truncated_normal(shape, stddev=0.1)
	return tf.Variable(initial)

def bias_variable(shape):
	initial = tf.constant(0.1, shape = shape)
	return tf.Variable(initial)

# Convolution and pooling
def conv2d(x,w):
	return tf.nn.conv2d(x,w,strides=[1, 1, 1, 1], padding='SAME')

def max_pool_2x2(x):
	return tf.nn.max_pool(x, ksize=[1,2,2,1], strides=[1,2,2,1], padding='VALID')

# Define variables
w_conv1 = weight_variable([5, 5, 1, 16])
b_conv1 = bias_variable([16])
w_conv2 = weight_variable([3, 3, 16, 64])
b_conv2 = bias_variable([64])
w_conv3 = weight_variable([3, 3, 64, 128])
b_conv3 = bias_variable([128])


w_fc1 = weight_variable([4*4*128, 1200])
b_fc1 = bias_variable([1200])
w_fc2 = weight_variable([1200, 500])
b_fc2 = bias_variable([500])
w_fc3 = weight_variable([500, n_classes])
b_fc3 = bias_variable([n_classes])

# x = tf.placeholder('float', shape = [None, image_shape[0]*image_shape[1]*image_shape[2]])
x = tf.placeholder('float', shape = [None, image_shape[0]*image_shape[1]])
y_label = tf.placeholder('float', shape = [None, n_classes])

x_image = tf.reshape(x, [-1,image_shape[0],image_shape[1], 1])
# Convolution and pooling
h_conv1 = tf.nn.relu(conv2d(x_image, w_conv1) + b_conv1)
h_pool1 = max_pool_2x2(h_conv1)
h_conv2 = tf.nn.relu(conv2d(h_pool1, w_conv2) + b_conv2)
h_pool2 = max_pool_2x2(h_conv2)
h_conv3 = tf.nn.relu(conv2d(h_pool2, w_conv3) + b_conv3)
h_pool3 = max_pool_2x2(h_conv3)

h3_flat = tf.reshape(h_pool3, [-1, 4*4*128])

# Fully connected layers
h_fc1 = tf.nn.relu(tf.matmul(h3_flat, w_fc1) + b_fc1)
h_fc2 = tf.nn.relu(tf.matmul(h_fc1, w_fc2) + b_fc2)

# Dropout
keep_prob = tf.placeholder("float")
# h_fc2_drop = tf.nn.dropout(h_fc2, keep_prob)

# output
output = tf.nn.softmax(tf.matmul(h_fc2, w_fc3) + b_fc3)
# output = tf.matmul(h_fc2, w_fc3) + b_fc3


from random import shuffle

cross_entropy = -tf.reduce_sum(y_label*tf.log(output + 1e-10))
train_step = tf.train.AdamOptimizer(1e-4).minimize(cross_entropy)
correct_prediction = tf.equal(tf.argmax(output,1), tf.argmax(y_label,1))
accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))

# saver = tf.train.Saver()
sess.run(tf.global_variables_initializer())

   
num_epoch = 5  
num_batch = 200
batch_moving = 1 # 1


Data_set = []
batch_index = list(range(n_train))    
count_training = 1

for i in range(num_epoch):
    print('----------------- Epoch: ' + str(i + 1) + ' -----------------')

    shuffle(batch_index)

    batch_count = 0
    is_break = 0

    while True:

        Training_X_batch = np.zeros((1, image_shape[0] * image_shape[1]))
        Training_Y_batch = np.zeros((1, n_classes))

        if batch_count + num_batch < n_train:
            batch_index_sample = batch_index[batch_count : batch_count + num_batch]
        else:
            batch_index_sample = batch_index[batch_count : n_train]
            is_break = 1

        num_row = 0

#         print(batch_index_sample)
        for k in batch_index_sample:
            y_row = np.zeros((1, n_classes))
            y_row[0, y_train[k]] = 1
            Training_X_batch = np.insert(Training_X_batch, num_row, X_train_gray[k].flatten(), 0)
            Training_Y_batch = np.insert(Training_Y_batch, num_row, y_row, 0)
            num_row += 1

        Training_X_batch = np.delete(Training_X_batch, (0), axis=0)
        Training_Y_batch = np.delete(Training_Y_batch, (0), axis=0)

        batch_count += batch_moving
#         print(Training_Y_batch.shape)
        
        if count_training%100 == 0:
            train_accuracy = accuracy.eval(feed_dict={x:Training_X_batch, y_label:Training_Y_batch, keep_prob: 1.0})
            cost_value = sess.run(cross_entropy, feed_dict={x: Training_X_batch, y_label: Training_Y_batch, keep_prob: 1.0})
            print("step %d, training accuracy %f, Cost %f"%(count_training, train_accuracy, cost_value))
            # print(batch_count)
            # print(len(batch_index_sample))
            # weight = sess.run(output, feed_dict={x: Training_X_batch, y_label: Training_Y_batch, keep_prob: 1.0})
            # for m in list(range(weight.shape[1])):
            #     print(weight[0,m])

        train_step.run(feed_dict={x:Training_X_batch, y_label:Training_Y_batch, keep_prob:0.5})

        count_training += 1

        if is_break == True:
            break

print('----------------------------- Training is finished! -----------------------------')

import os 

path = './test_image/'
files = os.listdir('./test_image')
for i, file in enumerate(files):
    print(len(file))
    img = cv2.imread(path + file)
    img = cv2.resize(img, (32,32))
    img_copy = img
    img[:,:,0] = img_copy[:,:,1]
    img[:,:,1] = img_copy[:,:,0]
    plt.figure(i)
    plt.imshow(img)
    


for i, file in enumerate(files):
    img = cv2.imread(path + file)
    img = cv2.resize(img, (32,32))
    if len(file) == 6:
        label = int(file[0:2])
    elif len(file) == 5:
        label = int(file[0:1])
    print(label)
