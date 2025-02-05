import numpy as np
import pandas as pd
import tensorflow as tf
import matplotlib.pyplot as plt
from keras.models import Sequential
from keras.layers import LSTM, Dense, Dropout, TimeDistributed, Conv1D, MaxPooling1D, Flatten, BatchNormalization
from keras.optimizers import SGD
from keras import backend as K

from sklearn.model_selection import train_test_split

# Read Data
# bodyswing_df = pd.read_csv("boxing.txt")
# handswing_df = pd.read_csv("handclapping.txt")

talking_df = pd.read_csv("talking.txt")
walking_df = pd.read_csv("walking.txt")
walking_phone_df = pd.read_csv("walking_phone.txt")

print(talking_df.shape)
print(walking_df.shape)
print(walking_phone_df.shape)

X = []
y = []
no_of_timesteps = 10

dataset = talking_df.iloc[:,1:].values
n_sample = len(dataset)
for i in range(no_of_timesteps, n_sample):
    X.append(dataset[i-no_of_timesteps:i,:])
    y.append(1)

dataset = walking_df.iloc[:,1:].values
n_sample = len(dataset)
for i in range(no_of_timesteps, n_sample):
    X.append(dataset[i-no_of_timesteps:i,:])
    y.append(0)

dataset = walking_phone_df.iloc[:,1:].values
n_sample = len(dataset)
for i in range(no_of_timesteps, n_sample):
    X.append(dataset[i-no_of_timesteps:i,:])
    y.append(2)

X, y = np.array(X), np.array(y)
print(X.shape, y.shape)

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)
print(X_train.shape, y_train.shape, X_test.shape, y_test.shape)

model  = Sequential()
model.add(LSTM(units = 256, return_sequences = True, input_shape = (X.shape[1], X.shape[2])))
model.add(Dropout(0.2))
model.add(LSTM(units = 256, return_sequences = True))
model.add(Dropout(0.2))
model.add(LSTM(units = 64 ,return_sequences = True))
model.add(Dropout(0.2))
model.add(LSTM(units = 32, return_sequences = True))
model.add(Dropout(0.2))
model.add(LSTM(units = 16,))
model.add(Dropout(0.2))
# model.add(Dense(units = 512, activation="relu"))
# model.add(Dense(units = 256, activation="relu"))
model.add(Dense(units = 64, activation="relu"))
model.add(Dense(units = 3, activation="softmax"))

# model = Sequential()
# model.add(LSTM(units=64, return_sequences=True, input_shape=(X.shape[1], X.shape[2])))
# model.add(Dropout(0.2))
# model.add(LSTM(units=32))
# model.add(Dropout(0.2))
# model.add(Dense(32, activation='relu'))
# model.add(Dense(6, activation='softmax'))

model.summary()

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"  # Disable GPU

from sklearn.preprocessing import OneHotEncoder

# Assuming your y has unique values (0, 1, 2)
encoder = OneHotEncoder(sparse_output=False)
y_train_encoded = encoder.fit_transform(y_train.reshape(-1, 1))
y_test_encoded = encoder.transform(y_test.reshape(-1, 1))

# Now use y_train_encoded and y_test_encoded for training
model.compile(optimizer="adam", metrics=['accuracy'], loss="categorical_crossentropy")
history = model.fit(X_train, y_train_encoded, epochs=64, batch_size=2048, validation_data=(X_test, y_test_encoded))