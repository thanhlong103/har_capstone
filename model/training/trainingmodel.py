import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from keras.models import Sequential
from keras.layers import LSTM, Dense, Dropout
from sklearn.preprocessing import OneHotEncoder

from sklearn.model_selection import train_test_split

labels = ["closeUp", "getAway", "giveNotebook", "hug", "kick", "punch", "push", "shakeHands"]

X = []
y = []
no_of_timesteps = 10

count = 0
for label in labels:
  data = pd.read_csv(label + ".txt")
  print(data.shape)
  dataset = data.iloc[:,2:].values
  n_sample = len(dataset)
  for i in range(no_of_timesteps, n_sample):
      X.append(dataset[i-no_of_timesteps:i,:])
      y.append(count)

  count += 1

print("=========TRAIN-VAL DATA PROCESSING==========")
X, y = np.array(X), np.array(y)
print(X.shape, y.shape)

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2)
print(X_train.shape, y_train.shape, X_test.shape, y_test.shape)

model = Sequential()
model.add(LSTM(units = 256, return_sequences = True, input_shape = (X.shape[1], X.shape[2])))
model.add(Dropout(0.2))
model.add(LSTM(units = 256, return_sequences = True))
model.add(Dropout(0.2))
model.add(LSTM(units = 64, return_sequences = True))
model.add(Dropout(0.2))
model.add(LSTM(units = 32, return_sequences = True))
model.add(Dropout(0.2))
model.add(LSTM(units = 16))
model.add(Dropout(0.2))
model.add(Dense(units = 64, activation="relu"))
model.add(Dense(units = 8, activation="sigmoid"))
model.summary()

# Assuming your y has unique values (0, 1, 2)
encoder = OneHotEncoder(sparse=False)
y_train_encoded = encoder.fit_transform(y_train.reshape(-1, 1))
y_test_encoded = encoder.transform(y_test.reshape(-1, 1))

# Now use y_train_encoded and y_test_encoded for training
model.compile(optimizer="adam", metrics=['accuracy'], loss="categorical_crossentropy")
history = model.fit(X_train, y_train_encoded, epochs=256, batch_size=128, validation_data=(X_test, y_test_encoded))

# Visualize the training and validation loss over epochs
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('Model loss')
plt.ylabel('Loss')
plt.xlabel('Epochs')
plt.legend(['train', 'val'], loc='upper right')
plt.show()

# Visualize the training and validation accuracy over epochs
plt.plot(history.history['accuracy'])
plt.plot(history.history['val_accuracy'])
plt.title('Model accuracy')
plt.ylabel('Accuracy')
plt.xlabel('Epochs')
plt.legend(['train', 'val'], loc='upper left')
plt.show()

from sklearn.metrics import accuracy_score, recall_score, f1_score, precision_score, confusion_matrix, ConfusionMatrixDisplay
# Evaluate the model on test data
y_pred = model.predict(X_test)
y_pred_class = np.argmax(y_pred, axis=1)  # Get predicted class labels

# Calculate evaluation metrics
accuracy = accuracy_score(y_test, y_pred_class)
recall = recall_score(y_test, y_pred_class, average='weighted')  # Micro-averaging for multi-class
precision = precision_score(y_test, y_pred_class, average='weighted')
f1_score = f1_score(y_test, y_pred_class, average='weighted')

# Calculate confusion matrix
cm = confusion_matrix(y_test, y_pred_class)

print("Accuracy:", accuracy)
print("Recall:", recall)
print("Precision:", precision)
print("F1-score:", f1_score)
print("Confusion Matrix:\n", cm)

ConfusionMatrixDisplay(cm).plot()
plt.show()

model.save("model.h5")