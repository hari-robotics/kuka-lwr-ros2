from sklearn.neural_network import MLPRegressor
from sklearn.model_selection import train_test_split
from sklearn import datasets
from sklearn import metrics
from sklearn.preprocessing import MinMaxScaler
from sklearn.metrics import mean_squared_error
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
# import rospy
import pickle

#load the dataset. The first three columns are the Force sensor data. The last three are the orientation of the end-effector.
dataset = pd.read_csv("/home/wu/Documents/project/ROS/kuka_ws/src/sriforcesensor/src/ANN/realProbe.csv") # probe_data3.csv
X = dataset.iloc[:,3:6]; y = dataset.iloc[:,0:3]
X = X.values; y = y.values
X_train, X_test, y_train, y_test = train_test_split(X, y, random_state=0, test_size=0.30)

#normalize data
#scaler = MinMaxScaler()
#X_train_norm = scaler.fit_transform(X_train)
#X_test_norm = scaler.transform(X_test)

#Build the model
model = MLPRegressor(hidden_layer_sizes=(30,),activation='relu',
                                   solver='adam',
                                   learning_rate='constant',
                                   max_iter=5000,
                                   learning_rate_init=0.001,
                                   alpha=0.01)
                                   
model.fit(X_train,y_train)

filename="realProbe.sav"
pickle.dump(model,open(filename,'wb'))
#Predict the value
expected_y  = y_test
predicted_y = model.predict(X_test)
print('R_squared',metrics.r2_score(expected_y, predicted_y))
print('MSE',metrics.mean_squared_error(expected_y[:,2],predicted_y[:,2]))

plt.figure(figsize=(5,5))
plt.scatter(expected_y[:,0], predicted_y[:,0], c='crimson')
p1 = max(max(predicted_y[:,0]), max(expected_y[:,0]))
p2 = min(min(predicted_y[:,0]), min(expected_y[:,0]))
plt.plot([p1, p2], [p1, p2], 'b-')
plt.xlabel('Fx True Values', fontsize=15)
plt.ylabel('Fx Predictions', fontsize=15)
plt.axis('equal')
plt.figure(figsize=(5,5))
plt.scatter(expected_y[:,1], predicted_y[:,1], c='crimson')
p1 = max(max(predicted_y[:,1]), max(expected_y[:,1]))
p2 = min(min(predicted_y[:,1]), min(expected_y[:,1]))
plt.plot([p1, p2], [p1, p2], 'b-')
plt.xlabel('Fy True Values', fontsize=15)
plt.ylabel('Fy Predictions', fontsize=15)
plt.axis('equal')
plt.figure(figsize=(5,5))
plt.scatter(expected_y[:,2], predicted_y[:,2], c='crimson')
p1 = max(max(predicted_y[:,2]), max(expected_y[:,2]))
p2 = min(min(predicted_y[:,2]), min(expected_y[:,2]))
plt.plot([p1, p2], [p1, p2], 'b-')
plt.xlabel('Fz True Values', fontsize=15)
plt.ylabel('Fz Predictions', fontsize=15)
plt.axis('equal')
plt.show()

