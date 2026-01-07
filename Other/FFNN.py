import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Input

# Define a simple feedforward model
model = Sequential([
    Input(shape=(num_sensor_inputs,)), # e.g., temperature, pressure, humidity inputs
    Dense(64, activation='relu'),      # Hidden layer 1
    Dense(32, activation='relu'),      # Hidden layer 2
    Dense(1, activation='linear')      # Output layer (e.g., a prediction or single sensor reading)
])

# Compile the model
model.compile(optimizer='adam', loss='mean_squared_error')

# You would then train this model with your sensor data
# model.fit(X_train, y_train, epochs=10)
# And use it to make predictions
# predictions = model.predict(X_test)
num_sensor_inputs = 3  # Example number of sensor inputs
