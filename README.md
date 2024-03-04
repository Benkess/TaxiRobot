# TaxiRobot
Recreating the gym taxi environment in real life

This is an ongoing project and is not finalized.

I am working on implementing OpenAI's Taxi environment with a robotic car in real life and aiming to solve it using a deep reinforcement learning (RL) algorithm. The setup involves an automated vehicle with HC-020K motor encoders, an HC-SR04 ultrasonic distance sensor, and HC-06 RS232 Bluetooth for communication. This car will interact with a Raspberry Pi, which processes images from an overhead camera. The photos and sensor data will be incorporated into a Kalman Filter that estimates position. The deep RL algorithm, running on the Pi, will use the position data to dictate the car's goal location. A PID controller will then adjust the car's movements based on this goal, using feedback from the motor encoders to approximate the car's current location. A second machine-learning algorithm might verify the car's location through the camera feed. The project leverages ROS 2 for system communication.
