# Vehicle CAN Interface - Emergency Brake with LiDAR Demo

<img src="https://github.com/atefemran/Deep-Orange13-playground/blob/main/Vehicle%20CAN%20Interface%20-%20Emergency%20Brake%20with%20LiDAR%20Demo/images/viseo.png?raw=true" alt="screenshot" width="700">

The code is a part of the emergency brake demo for the vehicle interface over CAN. The node receives the point-cloud data, creates region of interest, and within this region of interest if any object passes within a predefined threshold, a command message is sent to the CAN interface pkg to forward the message over CAN to the DBW controller which is programmed to actuate the brake actuator with the received actuation distance.

Check the demo video here: https://www.linkedin.com/posts/atefemran_internship-ros-can-activity-6838241756358823936-C4_7

### The interface pipline
![picture alt](https://github.com/atefemran/Deep-Orange13-playground/blob/main/Vehicle%20CAN%20Interface%20-%20Emergency%20Brake%20with%20LiDAR%20Demo/images/readme01.PNG?raw=true)

### The demo structure 
![picture alt](https://github.com/atefemran/Deep-Orange13-playground/blob/main/Vehicle%20CAN%20Interface%20-%20Emergency%20Brake%20with%20LiDAR%20Demo/images/readme02.PNG?raw=true)

The main purpose of the node is just testing the vehicle interface pipeline created.


