# Vehicle CAN Interface - Emergency Brake with LiDAR Demo

![picture alt](https://github.com/atefemran/Deep-Orange13-playground/blob/main/Vehicle%20CAN%20Interface%20-%20Emergency%20Brake%20with%20LiDAR%20Demo/images/readme01.PNG?raw=true)

The code is a part of the emergency brake demo for the vehicle interface over CAN. The node receives the point-cloud data, creates region of interest, and within this region of interest if any object passes within a predefined threshold, a command message is sent to the CAN interface pkg to forward the message over CAN to the DBW controller which is programmed to actuate the brake actuator with the received actuation distance.

![picture alt](https://github.com/atefemran/Deep-Orange13-playground/blob/main/Vehicle%20CAN%20Interface%20-%20Emergency%20Brake%20with%20LiDAR%20Demo/images/readme02.PNG?raw=true)

The main purpose of the node is just testing the vehicle interface pipeline created.


