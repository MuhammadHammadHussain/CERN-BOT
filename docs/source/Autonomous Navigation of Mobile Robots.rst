Autonomous Navigation of Mobile Robots
=====
Introduction
------------
The robot is designed to transport heavy loads within CERN radiation facilities, where human presence is hazardous due to high radiation levels. Previously, robots required continuous manual control and monitoring, making it impossible for operators to take breaks and leading to inefficiency and safety concerns. Autonomous navigation enables the robot to operate independently during signal loss or operator absence, improving safety, efficiency, and allowing short operator breaks.

Robot Description
------------

The key components of the robot which are relevant for this project include:

   • **LiDAR Sensor**: Used for real-time environment scanning and mapping.
   • **Wheel Odometry**:P rovides accurate movement tracking, essential for localization and navigation, especially in signal-degraded environments.
   • **Custom D-Bot Platform**: Initial testing and development were performed on a custom-built D-Bot at Aalto University, with the intention to transfer the solution to CERN’s main robot.

CERN Facility
------------
The robot operates in a radiation facility at CERN, where human presence is limited due to safety concerns. The environment is characterized by high radiation levels, which necessitates the use of autonomous systems for tasks that would otherwise require human intervention. Continuous monitoring is required due to radiation risks. Signal loss is common due to facility structure, necessitating autonomous fallback. The robot must navigate complex layouts, avoid obstacles, and maintain operational reliability 

ROS and Its Libraries
----------------
The robot utilizes the Robot Operating System (ROS) for its software framework, which provides essential libraries and tools for building robot applications. ROS facilitates communication between different components of the robot, enabling efficient data exchange and control.

Key ROS libraries and packages used include:

- **rviz**: Visualization tool for monitoring robot state, sensor data, and navigation in real time.
- **rclpy**: Python API for ROS node development.
- **nav_msgs**: Standard message types for navigation.
- **gmapping**: SLAM (Simultaneous Localization and Mapping) for 2D occupancy grid creation using LiDAR and wheel odometry.

These libraries enable modular development, allowing each component (such as sensors, actuators, and navigation algorithms) to be developed and tested independently while ensuring seamless integration within the overall system.

Algorithms are first validated in Gazebo simulation before real-world deployment.

Algorithms and Their Logic
----------------
The autonomous navigation system employs a combination of algorithms for localization, mapping, and path planning. The key algorithms include:
* **A* (A-star)**: Informed search algorithm using a heuristic to efficiently find the shortest path. Balances optimality and computational efficiency.
* **Dijkstra’s Algorithm**: Guarantees the shortest path but explores all nodes equally, leading to higher computation time.
* **RRT (Rapidly-exploring Random Tree)**: Sampling-based method suitable for complex spaces; finds feasible but often suboptimal paths and is computationally intensive.
.. code-block:: console

   (.venv) $ pip install lumache

Creating recipes
----------------

To retrieve a list of random ingredients,
you can use the ``lumache.get_random_ingredients()`` function:

.. autofunction:: lumache.get_random_ingredients

The ``kind`` parameter should be either ``"meat"``, ``"fish"``,
or ``"veggies"``. Otherwise, :py:func:`lumache.get_random_ingredients`
will raise an exception.

.. autoexception:: lumache.InvalidKindError

For example:

>>> import lumache
>>> lumache.get_random_ingredients()
['shells', 'gorgonzola', 'parsley']

