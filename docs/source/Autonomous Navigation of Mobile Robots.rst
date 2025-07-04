Autonomous Navigation of Mobile Robots
=====
Introduction
------------
The robot is designed to transport heavy loads within CERN radiation facilities, where human presence is hazardous due to high radiation levels. Previously, robots required continuous manual control and monitoring, making it impossible for operators to take breaks and leading to inefficiency and safety concerns. Autonomous navigation enables the robot to operate independently during signal loss or operator absence, improving safety, efficiency, and allowing short operator breaks.

Robot Description
^^^^^^^^^^^^^^^

The key components of the robot which are relevant for this project include:

- **LiDAR Sensor**: Used for real-time environment scanning and mapping.
- **Wheel Odometry**:Provides accurate movement tracking, essential for localization and navigation, especially in signal-degraded environments.
- **Custom D-Bot Platform**: Initial testing and development were performed on a custom-built D-Bot at Aalto University, with the intention to transfer the solution to CERN’s main robot.

CERN Facility
^^^^^^^^^^^^
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

- **A (A-star)**: Informed search algorithm using a heuristic to efficiently find the shortest path. Balances optimality and computational efficiency.
- **Dijkstras Algorithm**: Guarantees the shortest path but explores all nodes equally, leading to higher computation time.
- **RRT (Rapidly-exploring Random Tree)**: Sampling-based method suitable for complex spaces; finds feasible but often suboptimal paths and is computationally intensive.

Algorithm Comparison Table
^^^^^^^^^^^^^^^^^^^^^^^^^

All three algorithms were tested in a controlled environment with the following parameters:

+-------------+----------------------+------------------------+------------------+
| Algorithm   | Path Length (m)      | Computation Time (ms) | Nodes Explored    |
+=============+======================+========================+==================+
| A*          | 23.54–27.88          | 24.18–493.01           | 341–461          |
+-------------+----------------------+------------------------+------------------+
| Dijkstra    | 21.92–23.75          | 306.67–1035.63         | 341–413          |
+-------------+----------------------+------------------------+------------------+
| RRT         | 22.46–35.17          | 3207.36–6275.46        | 23–32            |
+-------------+----------------------+------------------------+------------------+

A* offers the best balance for this application, with Dijkstra being slower and RRT less optimal for path quality

Python API
-------

Classes
^^^^^^^

Each algorithm is implemented as a ROS node class:

- **AStarNode**: Handles path planning using the A* algorithm.
- **DijkstraNode**: Implements Dijkstra’s algorithm for shortest path computation.
- **RRTNode**: Executes the RRT algorithm for sampling-based path planning.

These classes inherit from the ROS node base class and interact with ROS topics and services for receiving map data, publishing planned paths, and responding to navigation requests.

Functions
^^^^^^^

The following functions are defined within the ROS node classes to facilitate the autonomous navigation process:
- **plan_path(start, goal, map)**: Computes the path from start to goal.
- **update_map(sensor_data)**: Updates the occupancy grid using LiDAR and odometry.
- **publish_path(path)**: Publishes the computed path to a ROS topic.
- **handle_signal_loss()**: Switches to autonomous mode during communication loss.


Python Examples
-----------------

A* Algorithm
^^^^^^^^^^^^

.. code-block:: python

    import heapq
    def astar(start, goal, grid):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return reconstruct_path(came_from, current)
            for neighbor in get_neighbors(current, grid):
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
    return None

Dijkstra Algorithm
^^^^^^^^^^^^^^^^^^

.. code-block:: python

    import heapq
    def dijkstra(start, goal, grid):
        queue = []
        heapq.heappush(queue, (0, start))
        distances = {start: 0}
        came_from = {}
        while queue:
            dist, current = heapq.heappop(queue)
            if current == goal:
                return reconstruct_path(came_from, current)
            for neighbor in get_neighbors(current, grid):
                new_dist = dist + 1
                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    came_from[neighbor] = current
                    heapq.heappush(queue, (new_dist, neighbor))
    return None

RRT Algorithm
^^^^^^^^^^^^^

.. code-block:: python

    import random
    def rrt(start, goal, grid, max_iter=1000):
        tree = {start: None}
        for _ in range(max_iter):
            rand_point = random_point(grid)
            nearest = nearest_node(rand_point, tree)
            new_point = steer(nearest, rand_point)
            if is_free(new_point, grid):
                tree[new_point] = nearest
                if distance(new_point, goal) < threshold:
                    tree[goal] = new_point
                    return reconstruct_path(tree, goal)
    return None

These examples illustrate the core logic of each algorithm, focusing on pathfinding and grid navigation. The actual implementation in the ROS nodes includes additional functionality for integration with the robot's sensors and actuators.