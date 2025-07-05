Autonomous Navigation of Mobile Robots
=====
Introduction
------------
The robot is designed to transport heavy loads within CERN radiation facilities, where human presence is hazardous due to high radiation levels. Previously, robots required continuous manual control and monitoring, making it impossible for operators to take breaks and leading to inefficiency and safety concerns. Autonomous navigation enables the robot to operate independently during signal loss or operator absence, improving safety, efficiency, and allowing short operator breaks.

CERN Facility
^^^^^^^^^^^^
The robot operates in a radiation facility at CERN, where human presence is limited due to safety concerns. The environment is characterized by high radiation levels, which necessitates the use of autonomous systems for tasks that would otherwise require human intervention. Continuous monitoring is required due to radiation risks. Signal loss is common due to facility structure, necessitating autonomous fallback. The robot must navigate complex layouts, avoid obstacles, and maintain operational reliability 

Robot Description
^^^^^^^^^^^^^^^

The key components of the robot which are relevant for this project include:

- **LiDAR Sensor**: Used for real-time environment scanning and mapping.
- **Wheel Odometry**:Provides accurate movement tracking, essential for localization and navigation, especially in signal-degraded environments.
- **Custom D-Bot Platform**: Initial testing and development were performed on a custom-built D-Bot at Aalto University, with the intention to transfer the solution to CERN’s main robot.

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

All three algorithms were tested in a controlled simulated environment using Gazebo for different 2D occupancy maps which results in following parameters while navigation from Point A to Point B i.e. from start to end:

.. for table to remain centered, different code structure should be used.

+-------------+----------------------+------------------------+------------------+
| Algorithm   | Path Length (m)      | Computation Time (ms) | Nodes Explored    |
+=============+======================+========================+==================+
| A*          | 23.54–27.88          | 24.18–493.01           | 341–461          |
+-------------+----------------------+------------------------+------------------+
| Dijkstra    | 21.92–23.75          | 306.67–1035.63         | 341–413          |
+-------------+----------------------+------------------------+------------------+
| RRT         | 22.46–35.17          | 3207.36–6275.46        | 23–32            |
+-------------+----------------------+------------------------+------------------+

A* offers the best balance for this application, with Dijkstra being slower and RRT less optimal for path quality which evident from the table above.

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

.. no spaces between the headings and the text results in error in the documentation.

The following functions are defined within the ROS node classes to facilitate the autonomous navigation process:

- **plan_path(start, goal, map)**: Computes the path from start to goal.
- **update_map(sensor_data)**: Updates the occupancy grid using LiDAR and odometry.
- **publish_path(path)**: Publishes the computed path to a ROS topic.
- **handle_signal_loss()**: Switches to autonomous mode during communication loss.


Python Examples
-----------------

A* Algorithm
^^^^^^^^^^^^

.. code-block:: none

   function A_Star(startNode, goalNode, heuristicFunction)
      nodesToExplore = {startNode}
      bestPathMap = {}
      costFromStart = {startNode: 0}
      estimatedTotalCost = {startNode: heuristicFunction(startNode)}  // f = g + h

      while nodesToExplore is not empty:
         currentNode = node in nodesToExplore with lowest estimatedTotalCost value

         if currentNode == goalNode:
               return reconstruct_path(bestPathMap, currentNode)  // Path found!

         nodesToExplore.remove(currentNode)

         for each neighborNode of currentNode:
               pathCost = costFromStart[currentNode] + distance(currentNode, neighborNode)

               if pathCost < costFromStart.get(neighborNode, Infinity):
                  bestPathMap[neighborNode] = currentNode
                  costFromStart[neighborNode] = pathCost
                  estimatedTotalCost[neighborNode] = pathCost + heuristicFunction(neighborNode)

                  if neighborNode not in nodesToExplore:
                     nodesToExplore.add(neighborNode)


Dijkstra Algorithm
^^^^^^^^^^^^^^^^^^
.. ^^^^^ this was not completely unde the dijkstra section resulting in error and no further codes are displayed.
.. code-block:: none

   function Dijkstra(startNode, goalNode, distanceFunction)
      nodesToExplore = {startNode}         // Set of nodes to be evaluated
      bestPathMap = {}                     // Maps each node to its best previous node
      costFromStart = {startNode: 0}       // Tracks shortest distance from startNode

      while nodesToExplore is not empty:
         currentNode = node in nodesToExplore with lowest costFromStart value

         if currentNode == goalNode:
               return reconstruct_path(bestPathMap, currentNode)  // Path found!

         nodesToExplore.remove(currentNode)

         for each neighborNode of currentNode:
               pathCost = costFromStart[currentNode] + distanceFunction(currentNode, neighborNode)

               if pathCost < costFromStart.get(neighborNode, Infinity):
                  bestPathMap[neighborNode] = currentNode     // Update best path
                  costFromStart[neighborNode] = pathCost

                  if neighborNode not in nodesToExplore:
                     nodesToExplore.add(neighborNode)

      return failure  // No path found
   function reconstruct_path(bestPathMap, goalNode)
      shortestPath = [goalNode]
      while goalNode in bestPathMap:
         goalNode = bestPathMap[goalNode]
         shortestPath.prepend(goalNode)  // Add previous node to the path
      return shortestPath

RRT Algorithm
^^^^^^^^^^^^^

.. code-block:: none

   function RRT(startNode, goalNode, maxIterations, stepSize, obstacleChecker)
      tree = {startNode}         // Initialize tree with start node
      pathFound = false

      for i = 1 to maxIterations:
         randomNode = generateRandomNode()
         nearestNode = findNearestNode(tree, randomNode)
         newNode = extendTowards(nearestNode, randomNode, stepSize)

         if not obstacleChecker(newNode):
               continue  // Skip if node is invalid (collision)

         tree.add(newNode)

         if distance(newNode, goalNode) < stepSize:
               tree.add(goalNode)
               pathFound = true
               break

      if pathFound:
         return reconstruct_path(tree, goalNode)
      else:
         return failure  // No path found
   function reconstruct_path(tree, goalNode)
      path = [goalNode]
      currentNode = goalNode

      while currentNode in tree:
         currentNode = findParentNode(tree, currentNode)
         path.prepend(currentNode)
      return path


These examples illustrate the core logic of each algorithm, focusing on pathfinding and grid navigation. The actual implementation in the ROS nodes includes additional functionality for integration with the robot's sensors and actuators.