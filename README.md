# 3D Path Planners for Unmanned Aerial Vehicles (UAVs)

![image](https://github.com/markusbuchholz/uav-blue-robotics-path-planner/assets/30973337/914d4576-66b8-4f81-bf5b-c764bf8e3b4c)

## Introduction
This repository dedicated to simulating various 3D path planners for Unmanned Aerial Vehicles (UAVs). The primary intention behind this repository is to provide a comprehensive understanding and implementation of different path planning algorithms in a 3D space, specifically tailored for UAVs.

The [Orca4](https://github.com/clydemcqueen/orca4) repository has been used to copy the simulator and all necessary files. Modifications have been made to the Dockerfile, including the installation of the [PlotJuggler](https://github.com/facontidavide/PlotJuggler) plotting tool and nano. The obstacle visualization used by the path planner in Gazebo is located in the ```sand.word``` file.

As part of the UVA mission, the path planners have been tested by running a program that computes obstacle-free 3D paths with multiple waypoints. However, the UAV control system receives only a reduced number of waypoints since the controller is not tested or tuned.

To test the performance of the path planners outside of the simulator environment, the program can be run from the ```core_path_planners``` folder.

## Simplified review of path planners for UAVs

Unmanned Surface Vehicles (USVs) are autonomous or remotely operated boats used for a variety of applications, such as surveillance, environmental monitoring, and hydrographic surveying. A path planner is an algorithmic solution designed to find the most optimal path for a mobile agent (like a robot or vehicle) from a starting point to a destination, considering various constraints and objectives. 
Path planning for USVs is crucial to ensure safe and efficient navigation, especially in complex and dynamic environments. 

### Tasks of a Path Planner:

Environment Representation: Transform the planning space into a geometric space, discretizing the entire space into multiple sub-spaces.
Obstacle Avoidance: Identify and navigate around static and dynamic obstacles in the environment.
Optimal Path Identification: Find the shortest, safest, or most energy-efficient path based on the given criteria.
Dynamic Re-planning: Adjust the path in real-time based on changes in the environment or vehicle status.
Consideration of Vehicle Constraints: Account for the physical and performance constraints of the vehicle, such as speed, turning radius, and dynamics.

### Parameters for Optimal Performance:

Configuration Space (C-space): Represents the robot as a point, encompassing all possible configurations, including position and orientation.
Sampling Methods: Techniques like probabilistic or deterministic sampling of the C-space to avoid exhaustive construction of obstacles.
Metric Space: Defines the distance between two configurations in the C-space, crucial for determining proximity between states.
Environment Modeling: Techniques like the Visibility Graph, Voronoi Diagram, and Grid Map to represent the environment.
Planning Behavior and Constraints: Actions restrained by the environment and the dynamic performance of the vehicle.
Planning Criterion: Desired constraint condition based on planning space and behavior, focusing on feasibility or optimality.

### Challenges for Path Planners:

For Unmanned Surface Vehicles (USVs) and Autonomous Underwater Vehicles (AUVs):

* Dynamic Environment: Real-time path planning in complex and dynamic environments.
* Natural Elements: Influence of wind, waves, and currents on the vehicle's trajectory.
* Communication: Low-bandwidth communication channels in underwater environments.
* Vehicle Constraints: Nonholonomic constraints, like the inability to move directly sideways, affect movement prediction.
* Large-Scale Challenges: Weather changes, tsunamis, and typhoons for large-scale areas.
* Medium-Scale Challenges: Avoidance of both static and dynamic obstacles.
* Small-Scale Challenges: Steering stability, wind, wave, and current effects, and prevention of vehicle drift.


### Here are some common types of path planners used for USVs:

#### Geometric/Grid-Based Path Planning Algorithms:

* Specifics: These algorithms either focus on the geometric properties of the environment or discretize the environment into a grid. In the grid-based approach, each cell in the grid is classified as either free or occupied, and the algorithms search this grid to find a path.

* Differences from other groups: They provide a structured way to represent the environment, either through direct geometric representations or through grids. They don't typically rely on bio-inspired or machine learning techniques.

* Strengths: They can be straightforward and deterministic, providing solutions that are often optimal in terms of path length or other criteria.

* Example: RRT, Dijkstra, A*, D*-lite, Probabilistic Road Maps algorithms

#### Intelligent Path Planning Algorithms:

* Specifics: These algorithms leverage machine learning and other intelligent techniques. They can adapt and learn from the environment, making them suitable for dynamic and unpredictable scenarios.

* Differences from other groups: They can adapt and learn, making them suitable for dynamic environments. They might require more computational resources and training data.

* Strengths: Strong adaptability to the environment, especially in complex dynamic environments. They can improve over time with more data.

* Example: Particle Swarm Optimization (PSO), Ant Colony Optimization(ACO), Genetic algorithm (GA), Fireﬂy Algorithm(FA).

#### Reinforcement Learning in Path Planning:

```Reinforcement learning (RL)``` is a type of machine learning where an agent learns to make decisions by taking actions in an environment to maximize a reward. In the context of path planning, the agent is typically the AUV, the environment is the space in which the vehicle operates (water), and the reward is related to successfully navigating from a start to a goal without collisions, while possibly minimizing time or energy.

##### Details:

RL is suitable for path planning in complex and unknown dynamic environments. Unlike supervised learning, RL learns control strategies from interactions between the system and the environment. For instance, an AUV selects an action, the environment changes state, and a reward signal is generated, guiding the AUV's next action.
RL does not require prior knowledge and is real-time, efficient, and fast when solving path planning problems. However, challenges include convergence speed and balancing exploration and exploitation.
Deep Reinforcement Learning (DRL) combines the perception capabilities of deep learning with the decision-making abilities of RL. It can directly control a vehicle's motion based on input images to solve path planning problems.
DRL has been applied to AUV obstacle avoidance and navigation to optimize search paths.
Despite its potential, DRL can have disadvantages such as long training times and overfitting to specific environments.

##### Advantages:
 - Adaptability: RL can adapt to changing environments, making it suitable for dynamic scenarios.
 - No Need for Prior Knowledge: RL algorithms can learn optimal strategies without prior knowledge of the environment.
 - Real-time Decision Making: RL can make real-time decisions based on the current state of the environment.
 - Complex Environment Handling: DRL can handle complex environments by leveraging deep learning's perception capabilities.

##### Challenges:
 - Convergence Speed: Ensuring that the RL algorithm converges to an optimal policy in a reasonable amount of time can be challenging.
 - Balance of Exploration and Exploitation: Striking the right balance between exploring new paths and exploiting known paths is crucial.
- Training Time and Overfitting: DRL models can take a long time to train and might overfit to specific training environments.

##### Examples:
The Sarsa (λ) algorithm, a type of RL algorithm, was used for AUV path planning to reduce the cost of removing sea urchins.
DRL, combined with the asynchronous advantage actor-critic (A3C) network structure, was used to enable AUVs to explore unknown environments.


### Conclusion:

Path planning is a multifaceted domain with various approaches tailored to specific scenarios. While Geometric/Grid-Based methods provide structured ways to navigate known environments, Intelligent methods offer adaptability and learning capabilities for more complex scenarios. Reinforcement Learning methods, on the other hand, learn from interactions with the environment, making them suitable for dynamic and unpredictable situations. The choice of method would depend on the specific needs of the task and the environment in which the vehicle operates.

## AUV Control system

To ensure stable movement of UAV during the mission, the system control system gathers several measurements from sensors like Sonar, Doppler Velocity Logs,  Underwater Cameras, Inertial Navigation Systems, Barometers, etc deployed on UAV gathers data about the voyage, environment and surroundings. This data is processed by a SLAM component, which provides the UAV's current position and status to an MPC controller (for example). The below figure illustrates the software components integral to the UAV's control loop. 
Path planning plays a pivotal role in the advancement of Unmanned Surface Vehicles (USVs). Its primary objective is to utilize algorithms to chart optimal trajectories, guiding a vessel's journey. Essentially, path planning is the process of determining a collision-free and physically viable route between two points in a mobile space. This route should also meet specific optimization criteria. Common criteria include minimizing path length, time, energy consumption, and ensuring safety or risk measures.

While path planning generally operates within a geometric space, trajectory planning or generation extends to include temporal properties, such as dynamics. 

To clarify, we can derive. common terms,
```Path Planning```: Focuses on creating a geometric path by identifying a series of waypoints to navigate from a starting point to a destination.

```Path Smoothing```: This process refines a sequence of waypoints. It optimally connects them while considering the vessel's limited curvature or turning radius, resulting in a streamlined path.

```Trajectory Generation```: This is an extension of path planning. It not only considers the geometric path but also factors in constraints like turning angles, velocity, and acceleration. It assigns a temporal aspect (time law) to the geometric path, making it more dynamic.

The trajectory generation module is tasked with creating the reference trajectory for UAV. The MPC controller generates an optimal input, ensuring the UAV adheres to the reference trajectory within set constraints.Concluding the control loop is the localization module, which derives state estimations of the robot from sensor readings. 

![image](https://github.com/markusbuchholz/uav-blue-robotics-path-planners/assets/30973337/8e74d105-4e97-4f62-bd81-2167c0d2df69)


## Path planners in the simulator

Currently, there are four path planners implemented in the simulator (Geometric and Grid-Based Path Planning Algorithms). Additionally, all of them can be run independently from the simulator.
Here is a simple visualization,


### RRT (Rapidly Exploring Random Tree) path planner,

![image](https://github.com/markusbuchholz/uav-blue-robotics-path-planners/assets/30973337/db939701-5475-4f4a-8295-2fd7afcb095c)

### Dijkstra path planner,

![image](https://github.com/markusbuchholz/uav-blue-robotics-path-planners/assets/30973337/85d7c70e-5ae2-472d-ab93-727286dc818e)


### PRM (Probabilistic roadmap) path planner,

![image](https://github.com/markusbuchholz/uav-blue-robotics-path-planners/assets/30973337/3f53d6ae-cec2-4d98-b614-7e2c5514e240)

### A* path planner,
![image](https://github.com/markusbuchholz/uav-blue-robotics-path-planners/assets/30973337/7bb65625-8bdf-44ef-aed2-d776c50960cd)


### Usage

For complete installation visit [Orca4](https://github.com/clydemcqueen/orca4).

For running simulator in ```Docker```,

```bash
git clone 
cd 
./build.sh
```

run Docker container (you can modify the repository, but you have to rebuild),
```bash
./run.sh
```

inside container run simulator,
```bash
ros2 launch orca_bringup sim_launch.py
```

in othe terminal run the mission (path planner),
```bash
docker exec -it orca4 /bin/bash
ros2 run orca_bringup rrt_mission_runner.py
```

you can also plot the data (topic),
```bash
docker exec -it orca4 /bin/bash
ros2 run plotjuggler plotjuggler
```





