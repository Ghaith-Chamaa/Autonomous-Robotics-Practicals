# Autonomous Robotics Course Practicals

This repository contains the code, solutions, and documentation for the practical labs completed as part of the Autonomous Robotics course. The labs cover fundamental concepts in robotics, from deterministic and sampling-based path planning to motion planning with kinematic constraints.

## 📜 Table of Contents

*   [General Setup & Prerequisites](#-general-setup--prerequisites)
*   [Practicals](#-practicals)
    *   [Practical 1: A* Path Planning on Grid and Voronoi Maps](#-practical-1-a-path-planning-on-grid-and-voronoi-maps)
    *   [Practical 2: Sampling-Based Planners - PRM vs. RRT](#-practical-2-sampling-based-planners---prm-vs-rrt)
    *   [Practical 3: RRT with Kinematic Constraints (Bicycle Model)](#-practical-3-rrt-with-kinematic-constraints-bicycle-model)
*   [How to Use This Repository](#-how-to-use-this-repository)

## 🛠️ General Setup & Prerequisites

All practicals in this repository were developed using **MATLAB**. To run the simulations, you will need:

*   **MATLAB (R2021b or newer recommended):** The core environment for running the scripts.
*   **MATLAB Navigation Toolbox™:** Required for Practical 2 and 3, as it provides built-in functions for `plannerRRT`, `plannerPRM`, `plannerControlRRT`, `occupancyMap`, and other essential robotics tools.

## 🔬 Practicals

This section provides a detailed breakdown of each lab, including its objectives, key concepts, implementation details.

---

### 📍 Practical 1: A* Path Planning on Grid and Voronoi Maps

This practical explores the fundamental path planning algorithm, A*, and its application to two different environment representations: a discrete occupancy grid and a continuous graph based on a Voronoi diagram.

#### 🎯 Objective

The goal is to implement the A* algorithm in MATLAB for a holonomic robot in two scenarios:
1.  **Grid-based Representation:** Find the shortest path in a 2D matrix where cells are either free or occupied.
2.  **Voronoi-based Representation:** Generate a path that maximizes the distance from obstacles by building a graph from a Voronoi diagram of the free space.

#### 💡 Key Concepts
*   A* Search Algorithm
*   Heuristics (Euclidean Distance)
*   Occupancy Grids
*   Voronoi Diagrams
*   Graph-based Path Planning

#### 📂 Files & Scripts
*   `astar_grid.m`: The core A* function for grid-based searches.
*   `astar_voronoi.m`: The core A* function for Voronoi graph searches.
*   `plot_paths_grid.m`: Main script to define a grid map, run the grid-based A*, and visualize the result.
*   `plot_paths_voronoi.m`: Main script to define obstacles, run the Voronoi-based A*, and visualize the result.

#### 📊 Results and Analysis

The two methods were tested on several map configurations, the results are shown in the report.

**Summary:** Grid-based A\* is optimal for finding the shortest possible path, while Voronoi-based A\* is superior for finding safer paths with much lower computational effort, making it ideal for real-time applications in environments with sparse obstacles.

---

### 📍 Practical 2: Sampling-Based Planners - PRM vs. RRT

This practical delves into sampling-based motion planning, a powerful technique for navigating complex spaces. It focuses on implementing and comparing two seminal algorithms: the Probabilistic Roadmap (PRM) and the Rapidly-exploring Random Tree (RRT).

#### 🎯 Objective

The objective is to understand, implement, and compare the performance of PRM and RRT using MATLAB's Navigation Toolbox. The comparison is performed across different environments to analyze their strengths and weaknesses.

#### 💡 Key Concepts
*   Sampling-Based Motion Planning
*   Probabilistic Roadmap (PRM)
*   Rapidly-exploring Random Tree (RRT)
*   State Space (SE2)
*   Collision Checking

#### 📂 Files & Scripts
*   `planning.m`: Sets up a **simple map** and runs both PRM and RRT, visualizing the results side-by-side.
*   `cluttered.m`: Sets up a **highly cluttered map** to stress-test the planners.

#### 📊 Results and Analysis

Two primary test cases were evaluated: a simple map with sparse obstacles and a cluttered map, the results are shown in the report.

**Summary:** RRT consistently found shorter, more efficient paths than PRM in the conducted tests. This is because RRT's focused, goal-biased exploration allowed it to find a solution more directly, while PRM's uniform sampling struggled to place nodes effectively in narrow passages or create an optimal roadmap with a limited node count.

---

### 📍 Practical 3: RRT with Kinematic Constraints (Bicycle Model)

This practical extends motion planning by introducing non-holonomic constraints. While previous planners assumed a robot that can move in any direction, this lab focuses on generating feasible, drivable paths for a car-like robot using a **Bicycle Kinematic Model**.

#### 🎯 Objective

*   To understand and implement the bicycle model for a non-holonomic, front-steered vehicle.
*   To use a kinematically-aware RRT planner to generate smooth, drivable paths.
*   To compare the performance and path quality of a kinematically-constrained RRT against a standard (holonomic) RRT.

#### 💡 Key Concepts
*   Non-Holonomic Constraints
*   Bicycle Kinematic Model
*   State Propagation
*   Feasible Path vs. Geometric Path

#### 📂 Files & Scripts
*   `Bicycle_model.m`: A standalone script to demonstrate the behavior of the bicycle model with various parameters.
*   `normal.m`: Compares the performance of Standard RRT vs. Kinematic RRT on a **simple map**.
*   `cluttered.m`: Compares the planners on a **highly cluttered map**.

#### 📊 Results and Analysis

The planners were tested on a simple and a cluttered map, revealing a critical trade-off between path feasibility and computational cost, the results are shown in the report.

**Summary:** This practical powerfully demonstrates that a geometrically "optimal" path is often useless if it doesn't respect the robot's physical constraints.
*   **Standard RRT** is fast but produces unrealistic paths with sharp turns impossible for a car to execute.
*   **Kinematic RRT** generates truly feasible, smooth paths but at a massive computational expense. It struggled and failed to find a solution in the cluttered map, highlighting that navigating tight spaces with turning constraints is a significantly harder problem.

## 🚀 How to Use This Repository

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/Ghaith-Chamaa/Autonomous-Robotics-Practicals.git
    ```
2.  **Organize Files:** The files for each practical are in separate folders.
3.  **Run in MATLAB:**
    *   Open MATLAB.
    *   Navigate to the directory containing the desired practical's scripts.
    *   Run the main scripts (e.g., `plot_paths_grid.m`, `planning.m`, `normal.m`) from the MATLAB Command Window or Editor.
    *   Modify parameters within these main scripts to experiment with different maps, start/goal positions, and planner settings.
