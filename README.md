# Smart Mars Rover: Multi-Site Navigation & Assisted Excavation Planning

An autonomous Martian rover simulation designed and implemented in **C++17** using **SFML 3**. The system combines graph theory, self-balancing search trees, spatial partitioning trees, dynamic programming, and a first-principles neural network to autonomously explore, navigate, and sample scientific targets on Mars under finite battery constraints.

Developed for **B.Tech III Semester**, Jaypee Institute of Information Technology, Noida.

---

## Team Members & Responsibilities

* **Kaustav Goel** (2501200039): NASA DEM Terrain Preprocessing ($O(N)$), Weighted Graph model, and Map reset functionality.
* **Keshav Narayan Gupta** (2501200060): Mars Rover Vehicle state machine, battery management system, movement physics, trail rendering, and animation.
* **Navya Chaddha** (2501200038): SFML User Interface HUD, visual telemetry dashboard, color coding, and documentation.
* **Algorithm & DSA Implementation**: Max-Heap, AVL Tree, 2D QuadTree, 0/1 Knapsack DP, Decision Tree, Neural Network, Doubly Linked List, and A* vs Dijkstra verification.

---

## Core Data Structures & Algorithms

1. **NASA DEM Terrain Preprocessing ($O(N)$)**:
   - Sourced from Martian elevation data (Jezero Crater DEM).
   - Classifies terrain into Plains, Moderate Slopes, Rough Regolith, and Obstacles based on elevation gradients and local roughness.
2. **Weighted Graph & Dual Pathfinding**:
   - Traversal costs model distance, surface friction, and slope incline penalties.
   - **A\*** (heuristic-guided) and **Dijkstra** (exhaustive baseline) run side-by-side to empirically validate path optimality.
3. **Dynamic Priority Ranking (Max-Heap)**:
   - Live max-heap prioritizing candidate scientific targets by scientific value return.
4. **Permanent Cataloging (AVL Tree)**:
   - Self-balancing binary search tree with rotations (LL, RR, LR, RL) providing guaranteed $O(\log N)$ search and insertion.
5. **Spatial Indexing (2D QuadTree)**:
   - Replaces full-grid scans with $O(\log N)$ proximity searches for obstacles and candidate sites.
   - Includes interactive bounding-box overlay.
6. **0/1 Knapsack Dynamic Programming Optimizer**:
   - Solves the optimal subset of candidate sites to visit within the rover's finite battery budget ($420\text{ Wh}$).
7. **Autonomous Decision Tree**:
   - Evaluates Travel, Terrain Risk, Science efficiency, and Substrate drilling viability (`PROCEED`, `SKIP`, or `ABORT`).
8. **First-Principles Neural Network (No ML Libraries)**:
   - Built from scratch using custom `MathVector` and `MathMatrix` classes with backpropagation gradient descent.
   - Predicts sub-cell coordinate offset and core sample dig depth from terrain features.
9. **Excavation Log (Doubly Linked List)**:
   - Bidirectional linked list permanently logging every excavation: site ID, predicted offset, depth, sample yield, battery spent, and outcome status.

---

## Project Structure

```
smart_mars_rover/
├── Types.hpp                    # Core types, Point2D template, and palette
├── MarsElevationData.hpp        # NASA Jezero Crater DEM & O(N) classifier
├── QuadTree.hpp                 # 2D Spatial QuadTree for O(log N) proximity
├── MaxHeap.hpp                  # Custom Max-Heap for live site ranking
├── AVLTree.hpp                  # Custom self-balancing AVL Tree
├── PathFinding.hpp              # A* and Dijkstra pathfinding
├── KnapsackOptimizer.hpp        # 0/1 Knapsack DP battery optimizer
├── DecisionTree.hpp             # Autonomous multi-tier Decision Tree
├── NeuralNetwork.hpp            # First-principles Neural Network & backprop
├── ExcavationLog.hpp            # Custom Doubly Linked List excavation log
├── MarsRover.hpp                # Rover entity with battery and motion physics
├── HUD.hpp                      # SFML Heads-Up Display dashboard
├── main.cpp                     # Simulation loop & keyboard controls
├── Makefile                     # Build script
├── CMakeLists.txt               # CMake build configuration
├── test_suite.cpp               # Automated verification test suite
└── smart_mars_rover_all_in_one.cpp # Single-file consolidated edition
```

---

## How to Build and Run

### Prerequisites
* C++17 compatible compiler (`clang++` or `g++`)
* [SFML 3](https://www.sfml-dev.org/) (e.g. `brew install sfml` on macOS)

### Option 1: Modular Build (Recommended)
```bash
make
./smart_mars_rover
```

### Option 2: Single-File Build
```bash
clang++ -std=c++17 -O3 -I/opt/homebrew/include smart_mars_rover_all_in_one.cpp \
  -L/opt/homebrew/lib -lsfml-graphics -lsfml-window -lsfml-system -o smart_mars_rover_single
./smart_mars_rover_single
```

### Option 3: Run the Verification Test Suite
```bash
clang++ -std=c++17 -O3 -I/opt/homebrew/include -I. test_suite.cpp \
  -L/opt/homebrew/lib -lsfml-graphics -lsfml-window -lsfml-system -o test_suite
./test_suite
```

---

## Keyboard Controls

| Key | Action |
| :--- | :--- |
| **`[Arrow Keys]`** | Move Rover manually (tap for single step, hold for continuous drive) |
| **`[M]`** | Toggle between **Manual Control** and **Auto Mission Tour** (A*) |
| **`[Q]`** | Toggle **Spatial QuadTree Bounding-Box Overlay** |
| **`[D]`** | Toggle **Dijkstra Baseline Path Overlay** |
| **`[Space]`** | Advance one step forward in Auto Mode |
| **`[B]`** | Recharge Rover battery back to 100% |
| **`[P]`** | Pause / Resume simulation time |
| **`[R]`** | Reset Rover, battery, candidate sites, and logs to mission start |

---

## Academic References

* Cormen, T. H., Leiserson, C. E., Rivest, R. L., & Stein, C. — *Introduction to Algorithms*, MIT Press.

* Adelson-Velsky, G. M., & Landis, E. M. — *An Algorithm for the Organization of Information* (Original AVL Tree).

* NASA / USGS — Mars elevation/terrain datasets for the DEM/heightmap data.
