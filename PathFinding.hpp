#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include "Types.hpp"
#include "MarsElevationData.hpp"
#include <vector>
#include <queue>
#include <cmath>
#include <chrono>
#include <climits>
#include <algorithm>
#include <iostream>

struct PathResult {
    std::vector<Point2D> path;
    float totalEnergyCost;
    int nodesExplored;
    long long executionTimeMicroseconds;
    bool pathFound;

    PathResult()
        : totalEnergyCost(0.0f), nodesExplored(0),
          executionTimeMicroseconds(0), pathFound(false) {}
};

struct PathNode {
    int row;
    int col;
    float gCost; // Actual cost from start to current
    float fCost; // gCost + hCost (for A*) or gCost (for Dijkstra)
    PathNode* parent;

    PathNode(int r, int c, float g, float f, PathNode* p = nullptr)
        : row(r), col(c), gCost(g), fCost(f), parent(p) {}
};

struct PathNodeComparator {
    bool operator()(const PathNode* a, const PathNode* b) const {
        return a->fCost > b->fCost; // Min-heap behavior
    }
};

/**
 * @brief Autonomous Navigation System implementing both A* and Dijkstra's algorithms.
 * Allows empirical validation that heuristic-guided A* matches Dijkstra's true optimal path
 * while exploring substantially fewer grid nodes.
 */
class NavigationSystem {
public:
    static float calculateHeuristic(int r1, int c1, int r2, int c2) {
        // Manhattan distance multiplied by minimum possible terrain traversal cost (1.0)
        // Admissible heuristic: never overestimates true cost.
        return static_cast<float>(std::abs(r1 - r2) + std::abs(c1 - c2)) * 1.0f;
    }

    /**
     * @brief A* Heuristic-Guided Pathfinding on Weighted Terrain Graph.
     */
    static PathResult findPathAStar(const MarsTerrainGrid& terrain, Point2D start, Point2D goal) {
        auto startTime = std::chrono::high_resolution_clock::now();
        PathResult result;

        if (!terrain.isPassable(start.getRow(), start.getColumn()) ||
            !terrain.isPassable(goal.getRow(), goal.getColumn())) {
            result.pathFound = false;
            return result;
        }

        std::vector<std::vector<float>> distanceTo(TOTAL_ROWS, std::vector<float>(TOTAL_COLUMNS, 1e9f));
        std::vector<PathNode*> allocatedNodes;

        std::priority_queue<PathNode*, std::vector<PathNode*>, PathNodeComparator> openQueue;

        float h0 = calculateHeuristic(start.getRow(), start.getColumn(), goal.getRow(), goal.getColumn());
        PathNode* startNode = new PathNode(start.getRow(), start.getColumn(), 0.0f, h0, nullptr);
        allocatedNodes.push_back(startNode);
        openQueue.push(startNode);
        distanceTo[start.getRow()][start.getColumn()] = 0.0f;

        const int dr[4] = {-1, 1, 0, 0};
        const int dc[4] = {0, 0, -1, 1};

        PathNode* targetNode = nullptr;
        int exploredCount = 0;

        while (!openQueue.empty()) {
            PathNode* current = openQueue.top();
            openQueue.pop();
            exploredCount++;

            int cr = current->row;
            int cc = current->col;

            if (cr == goal.getRow() && cc == goal.getColumn()) {
                targetNode = current;
                break;
            }

            if (current->gCost > distanceTo[cr][cc]) {
                continue;
            }

            for (int i = 0; i < 4; ++i) {
                int nr = cr + dr[i];
                int nc = cc + dc[i];

                if (terrain.isPassable(nr, nc)) {
                    float edgeCost = terrain.getEdgeTraversalEnergy(Point2D(cr, cc), Point2D(nr, nc));
                    float newG = current->gCost + edgeCost;

                    if (newG < distanceTo[nr][nc]) {
                        distanceTo[nr][nc] = newG;
                        float h = calculateHeuristic(nr, nc, goal.getRow(), goal.getColumn());
                        PathNode* neighbor = new PathNode(nr, nc, newG, newG + h, current);
                        allocatedNodes.push_back(neighbor);
                        openQueue.push(neighbor);
                    }
                }
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        result.executionTimeMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
        result.nodesExplored = exploredCount;

        if (targetNode != nullptr) {
            result.pathFound = true;
            result.totalEnergyCost = targetNode->gCost;

            PathNode* curr = targetNode;
            while (curr != nullptr) {
                result.path.push_back(Point2D(curr->row, curr->col));
                curr = curr->parent;
            }
            std::reverse(result.path.begin(), result.path.end());
        } else {
            result.pathFound = false;
        }

        // Cleanup memory
        for (auto* node : allocatedNodes) {
            delete node;
        }

        return result;
    }

    /**
     * @brief Exhaustive Baseline: Dijkstra's Algorithm (Heuristic = 0.0).
     */
    static PathResult findPathDijkstra(const MarsTerrainGrid& terrain, Point2D start, Point2D goal) {
        auto startTime = std::chrono::high_resolution_clock::now();
        PathResult result;

        if (!terrain.isPassable(start.getRow(), start.getColumn()) ||
            !terrain.isPassable(goal.getRow(), goal.getColumn())) {
            result.pathFound = false;
            return result;
        }

        std::vector<std::vector<float>> distanceTo(TOTAL_ROWS, std::vector<float>(TOTAL_COLUMNS, 1e9f));
        std::vector<PathNode*> allocatedNodes;

        std::priority_queue<PathNode*, std::vector<PathNode*>, PathNodeComparator> openQueue;

        PathNode* startNode = new PathNode(start.getRow(), start.getColumn(), 0.0f, 0.0f, nullptr);
        allocatedNodes.push_back(startNode);
        openQueue.push(startNode);
        distanceTo[start.getRow()][start.getColumn()] = 0.0f;

        const int dr[4] = {-1, 1, 0, 0};
        const int dc[4] = {0, 0, -1, 1};

        PathNode* targetNode = nullptr;
        int exploredCount = 0;

        while (!openQueue.empty()) {
            PathNode* current = openQueue.top();
            openQueue.pop();
            exploredCount++;

            int cr = current->row;
            int cc = current->col;

            if (cr == goal.getRow() && cc == goal.getColumn()) {
                targetNode = current;
                break;
            }

            if (current->gCost > distanceTo[cr][cc]) {
                continue;
            }

            for (int i = 0; i < 4; ++i) {
                int nr = cr + dr[i];
                int nc = cc + dc[i];

                if (terrain.isPassable(nr, nc)) {
                    float edgeCost = terrain.getEdgeTraversalEnergy(Point2D(cr, cc), Point2D(nr, nc));
                    float newG = current->gCost + edgeCost;

                    if (newG < distanceTo[nr][nc]) {
                        distanceTo[nr][nc] = newG;
                        // For Dijkstra, fCost = gCost (no heuristic)
                        PathNode* neighbor = new PathNode(nr, nc, newG, newG, current);
                        allocatedNodes.push_back(neighbor);
                        openQueue.push(neighbor);
                    }
                }
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        result.executionTimeMicroseconds = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
        result.nodesExplored = exploredCount;

        if (targetNode != nullptr) {
            result.pathFound = true;
            result.totalEnergyCost = targetNode->gCost;

            PathNode* curr = targetNode;
            while (curr != nullptr) {
                result.path.push_back(Point2D(curr->row, curr->col));
                curr = curr->parent;
            }
            std::reverse(result.path.begin(), result.path.end());
        } else {
            result.pathFound = false;
        }

        // Cleanup memory
        for (auto* node : allocatedNodes) {
            delete node;
        }

        return result;
    }
};

#endif // PATHFINDING_HPP
