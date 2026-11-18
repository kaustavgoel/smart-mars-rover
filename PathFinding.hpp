#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include "Types.hpp"
#include "MarsElevationData.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

struct PathResult {
    CustomVector<Point2D> path;
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
    float gCost;
    float fCost;
    PathNode* parent;

    PathNode(int r, int c, float g, float f, PathNode* p = nullptr)
        : row(r), col(c), gCost(g), fCost(f), parent(p) {}
};


class PathMinHeap {
private:
    PathNode* heap[TOTAL_ROWS * TOTAL_COLUMNS * 4];
    int sizeCount;

    void bubbleUp(int idx) {
        while (idx > 0) {
            int p = (idx - 1) / 2;
            if (heap[idx]->fCost < heap[p]->fCost) {
                customSwap(heap[idx], heap[p]);
                idx = p;
            } else {
                break;
            }
        }
    }

    void bubbleDown(int idx) {
        while (2 * idx + 1 < sizeCount) {
            int left = 2 * idx + 1;
            int right = 2 * idx + 2;
            int smallest = idx;

            if (left < sizeCount && heap[left]->fCost < heap[smallest]->fCost) {
                smallest = left;
            }
            if (right < sizeCount && heap[right]->fCost < heap[smallest]->fCost) {
                smallest = right;
            }
            if (smallest != idx) {
                customSwap(heap[idx], heap[smallest]);
                idx = smallest;
            } else {
                break;
            }
        }
    }

public:
    PathMinHeap() : sizeCount(0) {}

    void push(PathNode* node) {
        heap[sizeCount] = node;
        bubbleUp(sizeCount);
        sizeCount++;
    }

    PathNode* top() const {
        return heap[0];
    }

    void pop() {
        if (sizeCount > 0) {
            heap[0] = heap[sizeCount - 1];
            sizeCount--;
            if (sizeCount > 0) bubbleDown(0);
        }
    }

    bool empty() const {
        return sizeCount == 0;
    }
};


class NavigationSystem {
public:
    static float calculateHeuristic(int r1, int c1, int r2, int c2) {

        return static_cast<float>(std::abs(r1 - r2) + std::abs(c1 - c2));
    }

    static PathResult findPathAStar(const MarsTerrainGrid& terrain, Point2D start, Point2D goal) {
        auto startTime = std::chrono::high_resolution_clock::now();
        PathResult result;

        if (!terrain.isPassable(start.getRow(), start.getColumn()) ||
            !terrain.isPassable(goal.getRow(), goal.getColumn())) {
            result.pathFound = false;
            return result;
        }

        float distanceTo[TOTAL_ROWS][TOTAL_COLUMNS];
        for (int r = 0; r < TOTAL_ROWS; ++r) {
            for (int c = 0; c < TOTAL_COLUMNS; ++c) {
                distanceTo[r][c] = 1e9f;
            }
        }

        PathNode* allocatedNodes[TOTAL_ROWS * TOTAL_COLUMNS * 4];
        int allocatedCount = 0;

        PathMinHeap openQueue;
        float h0 = calculateHeuristic(start.getRow(), start.getColumn(), goal.getRow(), goal.getColumn());
        PathNode* startNode = new PathNode(start.getRow(), start.getColumn(), 0.0f, h0, nullptr);
        allocatedNodes[allocatedCount++] = startNode;
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
                        allocatedNodes[allocatedCount++] = neighbor;
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

            int pSize = static_cast<int>(result.path.size());
            for (int i = 0; i < pSize / 2; ++i) {
                customSwap(result.path[i], result.path[pSize - 1 - i]);
            }
        } else {
            result.pathFound = false;
        }

        for (int i = 0; i < allocatedCount; ++i) {
            delete allocatedNodes[i];
        }

        return result;
    }

    static PathResult findPathDijkstra(const MarsTerrainGrid& terrain, Point2D start, Point2D goal) {
        auto startTime = std::chrono::high_resolution_clock::now();
        PathResult result;

        if (!terrain.isPassable(start.getRow(), start.getColumn()) ||
            !terrain.isPassable(goal.getRow(), goal.getColumn())) {
            result.pathFound = false;
            return result;
        }

        float distanceTo[TOTAL_ROWS][TOTAL_COLUMNS];
        for (int r = 0; r < TOTAL_ROWS; ++r) {
            for (int c = 0; c < TOTAL_COLUMNS; ++c) {
                distanceTo[r][c] = 1e9f;
            }
        }

        PathNode* allocatedNodes[TOTAL_ROWS * TOTAL_COLUMNS * 4];
        int allocatedCount = 0;

        PathMinHeap openQueue;
        PathNode* startNode = new PathNode(start.getRow(), start.getColumn(), 0.0f, 0.0f, nullptr);
        allocatedNodes[allocatedCount++] = startNode;
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
                        PathNode* neighbor = new PathNode(nr, nc, newG, newG, current);
                        allocatedNodes[allocatedCount++] = neighbor;
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
            int pSize = static_cast<int>(result.path.size());
            for (int i = 0; i < pSize / 2; ++i) {
                customSwap(result.path[i], result.path[pSize - 1 - i]);
            }
        } else {
            result.pathFound = false;
        }

        for (int i = 0; i < allocatedCount; ++i) {
            delete allocatedNodes[i];
        }

        return result;
    }
};

#endif
