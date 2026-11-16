#ifndef QUADTREE_HPP
#define QUADTREE_HPP

#include "Types.hpp"
#include <vector>
#include <memory>
#include <cmath>
#include <climits>
#include <iostream>

struct SpatialItem {
    Point2D coord;
    std::string type; // "OBSTACLE", "SITE", "ROVER"
    int id;           // Site ID or index

    SpatialItem() : coord(0, 0), type(""), id(0) {}
    SpatialItem(Point2D c, const std::string& t, int i = 0) : coord(c), type(t), id(i) {}
};

struct BoundingBox {
    float x;      // Column offset in grid space
    float y;      // Row offset in grid space
    float width;  // Columns wide
    float height; // Rows high

    BoundingBox() : x(0), y(0), width(0), height(0) {}
    BoundingBox(float _x, float _y, float _w, float _h) : x(_x), y(_y), width(_w), height(_h) {}

    bool contains(Point2D p) const {
        return (p.getColumn() >= x && p.getColumn() < x + width &&
                p.getRow() >= y && p.getRow() < y + height);
    }

    bool intersects(const BoundingBox& other) const {
        return !(other.x >= x + width ||
                 other.x + other.width <= x ||
                 other.y >= y + height ||
                 other.y + other.height <= y);
    }
};

/**
 * @brief Custom 2D QuadTree spatial index for fast O(log N) proximity and range queries.
 * Eliminates exhaustive O(N) map scans when checking hazards and nearest candidate sites.
 */
class QuadTreeNode {
public:
    BoundingBox boundary;
    int capacity;
    std::vector<SpatialItem> items;
    bool isDivided;

    QuadTreeNode* northWest;
    QuadTreeNode* northEast;
    QuadTreeNode* southWest;
    QuadTreeNode* southEast;

    QuadTreeNode(BoundingBox b, int cap = 4)
        : boundary(b), capacity(cap), isDivided(false),
          northWest(nullptr), northEast(nullptr),
          southWest(nullptr), southEast(nullptr) {}

    ~QuadTreeNode() {
        delete northWest;
        delete northEast;
        delete southWest;
        delete southEast;
    }

    void subdivide() {
        float halfW = boundary.width / 2.0f;
        float halfH = boundary.height / 2.0f;
        float x = boundary.x;
        float y = boundary.y;

        northWest = new QuadTreeNode(BoundingBox(x, y, halfW, halfH), capacity);
        northEast = new QuadTreeNode(BoundingBox(x + halfW, y, halfW, halfH), capacity);
        southWest = new QuadTreeNode(BoundingBox(x, y + halfH, halfW, halfH), capacity);
        southEast = new QuadTreeNode(BoundingBox(x + halfW, y + halfH, halfW, halfH), capacity);

        isDivided = true;

        // Re-distribute existing items into appropriate quadrants
        std::vector<SpatialItem> currentItems = items;
        items.clear();
        for (const auto& item : currentItems) {
            insert(item);
        }
    }

    bool insert(const SpatialItem& item) {
        if (!boundary.contains(item.coord)) {
            return false;
        }

        if (items.size() < static_cast<size_t>(capacity) && !isDivided) {
            items.push_back(item);
            return true;
        }

        if (!isDivided) {
            subdivide();
        }

        if (northWest->insert(item)) return true;
        if (northEast->insert(item)) return true;
        if (southWest->insert(item)) return true;
        if (southEast->insert(item)) return true;

        return false;
    }

    void queryRange(const BoundingBox& range, std::vector<SpatialItem>& found) const {
        if (!boundary.intersects(range)) {
            return;
        }

        for (const auto& item : items) {
            if (range.contains(item.coord)) {
                found.push_back(item);
            }
        }

        if (isDivided) {
            northWest->queryRange(range, found);
            northEast->queryRange(range, found);
            southWest->queryRange(range, found);
            southEast->queryRange(range, found);
        }
    }

    void collectAllBoxes(std::vector<BoundingBox>& boxes) const {
        boxes.push_back(boundary);
        if (isDivided) {
            northWest->collectAllBoxes(boxes);
            northEast->collectAllBoxes(boxes);
            southWest->collectAllBoxes(boxes);
            southEast->collectAllBoxes(boxes);
        }
    }
};

class QuadTree {
private:
    std::unique_ptr<QuadTreeNode> root;
    int totalItemsIndexed;

public:
    QuadTree() : totalItemsIndexed(0) {
        clear();
    }

    void clear() {
        BoundingBox universe(0.0f, 0.0f, static_cast<float>(TOTAL_COLUMNS), static_cast<float>(TOTAL_ROWS));
        root = std::make_unique<QuadTreeNode>(universe, 4);
        totalItemsIndexed = 0;
    }

    bool insert(Point2D coord, const std::string& type, int id = 0) {
        if (root->insert(SpatialItem(coord, type, id))) {
            totalItemsIndexed++;
            return true;
        }
        return false;
    }

    std::vector<SpatialItem> queryRange(float x, float y, float w, float h) const {
        std::vector<SpatialItem> results;
        BoundingBox range(x, y, w, h);
        root->queryRange(range, results);
        return results;
    }

    std::vector<SpatialItem> queryRadius(Point2D center, float radius) const {
        float x = static_cast<float>(center.getColumn()) - radius;
        float y = static_cast<float>(center.getRow()) - radius;
        float size = radius * 2.0f;
        std::vector<SpatialItem> candidates = queryRange(x, y, size, size);

        std::vector<SpatialItem> filtered;
        for (const auto& item : candidates) {
            if (center.distanceTo(item.coord) <= radius) {
                filtered.push_back(item);
            }
        }
        return filtered;
    }

    /**
     * @brief High-efficiency nearest obstacle lookup using QuadTree expanding search box.
     */
    int findDistanceToNearestObstacle(Point2D roverPos, float maxSearchRadius = 6.0f) const {
        float searchRad = 2.0f;
        while (searchRad <= maxSearchRadius + 2.0f) {
            auto nearby = queryRadius(roverPos, searchRad);
            int minManhattan = 99;
            bool foundObstacle = false;

            for (const auto& item : nearby) {
                if (item.type == "OBSTACLE") {
                    int d = roverPos.manhattanDistance(item.coord);
                    if (d < minManhattan) {
                        minManhattan = d;
                        foundObstacle = true;
                    }
                }
            }

            if (foundObstacle) {
                return minManhattan;
            }
            searchRad += 2.0f;
        }
        return 99;
    }

    std::vector<BoundingBox> getAllBoundingBoxes() const {
        std::vector<BoundingBox> boxes;
        if (root) {
            root->collectAllBoxes(boxes);
        }
        return boxes;
    }

    int getItemCount() const { return totalItemsIndexed; }
};

#endif // QUADTREE_HPP
