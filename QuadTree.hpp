#ifndef QUADTREE_HPP
#define QUADTREE_HPP

#include "Types.hpp"
#include <string>
#include <cmath>

struct SpatialItem {
    Point2D coord;
    std::string type; // "OBSTACLE", "SITE"
    int id;

    SpatialItem() : coord(0, 0), type(""), id(0) {}
    SpatialItem(Point2D c, const std::string& t, int i = 0) : coord(c), type(t), id(i) {}
};

struct BoundingBox {
    float x;
    float y;
    float width;
    float height;

    BoundingBox() : x(0), y(0), width(0), height(0) {}
    BoundingBox(float bx, float by, float bw, float bh) : x(bx), y(by), width(bw), height(bh) {}

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

// single node in quadtree holding up to 4 items before splitting
class QuadTreeNode {
public:
    BoundingBox boundary;
    SpatialItem items[4];
    int itemCount;
    bool isDivided;

    QuadTreeNode* northWest;
    QuadTreeNode* northEast;
    QuadTreeNode* southWest;
    QuadTreeNode* southEast;

    QuadTreeNode(BoundingBox b)
        : boundary(b), itemCount(0), isDivided(false),
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

        northWest = new QuadTreeNode(BoundingBox(x, y, halfW, halfH));
        northEast = new QuadTreeNode(BoundingBox(x + halfW, y, halfW, halfH));
        southWest = new QuadTreeNode(BoundingBox(x, y + halfH, halfW, halfH));
        southEast = new QuadTreeNode(BoundingBox(x + halfW, y + halfH, halfW, halfH));

        isDivided = true;

        // redistribute current items to children
        for (int i = 0; i < itemCount; ++i) {
            insert(items[i]);
        }
        itemCount = 0;
    }

    bool insert(const SpatialItem& item) {
        if (!boundary.contains(item.coord)) {
            return false;
        }

        if (itemCount < 4 && !isDivided) {
            items[itemCount++] = item;
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

    void queryRange(const BoundingBox& range, CustomVector<SpatialItem>& found) const {
        if (!boundary.intersects(range)) return;

        for (int i = 0; i < itemCount; ++i) {
            if (range.contains(items[i].coord)) {
                found.push_back(items[i]);
            }
        }

        if (isDivided) {
            northWest->queryRange(range, found);
            northEast->queryRange(range, found);
            southWest->queryRange(range, found);
            southEast->queryRange(range, found);
        }
    }

    void collectAllBoxes(CustomVector<BoundingBox>& boxes) const {
        boxes.push_back(boundary);
        if (isDivided) {
            northWest->collectAllBoxes(boxes);
            northEast->collectAllBoxes(boxes);
            southWest->collectAllBoxes(boxes);
            southEast->collectAllBoxes(boxes);
        }
    }
};

// 2d quadtree for fast spatial range and proximity queries
class QuadTree {
private:
    QuadTreeNode* root;
    int totalItemsIndexed;

public:
    QuadTree() : root(nullptr), totalItemsIndexed(0) {
        clear();
    }

    ~QuadTree() {
        delete root;
    }

    void clear() {
        delete root;
        BoundingBox universe(0.0f, 0.0f, static_cast<float>(TOTAL_COLUMNS), static_cast<float>(TOTAL_ROWS));
        root = new QuadTreeNode(universe);
        totalItemsIndexed = 0;
    }

    bool insert(Point2D coord, const std::string& type, int id = 0) {
        if (root->insert(SpatialItem(coord, type, id))) {
            totalItemsIndexed++;
            return true;
        }
        return false;
    }

    CustomVector<SpatialItem> queryRange(float x, float y, float w, float h) const {
        CustomVector<SpatialItem> results;
        BoundingBox range(x, y, w, h);
        if (root) root->queryRange(range, results);
        return results;
    }

    CustomVector<SpatialItem> queryRadius(Point2D center, float radius) const {
        float x = static_cast<float>(center.getColumn()) - radius;
        float y = static_cast<float>(center.getRow()) - radius;
        float size = radius * 2.0f;
        CustomVector<SpatialItem> candidates = queryRange(x, y, size, size);

        CustomVector<SpatialItem> filtered;
        for (const auto& item : candidates) {
            if (center.distanceTo(item.coord) <= radius) {
                filtered.push_back(item);
            }
        }
        return filtered;
    }

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
            if (foundObstacle) return minManhattan;
            searchRad += 2.0f;
        }
        return 99;
    }

    CustomVector<BoundingBox> getAllBoundingBoxes() const {
        CustomVector<BoundingBox> boxes;
        if (root) root->collectAllBoxes(boxes);
        return boxes;
    }

    int getItemCount() const { return totalItemsIndexed; }
};

#endif // QUADTREE_HPP
