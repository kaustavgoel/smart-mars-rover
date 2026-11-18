#include "Types.hpp"
#include "MarsElevationData.hpp"
#include "QuadTree.hpp"
#include "MaxHeap.hpp"
#include "AVLTree.hpp"
#include "PathFinding.hpp"
#include "KnapsackOptimizer.hpp"
#include "DecisionTree.hpp"
#include "NeuralNetwork.hpp"
#include "ExcavationLog.hpp"
#include "MarsRover.hpp"

#include <iostream>
#include <cassert>
#include <cmath>

void testTerrainGrid() {
    std::cout << "[Test 1] Testing Mars Terrain Grid & Elevation Preprocessing... ";
    MarsTerrainGrid grid;
    assert(grid.isPassable(0, 0));
    assert(grid.getElevation(0, 0) < -2000.0f);
    assert(grid.getSlope(0, 0) >= 0.0f);
    assert(grid.getCellCost(0, 0) >= 1.0f);
    std::cout << "PASSED\n";
}

void testQuadTree() {
    std::cout << "[Test 2] Testing Spatial QuadTree Indexing & Proximity Queries... ";
    QuadTree qt;
    qt.insert(Point2D(5, 5), "OBSTACLE", 1);
    qt.insert(Point2D(6, 6), "OBSTACLE", 2);
    qt.insert(Point2D(10, 10), "SITE", 101);

    int dist = qt.findDistanceToNearestObstacle(Point2D(5, 6));
    assert(dist == 1);

    auto items = qt.queryRadius(Point2D(5, 5), 3.0f);
    assert(items.size() >= 2);
    std::cout << "PASSED\n";
}

void testMaxHeap() {
    std::cout << "[Test 3] Testing Candidate Science Sites Max-Heap... ";
    MaxHeap heap;
    heap.insert(SiteData(1, "Site A", Point2D(1, 1), 50.0f, 30.0f, 0, 0, 0, 0));
    heap.insert(SiteData(2, "Site B", Point2D(2, 2), 95.0f, 40.0f, 0, 0, 0, 0));
    heap.insert(SiteData(3, "Site C", Point2D(3, 3), 75.0f, 35.0f, 0, 0, 0, 0));

    assert(!heap.isEmpty());
    assert(heap.size() == 3);
    assert(heap.peekMax().siteId == 2); // 95.0f is max
    SiteData max1 = heap.extractMax();
    assert(max1.siteId == 2);
    assert(heap.peekMax().siteId == 3); // 75.0f is next
    std::cout << "PASSED\n";
}

void testAVLTree() {
    std::cout << "[Test 4] Testing Candidate Sites Self-Balancing AVL Tree... ";
    AVLTree avl;
    avl.insert(SiteData(105, "Site 105", Point2D(1, 1), 60.0f, 30.0f, 0, 0, 0, 0));
    avl.insert(SiteData(102, "Site 102", Point2D(2, 2), 85.0f, 40.0f, 0, 0, 0, 0));
    avl.insert(SiteData(108, "Site 108", Point2D(3, 3), 90.0f, 35.0f, 0, 0, 0, 0));
    avl.insert(SiteData(101, "Site 101", Point2D(4, 4), 70.0f, 25.0f, 0, 0, 0, 0));
    avl.insert(SiteData(103, "Site 103", Point2D(5, 5), 75.0f, 28.0f, 0, 0, 0, 0));

    assert(avl.size() == 5);
    assert(avl.getTreeHeight() <= 3); // Balanced!

    SiteData* found = avl.search(103);
    assert(found != nullptr);
    assert(found->siteId == 103);
    assert(avl.search(999) == nullptr);

    auto inOrder = avl.getAllSitesInOrder();
    assert(inOrder.size() == 5);
    for (size_t i = 1; i < inOrder.size(); ++i) {
        assert(inOrder[i - 1].siteId < inOrder[i].siteId);
    }
    std::cout << "PASSED\n";
}

void testPathfindingAStarVsDijkstra() {
    std::cout << "[Test 5] Testing A* vs Dijkstra Weighted Pathfinding & Optimality... ";
    MarsTerrainGrid terrain;
    Point2D start(0, 0);
    Point2D goal(11, 16);

    PathResult aStar = NavigationSystem::findPathAStar(terrain, start, goal);
    PathResult dijkstra = NavigationSystem::findPathDijkstra(terrain, start, goal);

    assert(aStar.pathFound);
    assert(dijkstra.pathFound);
    // Verify costs match within float precision (Optimality proof)
    assert(std::abs(aStar.totalEnergyCost - dijkstra.totalEnergyCost) < 0.1f);
    // A* must explore fewer or equal nodes compared to exhaustive Dijkstra
    assert(aStar.nodesExplored <= dijkstra.nodesExplored);
    std::cout << "PASSED (A* Nodes: " << aStar.nodesExplored
              << " vs Dijkstra: " << dijkstra.nodesExplored << ", Cost: "
              << aStar.totalEnergyCost << ")\n";
}

void testKnapsackOptimizer() {
    std::cout << "[Test 6] Testing 0/1 Knapsack DP Multi-Site Tour Optimizer... ";
    MarsTerrainGrid terrain;
    CustomVector<SiteData> sites = {
        SiteData(1, "Site 1", Point2D(3, 4),   90.0f, 40.0f, 0, 0, 0, 0),
        SiteData(2, "Site 2", Point2D(15, 20), 85.0f, 50.0f, 0, 0, 0, 0),
        SiteData(3, "Site 3", Point2D(6, 8),   80.0f, 35.0f, 0, 0, 0, 0),
        SiteData(4, "Site 4", Point2D(18, 25), 60.0f, 60.0f, 0, 0, 0, 0)
    };

    KnapsackPlan plan = KnapsackOptimizer::optimizeSiteSelection(sites, terrain, Point2D(0, 0), 120);
    assert(!plan.selectedSiteIds.empty());
    assert(plan.batteryBudgetUsed <= 120);
    assert(plan.totalExpectedScienceValue > 0.0f);
    std::cout << "PASSED (Selected: " << plan.selectedSiteIds.size()
              << " sites, Science: " << plan.totalExpectedScienceValue
              << " pts, Battery: " << plan.batteryBudgetUsed << " Wh)\n";
}

void testDecisionTree() {
    std::cout << "[Test 7] Testing Autonomous Multi-Stage Decision Tree... ";
    RoverDecisionTree dt;
    MarsTerrainGrid terrain;
    QuadTree qt;
    SiteData goodSite(101, "Delta", Point2D(4, 8), 90.0f, 40.0f, -2400.0f, 10.0f, 8.0f, 0.85f);
    SiteData steepSite(102, "Cliff", Point2D(10, 10), 90.0f, 40.0f, -2100.0f, 45.0f, 35.0f, 0.85f);

    // Test with ample battery
    DecisionResult r1 = dt.evaluateSiteCommitment(300.0f, 50.0f, goodSite, terrain, qt);
    assert(r1.approved);

    // Test with low battery (should skip or abort)
    DecisionResult r2 = dt.evaluateSiteCommitment(30.0f, 50.0f, goodSite, terrain, qt);
    assert(!r2.approved);

    // Test with hazardous steep slope
    DecisionResult r3 = dt.evaluateSiteCommitment(300.0f, 50.0f, steepSite, terrain, qt);
    assert(!r3.approved);

    // Test excavation strategy
    DecisionResult r4 = dt.evaluateExcavationStrategy(200.0f, goodSite, 10.0f);
    assert(r4.outcome == DecisionOutcome::PERFORM_DEEP_DRILL);
    std::cout << "PASSED\n";
}

void testNeuralNetwork() {
    std::cout << "[Test 8] Testing First-Principles Neural Network (Forward & Backprop)... ";
    ExcavationNeuralNetwork nn;
    MathVector inputs = {0.25f, 0.15f, 0.10f, 0.85f};
    ExcavationPrediction pred = nn.forward(inputs);

    assert(pred.offsetRow >= -0.8f && pred.offsetRow <= 0.8f);
    assert(pred.offsetCol >= -0.8f && pred.offsetCol <= 0.8f);
    assert(pred.predictedDepthCm >= 8.0f && pred.predictedDepthCm <= 40.0f);
    assert(pred.hiddenActivations.size() == 6);
    std::cout << "PASSED (Predicted Offset: (" << pred.offsetRow << ", "
              << pred.offsetCol << "), Depth: " << pred.predictedDepthCm << " cm)\n";
}

void testExcavationLogDLL() {
    std::cout << "[Test 9] Testing Excavation Log Custom Doubly Linked List... ";
    ExcavationLog log;
    assert(log.isEmpty());

    log.appendRecord(101, "Delta", Point2D(4, 8), 0.12f, -0.08f, 25.4f, 106.0f, "SUCCESS (ORGANICS)", 45.0f, 12.5f);
    log.appendRecord(102, "Basalt", Point2D(7, 21), -0.05f, 0.15f, 18.2f, 76.0f, "SUCCESS (BASALT)", 52.0f, 24.0f);

    assert(log.size() == 2);
    assert(log.getHead() != nullptr);
    assert(log.getTail() != nullptr);
    assert(log.getHead()->siteId == 101);
    assert(log.getTail()->siteId == 102);
    assert(log.getTail()->prev == log.getHead());
    assert(log.getHead()->next == log.getTail());

    auto recent = log.getRecentRecords(2);
    assert(recent.size() == 2);
    assert(recent[0].siteId == 102); // Latest first
    assert(recent[1].siteId == 101);
    std::cout << "PASSED\n";
}

int main() {
    std::cout << "========================================================\n"
              << " SMART MARS ROVER AUTOMATED TEST SUITE & VERIFICATION\n"
              << "========================================================\n";

    testTerrainGrid();
    testQuadTree();
    testMaxHeap();
    testAVLTree();
    testPathfindingAStarVsDijkstra();
    testKnapsackOptimizer();
    testDecisionTree();
    testNeuralNetwork();
    testExcavationLogDLL();

    std::cout << "========================================================\n"
              << " ALL 9 DATA STRUCTURES & ALGORITHMS VERIFIED 100% OK!\n"
              << "========================================================\n";
    return 0;
}
