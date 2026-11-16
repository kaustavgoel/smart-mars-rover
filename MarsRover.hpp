#ifndef MARS_ROVER_HPP
#define MARS_ROVER_HPP

#include "Types.hpp"
#include "MarsElevationData.hpp"
#include "QuadTree.hpp"
#include "NeuralNetwork.hpp"
#include "ExcavationLog.hpp"
#include <deque>
#include <vector>
#include <string>
#include <iostream>

/**
 * @brief Autonomous Mars Rover Vehicle with Battery Subsystem, Smooth Motion, and Mission Executive.
 */
class MarsRoverVehicle {
private:
    int currentRowPosition;
    int currentColumnPosition;
    float drawingRowPosition;
    float drawingColumnPosition;

    std::vector<Point2D> currentPlannedPath;
    int currentPathStepIndex;
    std::deque<Point2D> movementTrail;

    int distanceToNearestObstacle;
    RoverState currentStatus;
    int totalNumberOfMovesMade;

    // Enhanced Battery System
    float batteryLevel;
    float maxBatteryLevel;
    float totalEnergyConsumed;

    // Multi-Site Mission Plan
    std::vector<int> plannedSiteTour; // List of site IDs in order
    size_t currentTourIndex;
    int activeTargetSiteId;

    // Excavation and Science Metrics
    float accumulatedScienceScore;
    float accumulatedSampleMassGrams;

    // Drilling animation timer
    float drillTimer;
    bool isDrillingActive;

public:
    MarsRoverVehicle(float initialBattery = 350.0f)
        : currentRowPosition(0), currentColumnPosition(0),
          drawingRowPosition(0.0f), drawingColumnPosition(0.0f),
          currentPathStepIndex(0), distanceToNearestObstacle(99),
          currentStatus(RoverState::WAITING_IDLE), totalNumberOfMovesMade(0),
          batteryLevel(initialBattery), maxBatteryLevel(initialBattery),
          totalEnergyConsumed(0.0f), currentTourIndex(0), activeTargetSiteId(-1),
          accumulatedScienceScore(0.0f), accumulatedSampleMassGrams(0.0f),
          drillTimer(0.0f), isDrillingActive(false) {}

    void resetRover() {
        currentRowPosition = 0;
        currentColumnPosition = 0;
        drawingRowPosition = 0.0f;
        drawingColumnPosition = 0.0f;
        currentPlannedPath.clear();
        movementTrail.clear();
        currentPathStepIndex = 0;
        distanceToNearestObstacle = 99;
        currentStatus = RoverState::WAITING_IDLE;
        totalNumberOfMovesMade = 0;
        batteryLevel = maxBatteryLevel;
        totalEnergyConsumed = 0.0f;
        plannedSiteTour.clear();
        currentTourIndex = 0;
        activeTargetSiteId = -1;
        accumulatedScienceScore = 0.0f;
        accumulatedSampleMassGrams = 0.0f;
        drillTimer = 0.0f;
        isDrillingActive = false;
    }

    void assignNewPath(const std::vector<Point2D>& newPath) {
        currentPlannedPath = newPath;
        currentPathStepIndex = 0;
        if (newPath.empty()) {
            currentStatus = RoverState::PATH_IS_STUCK;
        } else {
            currentStatus = RoverState::AUTOMATIC_NAVIGATING;
        }
    }

    void setPlannedTour(const std::vector<int>& tour) {
        plannedSiteTour = tour;
        currentTourIndex = 0;
        if (!plannedSiteTour.empty()) {
            activeTargetSiteId = plannedSiteTour[0];
        } else {
            activeTargetSiteId = -1;
        }
    }

    int getNextTourSiteId() {
        if (currentTourIndex < plannedSiteTour.size()) {
            activeTargetSiteId = plannedSiteTour[currentTourIndex];
            return activeTargetSiteId;
        }
        activeTargetSiteId = -1;
        return -1;
    }

    void advanceTour() {
        currentTourIndex++;
        if (currentTourIndex < plannedSiteTour.size()) {
            activeTargetSiteId = plannedSiteTour[currentTourIndex];
        } else {
            activeTargetSiteId = -1;
            currentStatus = RoverState::SUCCESSFULLY_ARRIVED;
        }
    }

    bool executeNextAutomaticStep(const MarsTerrainGrid& terrain) {
        if (batteryLevel <= 0.0f) {
            currentStatus = RoverState::BATTERY_DEPLETED;
            return false;
        }

        int totalPathLength = static_cast<int>(currentPlannedPath.size());
        if (currentPathStepIndex < totalPathLength) {
            addCurrentPositionToTrail();
            Point2D nextPoint = currentPlannedPath[currentPathStepIndex];

            // Calculate energy consumption for moving to next cell
            float energyCost = terrain.getEdgeTraversalEnergy(
                Point2D(currentRowPosition, currentColumnPosition), nextPoint
            );

            batteryLevel = std::max(0.0f, batteryLevel - energyCost);
            totalEnergyConsumed += energyCost;

            currentRowPosition = nextPoint.getRow();
            currentColumnPosition = nextPoint.getColumn();
            currentPathStepIndex++;
            totalNumberOfMovesMade++;

            if (currentPathStepIndex >= totalPathLength) {
                // Arrived at current target waypoint
                return false;
            }
            return true;
        }
        return false;
    }

    void moveManually(int dr, int dc, const MarsTerrainGrid& terrain) {
        if (batteryLevel <= 0.0f) {
            currentStatus = RoverState::BATTERY_DEPLETED;
            return;
        }

        int nextR = currentRowPosition + dr;
        int nextC = currentColumnPosition + dc;

        if (terrain.isPassable(nextR, nextC)) {
            addCurrentPositionToTrail();
            float energyCost = terrain.getEdgeTraversalEnergy(
                Point2D(currentRowPosition, currentColumnPosition), Point2D(nextR, nextC)
            );

            batteryLevel = std::max(0.0f, batteryLevel - energyCost);
            totalEnergyConsumed += energyCost;

            currentRowPosition = nextR;
            currentColumnPosition = nextC;
            totalNumberOfMovesMade++;
            currentStatus = RoverState::MANUALLY_EXPLORING;
        }
    }

    void deductExcavationBattery(float cost) {
        batteryLevel = std::max(0.0f, batteryLevel - cost);
        totalEnergyConsumed += cost;
    }

    void addScienceYield(float scienceVal, float sampleGrams) {
        accumulatedScienceScore += scienceVal;
        accumulatedSampleMassGrams += sampleGrams;
    }

    void updateAnimation(float deltaTime) {
        constexpr float animationSpeed = 12.0f;
        drawingRowPosition += (static_cast<float>(currentRowPosition) - drawingRowPosition) * animationSpeed * deltaTime;
        drawingColumnPosition += (static_cast<float>(currentColumnPosition) - drawingColumnPosition) * animationSpeed * deltaTime;
    }

    void addCurrentPositionToTrail() {
        movementTrail.push_back(Point2D(currentRowPosition, currentColumnPosition));
        if (static_cast<int>(movementTrail.size()) > MAXIMUM_TRAIL_LENGTH) {
            movementTrail.pop_front();
        }
    }

    // Proximity color based on distance to nearest obstacle
    sf::Color getRoverProximityColor() const {
        if (distanceToNearestObstacle <= 1) {
            return COLOR_DANGER;
        } else if (distanceToNearestObstacle <= 3) {
            return COLOR_WARNING;
        } else {
            return COLOR_SAFE;
        }
    }

    std::string getStatusString() const {
        switch (currentStatus) {
            case RoverState::WAITING_IDLE: return "IDLE / STANDBY";
            case RoverState::PLANNING_MISSION: return "MISSION PLANNING";
            case RoverState::AUTOMATIC_NAVIGATING: return "AUTO-NAVIGATING (A*)";
            case RoverState::EXCAVATING_SITE: return "EXCAVATING DRILL ACTIVE";
            case RoverState::SUCCESSFULLY_ARRIVED: return "MISSION TOUR COMPLETE";
            case RoverState::BATTERY_DEPLETED: return "BATTERY DEPLETED";
            case RoverState::PATH_IS_STUCK: return "PATH IS BLOCKED";
            case RoverState::MANUALLY_EXPLORING: return "MANUAL CONTROL";
            default: return "UNKNOWN";
        }
    }

    // Getters
    int getCurrentRow() const { return currentRowPosition; }
    int getCurrentColumn() const { return currentColumnPosition; }
    Point2D getPosition() const { return Point2D(currentRowPosition, currentColumnPosition); }
    float getDrawingRow() const { return drawingRowPosition; }
    float getDrawingColumn() const { return drawingColumnPosition; }
    const std::vector<Point2D>& getCurrentPath() const { return currentPlannedPath; }
    int getPathStepIndex() const { return currentPathStepIndex; }
    const std::deque<Point2D>& getTrail() const { return movementTrail; }

    int getObstacleDistance() const { return distanceToNearestObstacle; }
    void setObstacleDistance(int d) { distanceToNearestObstacle = d; }

    RoverState getStatus() const { return currentStatus; }
    void setStatus(RoverState s) { currentStatus = s; }

    float getBattery() const { return batteryLevel; }
    float getMaxBattery() const { return maxBatteryLevel; }
    float getBatteryPercentage() const { return (batteryLevel / maxBatteryLevel) * 100.0f; }
    float getTotalEnergyConsumed() const { return totalEnergyConsumed; }

    int getActiveTargetSiteId() const { return activeTargetSiteId; }
    void setActiveTargetSiteId(int id) { activeTargetSiteId = id; }
    size_t getCurrentTourIndex() const { return currentTourIndex; }
    const std::vector<int>& getPlannedTour() const { return plannedSiteTour; }

    float getAccumulatedScience() const { return accumulatedScienceScore; }
    float getAccumulatedSampleMass() const { return accumulatedSampleMassGrams; }
    int getTotalMoves() const { return totalNumberOfMovesMade; }
};

#endif // MARS_ROVER_HPP
