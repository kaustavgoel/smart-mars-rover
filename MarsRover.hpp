#ifndef MARS_ROVER_HPP
#define MARS_ROVER_HPP

#include "Types.hpp"
#include "MarsElevationData.hpp"
#include <string>
#include <iostream>


class MarsRoverVehicle {
private:
    int currentRowPosition;
    int currentColumnPosition;
    float drawingRowPosition;
    float drawingColumnPosition;

    CustomVector<Point2D> currentPlannedPath;
    int currentPathStepIndex;


    Point2D trailBuffer[MAXIMUM_TRAIL_LENGTH];
    int trailStart;
    int trailCount;

    int distanceToNearestObstacle;
    RoverState currentStatus;
    int totalNumberOfMovesMade;

    float batteryLevel;
    float maxBatteryLevel;
    float totalEnergyConsumed;

    CustomVector<int> plannedSiteTour;
    size_t currentTourIndex;
    int activeTargetSiteId;

    float accumulatedScienceScore;
    float accumulatedSampleMassGrams;

public:
    MarsRoverVehicle(float initialBattery = 350.0f)
        : currentRowPosition(0), currentColumnPosition(0),
          drawingRowPosition(0.0f), drawingColumnPosition(0.0f),
          currentPathStepIndex(0), trailStart(0), trailCount(0),
          distanceToNearestObstacle(99), currentStatus(RoverState::WAITING_IDLE),
          totalNumberOfMovesMade(0), batteryLevel(initialBattery),
          maxBatteryLevel(initialBattery), totalEnergyConsumed(0.0f),
          currentTourIndex(0), activeTargetSiteId(-1),
          accumulatedScienceScore(0.0f), accumulatedSampleMassGrams(0.0f) {}

    void resetRover() {
        currentRowPosition = 0;
        currentColumnPosition = 0;
        drawingRowPosition = 0.0f;
        drawingColumnPosition = 0.0f;
        currentPlannedPath.clear();
        trailStart = 0;
        trailCount = 0;
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
    }

    void assignNewPath(const CustomVector<Point2D>& newPath) {
        currentPlannedPath = newPath;
        currentPathStepIndex = 0;
        if (newPath.empty()) {
            currentStatus = RoverState::PATH_IS_STUCK;
        } else {
            currentStatus = RoverState::AUTOMATIC_NAVIGATING;
        }
    }

    void setPlannedTour(const CustomVector<int>& tour) {
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

            float energyCost = terrain.getEdgeTraversalEnergy(
                Point2D(currentRowPosition, currentColumnPosition), nextPoint
            );

            batteryLevel = findMax(0.0f, batteryLevel - energyCost);
            totalEnergyConsumed += energyCost;

            currentRowPosition = nextPoint.getRow();
            currentColumnPosition = nextPoint.getColumn();
            currentPathStepIndex++;
            totalNumberOfMovesMade++;

            if (currentPathStepIndex >= totalPathLength) {
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

            batteryLevel = findMax(0.0f, batteryLevel - energyCost);
            totalEnergyConsumed += energyCost;

            currentRowPosition = nextR;
            currentColumnPosition = nextC;
            totalNumberOfMovesMade++;
            currentStatus = RoverState::MANUALLY_EXPLORING;
        }
    }

    void deductExcavationBattery(float cost) {
        batteryLevel = findMax(0.0f, batteryLevel - cost);
        totalEnergyConsumed += cost;
    }

    void addScienceYield(float scienceVal, float sampleGrams) {
        accumulatedScienceScore += scienceVal;
        accumulatedSampleMassGrams += sampleGrams;
    }

    void rechargeBattery() {
        batteryLevel = maxBatteryLevel;
        if (currentStatus == RoverState::BATTERY_DEPLETED) {
            currentStatus = RoverState::WAITING_IDLE;
        }
    }

    void updateAnimation(float deltaTime) {
        constexpr float animationSpeed = 12.0f;
        drawingRowPosition += (static_cast<float>(currentRowPosition) - drawingRowPosition) * animationSpeed * deltaTime;
        drawingColumnPosition += (static_cast<float>(currentColumnPosition) - drawingColumnPosition) * animationSpeed * deltaTime;
    }

    void addCurrentPositionToTrail() {
        Point2D p(currentRowPosition, currentColumnPosition);
        if (trailCount < MAXIMUM_TRAIL_LENGTH) {
            trailBuffer[(trailStart + trailCount) % MAXIMUM_TRAIL_LENGTH] = p;
            trailCount++;
        } else {
            trailBuffer[trailStart] = p;
            trailStart = (trailStart + 1) % MAXIMUM_TRAIL_LENGTH;
        }
    }

    sf::Color getRoverProximityColor() const {
        if (distanceToNearestObstacle <= 1) return COLOR_DANGER;
        if (distanceToNearestObstacle <= 3) return COLOR_WARNING;
        return COLOR_SAFE;
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

    int getCurrentRow() const { return currentRowPosition; }
    int getCurrentColumn() const { return currentColumnPosition; }
    Point2D getPosition() const { return Point2D(currentRowPosition, currentColumnPosition); }
    float getDrawingRow() const { return drawingRowPosition; }
    float getDrawingColumn() const { return drawingColumnPosition; }
    const CustomVector<Point2D>& getCurrentPath() const { return currentPlannedPath; }
    int getPathStepIndex() const { return currentPathStepIndex; }

    CustomVector<Point2D> getTrail() const {
        CustomVector<Point2D> res;
        res.reserve(trailCount);
        for (int i = 0; i < trailCount; ++i) {
            res.push_back(trailBuffer[(trailStart + i) % MAXIMUM_TRAIL_LENGTH]);
        }
        return res;
    }

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
    const CustomVector<int>& getPlannedTour() const { return plannedSiteTour; }

    float getAccumulatedScience() const { return accumulatedScienceScore; }
    float getAccumulatedSampleMass() const { return accumulatedSampleMassGrams; }
    int getTotalMoves() const { return totalNumberOfMovesMade; }
};

#endif
