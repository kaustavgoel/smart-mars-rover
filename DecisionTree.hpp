#ifndef DECISION_TREE_HPP
#define DECISION_TREE_HPP

#include "Types.hpp"
#include "MarsElevationData.hpp"
#include "QuadTree.hpp"
#include <string>
#include <iostream>

enum class DecisionOutcome {
    PROCEED_TO_SITE,
    SKIP_INSUFFICIENT_BATTERY,
    SKIP_HAZARDOUS_TERRAIN,
    SKIP_LOW_SCIENCE_RETURN,
    ABORT_MISSION_DEPLETED,
    PERFORM_DEEP_DRILL,
    PERFORM_SHALLOW_TRENCH,
    SURFACE_SPECTRAL_ONLY
};

struct DecisionResult {
    DecisionOutcome outcome;
    std::string decisionTitle;
    std::string rationale;
    std::string tracePath;
    bool approved;

    DecisionResult()
        : outcome(DecisionOutcome::PROCEED_TO_SITE),
          decisionTitle("APPROVED"), rationale("All checks passed"),
          tracePath("Root -> BatteryOK -> TerrainOK -> ValueOK"),
          approved(true) {}
};

// decision tree to evaluate whether to commit, skip, or drill at a site
class RoverDecisionTree {
private:
    float safetyReserveBattery;
    float maxPermissibleSlope;

public:
    RoverDecisionTree(float safetyReserve = 25.0f, float maxSlope = 38.0f)
        : safetyReserveBattery(safetyReserve), maxPermissibleSlope(maxSlope) {}

    DecisionResult evaluateSiteCommitment(
        float currentBattery,
        float estimatedTripCost,
        const SiteData& targetSite,
        const MarsTerrainGrid& terrain,
        const QuadTree& spatialTree
    ) {
        DecisionResult res;

        // check if enough battery remains
        float totalNeeded = estimatedTripCost + targetSite.excavationEnergy + safetyReserveBattery;
        if (currentBattery < totalNeeded) {
            if (currentBattery < safetyReserveBattery + 10.0f) {
                res.outcome = DecisionOutcome::ABORT_MISSION_DEPLETED;
                res.decisionTitle = "MISSION ABORT";
                res.rationale = "Critical battery reserve reached (" + std::to_string(static_cast<int>(currentBattery)) + " Wh)";
                res.tracePath = "[Root: Battery] -> CRITICAL -> ABORT";
                res.approved = false;
                return res;
            } else {
                res.outcome = DecisionOutcome::SKIP_INSUFFICIENT_BATTERY;
                res.decisionTitle = "SKIP SITE (BATTERY)";
                res.rationale = "Required " + std::to_string(static_cast<int>(totalNeeded)) + " Wh, Available: " +
                                std::to_string(static_cast<int>(currentBattery)) + " Wh";
                res.tracePath = "[Root: Battery] -> DEFICIT -> SKIP";
                res.approved = false;
                return res;
            }
        }

        // check terrain hazards and nearby obstacles
        int nearestObstacleDist = spatialTree.findDistanceToNearestObstacle(targetSite.coord, 4.0f);
        float actualSlope = terrain.getSlope(targetSite.coord.getRow(), targetSite.coord.getColumn());
        if (targetSite.slope > maxPermissibleSlope || actualSlope > maxPermissibleSlope || nearestObstacleDist <= 0) {
            res.outcome = DecisionOutcome::SKIP_HAZARDOUS_TERRAIN;
            res.decisionTitle = "SKIP SITE (HAZARD)";
            res.rationale = "Site slope (" + std::to_string(static_cast<int>(targetSite.slope)) + " deg) exceeds rover safety limits";
            res.tracePath = "[Root: Battery OK] -> [Terrain: Hazard Check] -> DANGER -> SKIP";
            res.approved = false;
            return res;
        }

        // check science return ratio
        float efficiency = targetSite.scienceValue / (estimatedTripCost + targetSite.excavationEnergy + 0.1f);
        if (efficiency < 0.15f) {
            res.outcome = DecisionOutcome::SKIP_LOW_SCIENCE_RETURN;
            res.decisionTitle = "SKIP SITE (LOW VALUE)";
            res.rationale = "Expected science yield does not justify expenditure ratio";
            res.tracePath = "[Root: Battery OK] -> [Terrain OK] -> [Science: Ratio Check] -> LOW -> SKIP";
            res.approved = false;
            return res;
        }

        // all checks passed
        res.outcome = DecisionOutcome::PROCEED_TO_SITE;
        res.decisionTitle = "COMMIT & TRAVEL";
        res.rationale = "Site " + targetSite.name + " approved for travel and science extraction";
        res.tracePath = "[Battery OK] -> [Terrain OK] -> [Science OK] -> APPROVED";
        res.approved = true;
        return res;
    }

    // select drilling technique based on local conditions
    DecisionResult evaluateExcavationStrategy(
        float currentBattery,
        const SiteData& site,
        float localRoughness
    ) {
        DecisionResult res;

        if (currentBattery < site.excavationEnergy) {
            res.outcome = DecisionOutcome::SURFACE_SPECTRAL_ONLY;
            res.decisionTitle = "SURFACE SCAN ONLY";
            res.rationale = "Battery insufficient for mechanical drilling; falling back to passive spectrometer";
            res.tracePath = "[Site Arrival] -> [Drill Energy Check: Insufficient] -> SPECTRAL_SCAN";
            res.approved = true;
            return res;
        }

        if (localRoughness < 16.0f && site.slope < 18.0f) {
            res.outcome = DecisionOutcome::PERFORM_DEEP_DRILL;
            res.decisionTitle = "DEEP CORE DRILL";
            res.rationale = "Stable rock substrate detected. Safe for full core sample extraction";
            res.tracePath = "[Site Arrival] -> [Substrate: Stable] -> DEEP_CORE_DRILL";
            res.approved = true;
        } else if (site.slope < 30.0f) {
            res.outcome = DecisionOutcome::PERFORM_SHALLOW_TRENCH;
            res.decisionTitle = "SHALLOW TRENCH";
            res.rationale = "Moderately friable regolith. Excavating shallow 10-18 cm surface trench";
            res.tracePath = "[Site Arrival] -> [Substrate: Moderate] -> SHALLOW_TRENCH";
            res.approved = true;
        } else {
            res.outcome = DecisionOutcome::SURFACE_SPECTRAL_ONLY;
            res.decisionTitle = "SPECTRAL ONLY";
            res.rationale = "High slope hazard at target cell. Surface laser spectroscopy performed";
            res.tracePath = "[Site Arrival] -> [Substrate: Unstable] -> SPECTRAL_SCAN";
            res.approved = true;
        }

        return res;
    }
};

#endif // DECISION_TREE_HPP
