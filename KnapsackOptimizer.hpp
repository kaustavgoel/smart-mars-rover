#ifndef KNAPSACK_OPTIMIZER_HPP
#define KNAPSACK_OPTIMIZER_HPP

#include "Types.hpp"
#include "MarsElevationData.hpp"
#include <vector>
#include <algorithm>
#include <iostream>

struct KnapsackPlan {
    std::vector<int> selectedSiteIds;
    float totalExpectedScienceValue;
    float totalEnergyRequired;
    int batteryBudgetUsed;
    int batteryBudgetAvailable;

    KnapsackPlan()
        : totalExpectedScienceValue(0.0f), totalEnergyRequired(0.0f),
          batteryBudgetUsed(0), batteryBudgetAvailable(0) {}
};

/**
 * @brief 0/1 Knapsack Dynamic Programming Optimizer.
 * Optimally selects the highest-value combination of Mars candidate sites that
 * can be visited and excavated without exceeding the rover's finite battery capacity.
 */
class KnapsackOptimizer {
public:
    static KnapsackPlan optimizeSiteSelection(
        const std::vector<SiteData>& candidateSites,
        const MarsTerrainGrid& terrain,
        Point2D roverStartPos,
        int batteryBudget
    ) {
        KnapsackPlan plan;
        plan.batteryBudgetAvailable = batteryBudget;

        if (candidateSites.empty() || batteryBudget <= 0) {
            return plan;
        }

        int n = static_cast<int>(candidateSites.size());

        // Compute weights (total energy = estimated path energy + excavation energy)
        // and values (scientific value)
        std::vector<int> weights(n);
        std::vector<int> values(n);

        for (int i = 0; i < n; ++i) {
            // Path traversal energy estimate (using terrain cell cost and distance)
            float dist = roverStartPos.distanceTo(candidateSites[i].coord);
            float targetCellCost = terrain.getCellCost(candidateSites[i].coord.getRow(), candidateSites[i].coord.getColumn());
            float estimatedTravelEnergy = dist * targetCellCost;
            float totalSiteEnergy = estimatedTravelEnergy + candidateSites[i].excavationEnergy;

            weights[i] = std::max(1, static_cast<int>(std::round(totalSiteEnergy)));
            values[i] = static_cast<int>(std::round(candidateSites[i].scienceValue * 10.0f)); // Scale for integer DP
        }

        // DP Table: dp[i][w] = max value using a subset of first i sites with weight <= w
        // O(N * W) time and space complexity
        std::vector<std::vector<int>> dp(n + 1, std::vector<int>(batteryBudget + 1, 0));

        for (int i = 1; i <= n; ++i) {
            int w_i = weights[i - 1];
            int v_i = values[i - 1];

            for (int w = 0; w <= batteryBudget; ++w) {
                if (w >= w_i) {
                    dp[i][w] = std::max(dp[i - 1][w], dp[i - 1][w - w_i] + v_i);
                } else {
                    dp[i][w] = dp[i - 1][w];
                }
            }
        }

        // Backtracking to find selected sites
        int remainingCap = batteryBudget;
        float accumulatedScience = 0.0f;
        float accumulatedEnergy = 0.0f;

        for (int i = n; i >= 1 && remainingCap > 0; --i) {
            if (dp[i][remainingCap] != dp[i - 1][remainingCap]) {
                // Item i - 1 was included
                int siteIdx = i - 1;
                plan.selectedSiteIds.push_back(candidateSites[siteIdx].siteId);
                accumulatedScience += candidateSites[siteIdx].scienceValue;
                accumulatedEnergy += static_cast<float>(weights[siteIdx]);
                remainingCap -= weights[siteIdx];
            }
        }

        // Maintain selection in original order or logical visit sequence
        std::reverse(plan.selectedSiteIds.begin(), plan.selectedSiteIds.end());

        plan.totalExpectedScienceValue = accumulatedScience;
        plan.totalEnergyRequired = accumulatedEnergy;
        plan.batteryBudgetUsed = batteryBudget - remainingCap;

        std::cout << "[KnapsackOptimizer] DP Optimization complete: Selected "
                  << plan.selectedSiteIds.size() << " / " << n << " sites. "
                  << "Expected Science: " << plan.totalExpectedScienceValue
                  << ", Battery: " << plan.batteryBudgetUsed << " / " << batteryBudget << " Wh."
                  << std::endl;

        return plan;
    }
};

#endif // KNAPSACK_OPTIMIZER_HPP
