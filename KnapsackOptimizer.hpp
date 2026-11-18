#ifndef KNAPSACK_OPTIMIZER_HPP
#define KNAPSACK_OPTIMIZER_HPP

#include "Types.hpp"
#include "MarsElevationData.hpp"
#include <iostream>

struct KnapsackPlan {
    CustomVector<int> selectedSiteIds;
    float totalExpectedScienceValue;
    float totalEnergyRequired;
    int batteryBudgetUsed;
    int batteryBudgetAvailable;

    KnapsackPlan()
        : totalExpectedScienceValue(0.0f), totalEnergyRequired(0.0f),
          batteryBudgetUsed(0), batteryBudgetAvailable(0) {}
};

// 0/1 knapsack dynamic programming optimizer to pick best sites within battery limit
class KnapsackOptimizer {
public:
    static KnapsackPlan optimizeSiteSelection(
        const CustomVector<SiteData>& candidateSites,
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

        int* weights = new int[n];
        int* values = new int[n];

        for (int i = 0; i < n; ++i) {
            float dist = roverStartPos.distanceTo(candidateSites[i].coord);
            float targetCellCost = terrain.getCellCost(candidateSites[i].coord.getRow(), candidateSites[i].coord.getColumn());
            float estimatedTravelEnergy = dist * targetCellCost;
            float totalSiteEnergy = estimatedTravelEnergy + candidateSites[i].excavationEnergy;

            weights[i] = findMax(1, static_cast<int>(std::round(totalSiteEnergy)));
            values[i] = static_cast<int>(std::round(candidateSites[i].scienceValue * 10.0f));
        }

        // flat 1d array representing 2d dp table: (n + 1) x (batteryBudget + 1)
        int cols = batteryBudget + 1;
        int* dp = new int[(n + 1) * cols]();

        for (int i = 1; i <= n; ++i) {
            int w_i = weights[i - 1];
            int v_i = values[i - 1];

            for (int w = 0; w <= batteryBudget; ++w) {
                if (w >= w_i) {
                    int prevWithItem = dp[(i - 1) * cols + (w - w_i)] + v_i;
                    int prevWithout = dp[(i - 1) * cols + w];
                    dp[i * cols + w] = findMax(prevWithout, prevWithItem);
                } else {
                    dp[i * cols + w] = dp[(i - 1) * cols + w];
                }
            }
        }

        // backtrack to recover selected sites
        int remainingCap = batteryBudget;
        float accumulatedScience = 0.0f;
        float accumulatedEnergy = 0.0f;

        for (int i = n; i >= 1 && remainingCap > 0; --i) {
            if (dp[i * cols + remainingCap] != dp[(i - 1) * cols + remainingCap]) {
                int siteIdx = i - 1;
                plan.selectedSiteIds.push_back(candidateSites[siteIdx].siteId);
                accumulatedScience += candidateSites[siteIdx].scienceValue;
                accumulatedEnergy += static_cast<float>(weights[siteIdx]);
                remainingCap -= weights[siteIdx];
            }
        }

        // reverse to preserve sequence order
        int sSize = static_cast<int>(plan.selectedSiteIds.size());
        for (int i = 0; i < sSize / 2; ++i) {
            customSwap(plan.selectedSiteIds[i], plan.selectedSiteIds[sSize - 1 - i]);
        }

        plan.totalExpectedScienceValue = accumulatedScience;
        plan.totalEnergyRequired = accumulatedEnergy;
        plan.batteryBudgetUsed = batteryBudget - remainingCap;

        delete[] weights;
        delete[] values;
        delete[] dp;

        std::cout << "[KnapsackOptimizer] DP Optimization complete: Selected "
                  << plan.selectedSiteIds.size() << " / " << n << " sites. "
                  << "Expected Science: " << plan.totalExpectedScienceValue
                  << ", Battery: " << plan.batteryBudgetUsed << " / " << batteryBudget << " Wh."
                  << std::endl;

        return plan;
    }
};

#endif // KNAPSACK_OPTIMIZER_HPP
