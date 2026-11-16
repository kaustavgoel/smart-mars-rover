#ifndef HUD_HPP
#define HUD_HPP

#include "Types.hpp"
#include "MarsRover.hpp"
#include "MarsElevationData.hpp"
#include "QuadTree.hpp"
#include "MaxHeap.hpp"
#include "AVLTree.hpp"
#include "KnapsackOptimizer.hpp"
#include "DecisionTree.hpp"
#include "NeuralNetwork.hpp"
#include "ExcavationLog.hpp"
#include "PathFinding.hpp"
#include <SFML/Graphics.hpp>
#include <sstream>
#include <iomanip>
#include <vector>

/**
 * @brief User Interface HUD for the Smart Mars Rover system.
 * Designed with a clean, human-engineered dashboard layout.
 */
class UserInterfaceHUD {
private:
    const sf::Font& font;

    sf::Text createText(const std::string& str, unsigned int size, sf::Color color,
                        sf::Vector2f pos, sf::Text::Style style = sf::Text::Regular) const {
        sf::Text txt(font, str, size);
        txt.setFillColor(color);
        txt.setPosition(pos);
        txt.setStyle(style);
        return txt;
    }

    void drawBox(sf::RenderWindow& window, float x, float y, float w, float h,
                 sf::Color fill, sf::Color border, float thickness = 1.0f) const {
        sf::RectangleShape rect(sf::Vector2f(w, h));
        rect.setPosition(sf::Vector2f(x, y));
        rect.setFillColor(fill);
        rect.setOutlineColor(border);
        rect.setOutlineThickness(thickness);
        window.draw(rect);
    }

    void drawLabel(sf::RenderWindow& window, const std::string& key, const std::string& val,
                   float x, float y, sf::Color valColor = COLOR_TEXT_MAIN) const {
        sf::Text keyText = createText(key, 10, COLOR_TEXT_MUTED, {x, y});
        window.draw(keyText);
        sf::Text valText = createText(val, 13, valColor, {x, y + 13.0f}, sf::Text::Bold);
        window.draw(valText);
    }

public:
    explicit UserInterfaceHUD(const sf::Font& f) : font(f) {}

    void render(
        sf::RenderWindow& window,
        const MarsRoverVehicle& rover,
        const MaxHeap& siteHeap,
        const AVLTree& siteAVL,
        const KnapsackPlan& knapsackPlan,
        const DecisionResult& decision,
        const ExcavationPrediction& nnPred,
        const ExcavationLog& digLog,
        bool isAutoMode,
        bool isPaused,
        float missionTime,
        const PathResult& aStarResult,
        const PathResult& dijkstraResult,
        bool showQuadTree,
        bool showDijkstra
    ) const {
        float yStart = static_cast<float>(MAP_HEIGHT_PIXELS);
        float totalW = static_cast<float>(WINDOW_WIDTH_PIXELS);
        float totalH = static_cast<float>(HUD_HEIGHT_PIXELS);

        // 1. HUD Background and Border
        drawBox(window, 0.0f, yStart, totalW, totalH, COLOR_HUD_BG, COLOR_PANEL_BORDER, 2.0f);

        // 2. Header Status Line
        float headerY = yStart + 8.0f;
        sf::Text title = createText("MARS ROVER: MULTI-SITE SCIENCE & EXCAVATION", 12, COLOR_ACCENT,
                                    {16.0f, headerY}, sf::Text::Bold);
        window.draw(title);

        std::string modeBadge = isAutoMode ? "[ MODE: AUTO / A* ]" : "[ MODE: MANUAL ]";
        sf::Color modeColor = isAutoMode ? COLOR_ACCENT_GLOW : COLOR_TEXT_MAIN;
        sf::Text modeText = createText(modeBadge, 12, modeColor, {totalW / 2.0f - 60.0f, headerY}, sf::Text::Bold);
        window.draw(modeText);

        if (isPaused) {
            sf::Text pausedText = createText("[ PAUSED ]", 12, COLOR_WARNING, {totalW - 100.0f, headerY}, sf::Text::Bold);
            window.draw(pausedText);
        }

        // 3. Four Dashboard Columns
        float col1X = 16.0f;
        float col2X = 230.0f;
        float col3X = 450.0f;
        float col4X = 670.0f;

        float row1Y = headerY + 22.0f;

        // Column 1: Rover Status & Position
        drawLabel(window, "ROVER STATUS", rover.getStatusString(), col1X, row1Y, rover.getRoverProximityColor());
        drawLabel(window, "GPS POSITION", "(" + std::to_string(rover.getCurrentRow()) + ", " +
                                       std::to_string(rover.getCurrentColumn()) + ")", col2X, row1Y);

        // Active Target Site
        std::string targetStr = "None";
        if (rover.getActiveTargetSiteId() != -1) {
            const SiteData* ts = siteAVL.search(rover.getActiveTargetSiteId());
            if (ts) targetStr = "S" + std::to_string(ts->siteId) + " - " + ts->name;
        } else if (rover.getStatus() == RoverState::SUCCESSFULLY_ARRIVED) {
            targetStr = "Mission Tour Complete";
        }
        drawLabel(window, "CURRENT TARGET", targetStr, col3X, row1Y, COLOR_WARNING);

        // Obstacle Proximity via QuadTree
        int dist = rover.getObstacleDistance();
        std::string proxStr = (dist >= 99) ? "ALL CLEAR" : std::to_string(dist) + " cells";
        drawLabel(window, "OBSTACLE PROXIMITY", proxStr, col4X, row1Y, rover.getRoverProximityColor());

        // Row 2
        float row2Y = row1Y + 32.0f;

        // Battery Power & Gauge Bar
        float pct = rover.getBatteryPercentage();
        std::stringstream ssBat;
        ssBat << std::fixed << std::setprecision(0) << rover.getBattery() << "/" << rover.getMaxBattery() << " Wh (" << static_cast<int>(pct) << "%)";
        sf::Color batColor = (pct > 50.0f) ? COLOR_BATTERY_GREEN : (pct > 25.0f ? COLOR_BATTERY_YELLOW : COLOR_BATTERY_RED);
        drawLabel(window, "BATTERY BUDGET", ssBat.str(), col1X, row2Y, batColor);

        // Battery Gauge Bar below text
        float barW = 180.0f;
        float barH = 6.0f;
        drawBox(window, col1X, row2Y + 28.0f, barW, barH, sf::Color(30, 38, 50), sf::Color(55, 70, 95));
        float fillW = barW * (clampValue(pct, 0.0f, 100.0f) / 100.0f);
        if (fillW > 0.0f) {
            sf::RectangleShape fillBar(sf::Vector2f(fillW, barH));
            fillBar.setPosition(sf::Vector2f(col1X, row2Y + 28.0f));
            fillBar.setFillColor(batColor);
            window.draw(fillBar);
        }

        // Mission Stats
        int mins = static_cast<int>(missionTime) / 60;
        int secs = static_cast<int>(missionTime) % 60;
        std::string statStr = padLeft(std::to_string(mins), 2) + ":" + padLeft(std::to_string(secs), 2) +
                              " | " + std::to_string(rover.getTotalMoves()) + " moves";
        drawLabel(window, "MISSION TELEMETRY", statStr, col2X, row2Y);

        // Priority Site (Max-Heap) & Catalog (AVL Tree)
        std::string priorityStr;
        if (!siteHeap.isEmpty()) {
            SiteData top = siteHeap.peekMax();
            priorityStr = "S" + std::to_string(top.siteId) + " (" + std::to_string(static_cast<int>(top.scienceValue)) + " pts) | " +
                          std::to_string(siteAVL.size()) + " in AVL";
        } else {
            priorityStr = "All Visited | " + std::to_string(siteAVL.size()) + " in AVL";
        }
        drawLabel(window, "HEAP PRIORITY & AVL", priorityStr, col3X, row2Y);

        // Decision Tree, NN Dig Depth & A* Path
        std::stringstream ssNN;
        ssNN << decision.decisionTitle << " | " << std::fixed << std::setprecision(1) << nnPred.predictedDepthCm << " cm";
        if (aStarResult.pathFound) {
            ssNN << " (" << aStarResult.nodesExplored << " nodes)";
        }
        (void)dijkstraResult;
        drawLabel(window, "DECISION & NN DIG", ssNN.str(), col4X, row2Y, decision.approved ? COLOR_SAFE : COLOR_DANGER);

        // Row 3: Knapsack Tour & Excavation Log
        float row3Y = row2Y + 38.0f;

        // Knapsack Tour sequence
        std::string tourStr = "Tour: ";
        for (size_t i = 0; i < knapsackPlan.selectedSiteIds.size(); ++i) {
            int sid = knapsackPlan.selectedSiteIds[i];
            if (sid == rover.getActiveTargetSiteId()) tourStr += "[S" + std::to_string(sid) + "*] ";
            else tourStr += "S" + std::to_string(sid) + " ";
        }
        tourStr += " (Est: " + std::to_string(static_cast<int>(knapsackPlan.totalExpectedScienceValue)) + " pts)";
        sf::Text knapText = createText(tourStr, 11, COLOR_SAFE, {col1X, row3Y}, sf::Text::Bold);
        window.draw(knapText);

        // Recent Excavation Log
        std::string logStr = "Excavation Log: ";
        auto recentDigs = digLog.getRecentRecords(1);
        if (!recentDigs.empty()) {
            const auto& r = recentDigs[0];
            logStr += "#" + std::to_string(r.logIndex) + " S" + std::to_string(r.siteId) +
                      " Depth: " + std::to_string(static_cast<int>(r.predictedDepthCm)) + "cm, Yield: " +
                      std::to_string(static_cast<int>(r.sampleYieldGrams)) + "g";
        } else {
            logStr += "Standby at candidate coordinates";
        }
        sf::Text logText = createText(logStr, 11, COLOR_TEXT_MAIN, {col3X, row3Y});
        window.draw(logText);

        // 4. Bottom Controls Strip (Pure Keyboard)
        float controlsY = yStart + totalH - 22.0f;
        drawBox(window, 16.0f, controlsY - 4.0f, totalW - 32.0f, 1.0f, COLOR_PANEL_BORDER, COLOR_PANEL_BORDER);

        std::string controlsStr = "[ARROWS] Move   [M] Auto/Manual   [Q] QuadTree (" +
                                  std::string(showQuadTree ? "ON" : "OFF") + ")   [D] Dijkstra (" +
                                  std::string(showDijkstra ? "ON" : "OFF") + ")   [P] Pause   [R] Reset";
        sf::Text controlsText = createText(controlsStr, 11, COLOR_TEXT_MUTED, {col1X, controlsY});
        window.draw(controlsText);
    }
};

#endif // HUD_HPP
