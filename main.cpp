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
#include "HUD.hpp"

#include <SFML/Graphics.hpp>
#include <iostream>
#include <ctime>
#include <cstdlib>

// main simulation class tying all subsystems together
class MarsSimulation {
private:
    sf::RenderWindow window;
    sf::Font font;

    MarsTerrainGrid terrainGrid;
    QuadTree spatialTree;
    CustomVector<SiteData> candidateSites;

    AVLTree siteAVL;
    MaxHeap siteHeap;
    MarsRoverVehicle rover;
    RoverDecisionTree decisionTree;
    ExcavationNeuralNetwork neuralNet;
    ExcavationLog excavationLog;
    UserInterfaceHUD hud;

    // runtime flags
    bool isAutoMode;
    bool isPaused;
    bool showQuadTree;
    bool showDijkstra;
    float missionTime;

    PathResult currentAStarResult;
    PathResult currentDijkstraResult;
    DecisionResult currentDecision;
    ExcavationPrediction currentNNPrediction;
    KnapsackPlan currentKnapsackPlan;

    sf::Clock frameClock;
    sf::Clock moveTimer;
    float drillAnimationTimer;
    bool isDrilling;

    void loadSystemFont() {
        const std::string fontCandidates[] = {
            "/System/Library/Fonts/Supplemental/Arial.ttf",
            "/System/Library/Fonts/Helvetica.ttc",
            "/Library/Fonts/Arial.ttf",
            "arial.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
            "C:/Windows/Fonts/arial.ttf"
        };
        for (const auto& path : fontCandidates) {
            if (font.openFromFile(path)) {
                std::cout << "[System] Font loaded: " << path << std::endl;
                return;
            }
        }
        std::cerr << "[Warning] No font loaded, text might not show.\n";
    }

    void initCandidateSites() {
        candidateSites = {
            SiteData(101, "Delta Clay Beds",    Point2D(4, 8),   94.0f, 45.0f, -2415.0f, 12.0f, 8.5f,  0.88f),
            SiteData(102, "Olivine Outcrop",    Point2D(7, 21),  86.0f, 52.0f, -2120.0f, 22.0f, 14.0f, 0.76f),
            SiteData(103, "Carbonate Terrace",  Point2D(11, 16), 91.0f, 48.0f, -2180.0f, 15.0f, 9.2f,  0.92f),
            SiteData(104, "Hematite Spherules", Point2D(16, 6),  78.0f, 40.0f, -2410.0f, 8.0f,  6.0f,  0.82f),
            SiteData(105, "Basalt Dune Crest",  Point2D(15, 24), 65.0f, 38.0f, -2080.0f, 18.0f, 11.5f, 0.60f),
            SiteData(106, "Crater Ejecta Rim",  Point2D(3, 24),  72.0f, 60.0f, -2150.0f, 28.0f, 24.0f, 0.55f)
        };
    }

    void populateTreesAndHeap() {
        siteAVL.clear();
        siteHeap.clear();
        for (const auto& s : candidateSites) {
            siteAVL.insert(s);
            siteHeap.insert(s);
            spatialTree.insert(s.coord, "SITE", s.siteId);
        }
    }

    void planNextTarget() {
        int nextId = rover.getActiveTargetSiteId();
        if (nextId == -1) {
            rover.setStatus(RoverState::SUCCESSFULLY_ARRIVED);
            return;
        }

        SiteData* target = siteAVL.search(nextId);
        if (!target) return;

        float dist = rover.getPosition().distanceTo(target->coord);
        float estTripCost = dist * 2.2f;

        currentDecision = decisionTree.evaluateSiteCommitment(
            rover.getBattery(), estTripCost, *target, terrainGrid, spatialTree
        );

        if (!currentDecision.approved) {
            std::cout << "[DecisionTree] Site S" << target->siteId << " skipped ("
                      << currentDecision.decisionTitle << ").\n";
            rover.advanceTour();
            planNextTarget();
            return;
        }

        currentAStarResult = NavigationSystem::findPathAStar(terrainGrid, rover.getPosition(), target->coord);
        currentDijkstraResult = NavigationSystem::findPathDijkstra(terrainGrid, rover.getPosition(), target->coord);

        if (currentAStarResult.pathFound) {
            rover.assignNewPath(currentAStarResult.path);
        } else {
            rover.setStatus(RoverState::PATH_IS_STUCK);
        }
    }

    void doManualMove(int dr, int dc) {
        if (isDrilling) return;
        rover.moveManually(dr, dc, terrainGrid);
        for (auto& site : candidateSites) {
            SiteData* found = siteAVL.search(site.siteId);
            bool visited = (found ? found->isVisited : site.isVisited);
            if (!visited && rover.getCurrentRow() == site.coord.getRow() &&
                rover.getCurrentColumn() == site.coord.getColumn()) {
                rover.setActiveTargetSiteId(site.siteId);
                isDrilling = true;
                drillAnimationTimer = 0.0f;
                std::cout << "[Mission] Arrived at Site S" << site.siteId << "! Initiating excavation drill.\n";
                break;
            }
        }
    }

    void resetSimulation() {
        std::cout << "[Mission] Resetting environment and rover.\n";
        rover.resetRover();
        initCandidateSites();
        populateTreesAndHeap();
        excavationLog.clear();
        currentKnapsackPlan = KnapsackOptimizer::optimizeSiteSelection(
            candidateSites, terrainGrid, rover.getPosition(), static_cast<int>(rover.getBattery())
        );
        rover.setPlannedTour(currentKnapsackPlan.selectedSiteIds);
        planNextTarget();
        rover.setStatus(RoverState::WAITING_IDLE);
        isAutoMode = false;
        isPaused = false;
        missionTime = 0.0f;
        isDrilling = false;
    }

    void handleEvents() {
        while (auto eventOpt = window.pollEvent()) {
            auto& event = *eventOpt;
            if (event.is<sf::Event::Closed>()) {
                window.close();
            }

            if (auto* key = event.getIf<sf::Event::KeyPressed>()) {
                using Key = sf::Keyboard::Key;
                if (key->code == Key::M) {
                    isAutoMode = !isAutoMode;
                    if (isAutoMode) {
                        planNextTarget();
                        rover.setStatus(RoverState::AUTOMATIC_NAVIGATING);
                        std::cout << "[Control] Switched to AUTOMATIC mode.\n";
                    } else {
                        rover.setStatus(RoverState::MANUALLY_EXPLORING);
                        std::cout << "[Control] Switched to MANUAL mode.\n";
                    }
                } else if (key->code == Key::P) {
                    isPaused = !isPaused;
                    std::cout << (isPaused ? "[Control] Simulation PAUSED.\n" : "[Control] Simulation RESUMED.\n");
                } else if (key->code == Key::R) {
                    resetSimulation();
                } else if (key->code == Key::B) {
                    rover.rechargeBattery();
                    std::cout << "[Power] Solar battery recharged to 100%!\n";
                } else if (key->code == Key::Q) {
                    showQuadTree = !showQuadTree;
                    std::cout << "[Display] QuadTree overlay: " << (showQuadTree ? "ON" : "OFF") << "\n";
                } else if (key->code == Key::D) {
                    showDijkstra = !showDijkstra;
                    std::cout << "[Display] Dijkstra overlay: " << (showDijkstra ? "ON" : "OFF") << "\n";
                } else if (key->code == Key::Space) {
                    if (rover.getStatus() == RoverState::AUTOMATIC_NAVIGATING) {
                        bool stillMoving = rover.executeNextAutomaticStep(terrainGrid);
                        if (!stillMoving && rover.getActiveTargetSiteId() != -1) {
                            isDrilling = true;
                            drillAnimationTimer = 0.0f;
                        }
                    }
                } else if (key->code == Key::Up) {
                    isAutoMode = false;
                    doManualMove(-1, 0);
                } else if (key->code == Key::Down) {
                    isAutoMode = false;
                    doManualMove(1, 0);
                } else if (key->code == Key::Left) {
                    isAutoMode = false;
                    doManualMove(0, -1);
                } else if (key->code == Key::Right) {
                    isAutoMode = false;
                    doManualMove(0, 1);
                }
            }
        }
    }

    void update(float deltaTime) {
        if (isPaused) return;
        missionTime += deltaTime;
        rover.updateAnimation(deltaTime);

        // proximity sensor
        int nearestObs = spatialTree.findDistanceToNearestObstacle(rover.getPosition(), 6.0f);
        rover.setObstacleDistance(nearestObs);

        if (isDrilling) {
            drillAnimationTimer += deltaTime;
            rover.setStatus(RoverState::EXCAVATING_SITE);

            if (drillAnimationTimer >= 1.6f) {
                int currentSiteId = rover.getActiveTargetSiteId();
                SiteData* site = siteAVL.search(currentSiteId);

                if (site) {
                    DecisionResult digDecision = decisionTree.evaluateExcavationStrategy(
                        rover.getBattery(), *site, site->roughness
                    );

                    MathVector nnInputs = {
                        terrainGrid.getNormalizedElevation(site->coord.getRow(), site->coord.getColumn()),
                        site->slope / 45.0f,
                        site->roughness / 30.0f,
                        site->mineralSignature
                    };

                    currentNNPrediction = neuralNet.forward(nnInputs);
                    rover.deductExcavationBattery(site->excavationEnergy);
                    float sampleGrams = currentNNPrediction.predictedDepthCm * 4.2f;
                    rover.addScienceYield(site->scienceValue, sampleGrams);

                    std::string outcome = digDecision.decisionTitle + " (" +
                                          std::to_string(static_cast<int>(sampleGrams)) + "g core sample)";
                    excavationLog.appendRecord(
                        site->siteId, site->name, site->coord,
                        currentNNPrediction.offsetRow, currentNNPrediction.offsetCol,
                        currentNNPrediction.predictedDepthCm, sampleGrams,
                        outcome, site->excavationEnergy, missionTime
                    );

                    site->isVisited = true;
                    for (auto& cs : candidateSites) {
                        if (cs.siteId == site->siteId) {
                            cs.isVisited = true;
                            break;
                        }
                    }
                    siteHeap.removeSiteById(site->siteId);

                    std::cout << "[Science] Core extracted at S" << site->siteId << " (" << site->name
                              << ")! Depth: " << currentNNPrediction.predictedDepthCm << " cm, Yield: "
                              << sampleGrams << " g.\n";
                }

                isDrilling = false;
                drillAnimationTimer = 0.0f;
                if (isAutoMode) {
                    rover.advanceTour();
                    planNextTarget();
                } else {
                    rover.setStatus(RoverState::MANUALLY_EXPLORING);
                }
            }
        } else if (isAutoMode) {
            if (moveTimer.getElapsedTime().asSeconds() >= AUTOMATIC_MOVE_INTERVAL) {
                moveTimer.restart();
                if (rover.getStatus() == RoverState::AUTOMATIC_NAVIGATING) {
                    bool stillMoving = rover.executeNextAutomaticStep(terrainGrid);
                    if (!stillMoving && rover.getActiveTargetSiteId() != -1) {
                        isDrilling = true;
                        drillAnimationTimer = 0.0f;
                    }
                }
            }
        } else {
            if (moveTimer.getElapsedTime().asSeconds() >= MANUAL_MOVE_INTERVAL) {
                using Key = sf::Keyboard::Key;
                int dr = 0, dc = 0;
                if (sf::Keyboard::isKeyPressed(Key::Up)) dr = -1;
                else if (sf::Keyboard::isKeyPressed(Key::Down)) dr = 1;
                else if (sf::Keyboard::isKeyPressed(Key::Left)) dc = -1;
                else if (sf::Keyboard::isKeyPressed(Key::Right)) dc = 1;

                if (dr != 0 || dc != 0) {
                    moveTimer.restart();
                    doManualMove(dr, dc);
                }
            }
        }
    }

    void render() {
        window.clear(COLOR_BACKGROUND);

        // draw terrain cells
        for (int r = 0; r < TOTAL_ROWS; ++r) {
            for (int c = 0; c < TOTAL_COLUMNS; ++c) {
                float px = static_cast<float>(c * CELL_SIZE_PIXELS);
                float py = static_cast<float>(r * CELL_SIZE_PIXELS);

                sf::RectangleShape cell(sf::Vector2f(static_cast<float>(CELL_SIZE_PIXELS - 1),
                                                     static_cast<float>(CELL_SIZE_PIXELS - 1)));
                cell.setPosition(sf::Vector2f(px, py));
                cell.setFillColor(terrainGrid.getCellVisualColor(r, c));
                window.draw(cell);

                if (terrainGrid.getTerrainType(r, c) == TerrainType::OBSTACLE) {
                    sf::RectangleShape obsRift(sf::Vector2f(static_cast<float>(CELL_SIZE_PIXELS - 1), 3.0f));
                    obsRift.setPosition(sf::Vector2f(px, py + 1.0f));
                    obsRift.setFillColor(sf::Color(255, 90, 80, 180));
                    window.draw(obsRift);
                }
            }
        }

        // draw quadtree boxes
        if (showQuadTree) {
            auto boxes = spatialTree.getAllBoundingBoxes();
            for (const auto& b : boxes) {
                sf::RectangleShape boxRect(sf::Vector2f(b.width * CELL_SIZE_PIXELS, b.height * CELL_SIZE_PIXELS));
                boxRect.setPosition(sf::Vector2f(b.x * CELL_SIZE_PIXELS, b.y * CELL_SIZE_PIXELS));
                boxRect.setFillColor(sf::Color(0, 200, 255, 12));
                boxRect.setOutlineColor(sf::Color(0, 220, 255, 80));
                boxRect.setOutlineThickness(1.0f);
                window.draw(boxRect);
            }
        }

        // dijkstra baseline path
        if (showDijkstra && currentDijkstraResult.pathFound) {
            for (const auto& pt : currentDijkstraResult.path) {
                sf::CircleShape dot(3.0f);
                dot.setFillColor(sf::Color(80, 220, 255, 160));
                dot.setPosition(sf::Vector2f(pt.getColumn() * CELL_SIZE_PIXELS + CELL_SIZE_PIXELS / 2.0f - 3.0f,
                                             pt.getRow() * CELL_SIZE_PIXELS + CELL_SIZE_PIXELS / 2.0f - 3.0f));
                window.draw(dot);
            }
        }

        // a* planned route
        const auto& plannedPath = rover.getCurrentPath();
        int pathLen = static_cast<int>(plannedPath.size());
        for (int i = 0; i < pathLen; ++i) {
            Point2D pt = plannedPath[i];
            bool passed = (i < rover.getPathStepIndex());

            sf::RectangleShape pathTile(sf::Vector2f(CELL_SIZE_PIXELS - 8.0f, CELL_SIZE_PIXELS - 8.0f));
            pathTile.setPosition(sf::Vector2f(pt.getColumn() * CELL_SIZE_PIXELS + 4.0f,
                                              pt.getRow() * CELL_SIZE_PIXELS + 4.0f));
            pathTile.setFillColor(passed ? sf::Color(255, 215, 60, 30) : sf::Color(255, 215, 60, 140));
            window.draw(pathTile);

            if (!passed) {
                sf::CircleShape dot(2.5f);
                dot.setFillColor(sf::Color(255, 245, 140, 220));
                dot.setPosition(sf::Vector2f(pt.getColumn() * CELL_SIZE_PIXELS + CELL_SIZE_PIXELS / 2.0f - 2.5f,
                                             pt.getRow() * CELL_SIZE_PIXELS + CELL_SIZE_PIXELS / 2.0f - 2.5f));
                window.draw(dot);
            }
        }

        // rover trail
        auto trail = rover.getTrail();
        int trailLen = static_cast<int>(trail.size());
        for (int i = 0; i < trailLen; ++i) {
            Point2D tp = trail[i];
            float alpha = (static_cast<float>(i + 1) / trailLen) * 160.0f;
            float rad = 3.5f + 2.5f * (static_cast<float>(i + 1) / trailLen);

            sf::CircleShape trailCircle(rad);
            trailCircle.setFillColor(sf::Color(70, 190, 245, static_cast<uint8_t>(alpha)));
            trailCircle.setPosition(sf::Vector2f(tp.getColumn() * CELL_SIZE_PIXELS + CELL_SIZE_PIXELS / 2.0f - rad,
                                                 tp.getRow() * CELL_SIZE_PIXELS + CELL_SIZE_PIXELS / 2.0f - rad));
            window.draw(trailCircle);
        }

        // candidate sites
        float pulse = 0.5f + 0.5f * std::sin(missionTime * 3.5f);
        auto allSites = siteAVL.getAllSitesInOrder();
        for (const auto& site : allSites) {
            float sx = static_cast<float>(site.coord.getColumn() * CELL_SIZE_PIXELS);
            float sy = static_cast<float>(site.coord.getRow() * CELL_SIZE_PIXELS);

            bool isTarget = (site.siteId == rover.getActiveTargetSiteId());
            sf::Color siteCol = site.isVisited ? COLOR_SAFE :
                                (isTarget ? COLOR_WARNING :
                                (site.isSelectedInKnapsack ? COLOR_ACCENT : sf::Color(160, 180, 205)));

            if (!site.isVisited) {
                float ringRad = CELL_SIZE_PIXELS / 2.0f + pulse * 5.0f;
                sf::CircleShape ring(ringRad);
                ring.setPosition(sf::Vector2f(sx + CELL_SIZE_PIXELS / 2.0f - ringRad,
                                              sy + CELL_SIZE_PIXELS / 2.0f - ringRad));
                ring.setFillColor(sf::Color(siteCol.r, siteCol.g, siteCol.b, static_cast<uint8_t>(40 * pulse)));
                ring.setOutlineColor(siteCol);
                ring.setOutlineThickness(1.0f);
                window.draw(ring);
            }

            sf::CircleShape core(7.0f);
            core.setPosition(sf::Vector2f(sx + CELL_SIZE_PIXELS / 2.0f - 7.0f,
                                          sy + CELL_SIZE_PIXELS / 2.0f - 7.0f));
            core.setFillColor(siteCol);
            core.setOutlineColor(sf::Color::White);
            core.setOutlineThickness(1.0f);
            window.draw(core);

            sf::Text label(font, "S" + std::to_string(site.siteId), 9);
            label.setFillColor(sf::Color(240, 245, 255));
            label.setStyle(sf::Text::Bold);
            label.setPosition(sf::Vector2f(sx + 2.0f, sy + 1.0f));
            window.draw(label);
        }

        // rover body
        {
            float roverCenterX = rover.getDrawingColumn() * CELL_SIZE_PIXELS + CELL_SIZE_PIXELS / 2.0f;
            float roverCenterY = rover.getDrawingRow() * CELL_SIZE_PIXELS + CELL_SIZE_PIXELS / 2.0f;
            float rRad = CELL_SIZE_PIXELS / 2.0f - 3.5f;

            sf::CircleShape shadow(rRad + 2.0f);
            shadow.setFillColor(sf::Color(0, 0, 0, 90));
            shadow.setPosition(sf::Vector2f(roverCenterX - rRad, roverCenterY - rRad));
            window.draw(shadow);

            sf::Color roverCol = rover.getRoverProximityColor();
            sf::CircleShape glow(rRad + 4.0f);
            glow.setFillColor(sf::Color(roverCol.r, roverCol.g, roverCol.b, 60));
            glow.setPosition(sf::Vector2f(roverCenterX - rRad - 4.0f, roverCenterY - rRad - 4.0f));
            window.draw(glow);

            sf::CircleShape body(rRad);
            body.setFillColor(roverCol);
            body.setOutlineColor(sf::Color::White);
            body.setOutlineThickness(1.5f);
            body.setPosition(sf::Vector2f(roverCenterX - rRad, roverCenterY - rRad));
            window.draw(body);

            sf::CircleShape centerHub(rRad * 0.45f);
            centerHub.setFillColor(sf::Color(25, 30, 40));
            centerHub.setPosition(sf::Vector2f(roverCenterX - rRad * 0.45f, roverCenterY - rRad * 0.45f));
            window.draw(centerHub);

            sf::CircleShape cameraEye(2.5f);
            cameraEye.setFillColor(COLOR_ACCENT_GLOW);
            cameraEye.setPosition(sf::Vector2f(roverCenterX + rRad * 0.35f - 2.5f, roverCenterY - 2.5f));
            window.draw(cameraEye);
        }

        // drill animation
        if (isDrilling) {
            float drillR = (drillAnimationTimer / 1.6f) * (CELL_SIZE_PIXELS * 1.8f);
            float drillX = rover.getCurrentColumn() * CELL_SIZE_PIXELS + CELL_SIZE_PIXELS / 2.0f;
            float drillY = rover.getCurrentRow() * CELL_SIZE_PIXELS + CELL_SIZE_PIXELS / 2.0f;

            sf::CircleShape wave(drillR);
            wave.setPosition(sf::Vector2f(drillX - drillR, drillY - drillR));
            wave.setFillColor(sf::Color(245, 180, 45, static_cast<uint8_t>(120 * (1.0f - drillAnimationTimer / 1.6f))));
            wave.setOutlineColor(sf::Color::White);
            wave.setOutlineThickness(2.0f);
            window.draw(wave);

            sf::Text drillText(font, "[ EXCAVATING CORE... ]", 14);
            drillText.setFillColor(COLOR_WARNING);
            drillText.setStyle(sf::Text::Bold);
            drillText.setPosition(sf::Vector2f(drillX - 85.0f, drillY - 25.0f));
            window.draw(drillText);
        }

        hud.render(window, rover, siteHeap, siteAVL, currentKnapsackPlan,
                   currentDecision, currentNNPrediction, excavationLog,
                   isAutoMode, isPaused, missionTime, currentAStarResult,
                   currentDijkstraResult, showQuadTree, showDijkstra);

        window.display();
    }

public:
    MarsSimulation()
        : window(sf::VideoMode({static_cast<unsigned int>(WINDOW_WIDTH_PIXELS),
                                static_cast<unsigned int>(WINDOW_HEIGHT_PIXELS)}),
                 "Mars Rover: Multi-Site Navigation & Excavation Planning"),
          rover(420.0f),
          hud(font),
          isAutoMode(false),
          isPaused(false),
          showQuadTree(false),
          showDijkstra(false),
          missionTime(0.0f),
          drillAnimationTimer(0.0f),
          isDrilling(false)
    {
        window.setFramerateLimit(60);
        loadSystemFont();

        // obstacle indexing into quadtree
        for (int r = 0; r < TOTAL_ROWS; ++r) {
            for (int c = 0; c < TOTAL_COLUMNS; ++c) {
                if (terrainGrid.getTerrainType(r, c) == TerrainType::OBSTACLE) {
                    spatialTree.insert(Point2D(r, c), "OBSTACLE");
                }
            }
        }

        initCandidateSites();
        populateTreesAndHeap();

        currentKnapsackPlan = KnapsackOptimizer::optimizeSiteSelection(
            candidateSites, terrainGrid, rover.getPosition(), static_cast<int>(rover.getBattery())
        );
        rover.setPlannedTour(currentKnapsackPlan.selectedSiteIds);

        planNextTarget();
        rover.setStatus(RoverState::WAITING_IDLE);

        std::cout << "[System] Ready. Manual control active. Use Arrow keys to move.\n";
    }

    void run() {
        while (window.isOpen()) {
            float dt = frameClock.restart().asSeconds();
            handleEvents();
            update(dt);
            render();
        }
        std::cout << "[Mission] Simulation closed.\n";
    }
};

int main() {
    std::cout << "========================================================\n"
              << " SMART MARS ROVER: MULTI-SITE NAVIGATION & EXCAVATION\n"
              << " Jaypee Institute of Information Technology - B.Tech III\n"
              << "========================================================\n";

    srand(static_cast<unsigned int>(time(nullptr)));

    MarsSimulation simulation;
    simulation.run();

    return 0;
}
