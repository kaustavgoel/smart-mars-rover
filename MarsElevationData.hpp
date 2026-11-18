#ifndef MARS_ELEVATION_DATA_HPP
#define MARS_ELEVATION_DATA_HPP

#include "Types.hpp"
#include <iostream>


class MarsTerrainGrid {
private:
    float elevationMatrix[TOTAL_ROWS][TOTAL_COLUMNS];
    float slopeMatrix[TOTAL_ROWS][TOTAL_COLUMNS];
    float roughnessMatrix[TOTAL_ROWS][TOTAL_COLUMNS];
    TerrainType terrainGrid[TOTAL_ROWS][TOTAL_COLUMNS];
    float traversalCostGrid[TOTAL_ROWS][TOTAL_COLUMNS];

    float minElevation;
    float maxElevation;

public:
    MarsTerrainGrid() {
        for (int r = 0; r < TOTAL_ROWS; ++r) {
            for (int c = 0; c < TOTAL_COLUMNS; ++c) {
                elevationMatrix[r][c] = 0.0f;
                slopeMatrix[r][c] = 0.0f;
                roughnessMatrix[r][c] = 0.0f;
                terrainGrid[r][c] = TerrainType::PLAINS;
                traversalCostGrid[r][c] = 1.0f;
            }
        }
        loadRealMarsDatasetTile();
        preprocessTerrain();
    }


    void loadRealMarsDatasetTile() {
        static const float baseDEM[TOTAL_ROWS][TOTAL_COLUMNS] = {
            {-2440, -2438, -2435, -2432, -2430, -2425, -2420, -2415, -2412, -2410, -2408, -2405, -2402, -2400, -2395, -2380, -2350, -2300, -2250, -2200, -2180, -2170, -2160, -2150, -2160, -2180, -2220, -2300, -2380, -2430},
            {-2438, -2435, -2432, -2430, -2426, -2422, -2418, -2412, -2410, -2408, -2404, -2400, -2398, -2392, -2385, -2340, -2280, -2220, -2170, -2140, -2130, -2140, -2155, -2165, -2180, -2210, -2260, -2340, -2400, -2435},
            {-2435, -2432, -2430, -2427, -2423, -2419, -2415, -2410, -2406, -2402, -2398, -2395, -2390, -2385, -2360, -2300, -2240, -2180, -2140, -2120, -2110, -2125, -2145, -2160, -2190, -2230, -2290, -2360, -2410, -2438},
            {-2434, -2430, -2428, -2425, -2420, -2416, -2412, -2408, -2404, -2400, -2395, -2390, -2386, -2370, -2320, -2260, -2200, -2150, -2120, -2100, -2090, -2110, -2135, -2150, -2180, -2220, -2270, -2340, -2400, -2436},
            {-2436, -2432, -2429, -2425, -2420, -2415, -2410, -2405, -2400, -2395, -2390, -2384, -2375, -2340, -2280, -2220, -2160, -2130, -2105, -2085, -2075, -2095, -2120, -2140, -2170, -2210, -2260, -2330, -2390, -2434},
            {-2438, -2435, -2430, -2426, -2421, -2415, -2408, -2400, -2394, -2388, -2382, -2375, -2355, -2310, -2250, -2190, -2140, -2110, -2090, -2070, -2060, -2080, -2110, -2130, -2160, -2200, -2250, -2320, -2380, -2432},
            {-2440, -2437, -2433, -2428, -2422, -2415, -2405, -2395, -2388, -2380, -2375, -2365, -2335, -2280, -2220, -2170, -2120, -2095, -2080, -2065, -2050, -2070, -2100, -2125, -2155, -2195, -2245, -2315, -2375, -2430},
            {-2442, -2439, -2435, -2430, -2424, -2416, -2406, -2392, -2382, -2374, -2368, -2355, -2315, -2260, -2200, -2150, -2110, -2085, -2070, -2060, -2040, -2060, -2090, -2120, -2150, -2190, -2240, -2310, -2370, -2428},
            {-2444, -2441, -2438, -2432, -2425, -2418, -2408, -2395, -2385, -2376, -2365, -2345, -2300, -2240, -2185, -2135, -2095, -2075, -2065, -2055, -2035, -2050, -2080, -2110, -2145, -2185, -2235, -2305, -2365, -2425},
            {-2445, -2443, -2440, -2435, -2428, -2420, -2410, -2398, -2388, -2378, -2360, -2335, -2285, -2225, -2170, -2120, -2080, -2065, -2060, -2050, -2030, -2045, -2070, -2105, -2140, -2180, -2230, -2300, -2360, -2422},
            {-2446, -2444, -2442, -2438, -2430, -2422, -2412, -2400, -2390, -2375, -2350, -2320, -2270, -2210, -2155, -2105, -2070, -2055, -2050, -2045, -2025, -2040, -2065, -2100, -2135, -2175, -2225, -2295, -2355, -2420},
            {-2447, -2445, -2443, -2439, -2432, -2424, -2414, -2402, -2388, -2370, -2340, -2305, -2255, -2195, -2140, -2090, -2060, -2045, -2040, -2035, -2020, -2035, -2060, -2095, -2130, -2170, -2220, -2290, -2350, -2418},
            {-2448, -2446, -2444, -2440, -2434, -2426, -2415, -2404, -2385, -2365, -2330, -2290, -2240, -2180, -2125, -2080, -2050, -2035, -2030, -2025, -2015, -2030, -2055, -2090, -2125, -2165, -2215, -2285, -2345, -2415},
            {-2449, -2447, -2445, -2441, -2435, -2427, -2416, -2405, -2382, -2360, -2320, -2275, -2225, -2165, -2110, -2070, -2040, -2025, -2020, -2015, -2010, -2025, -2050, -2085, -2120, -2160, -2210, -2280, -2340, -2412},
            {-2450, -2448, -2446, -2442, -2436, -2428, -2417, -2405, -2380, -2355, -2310, -2260, -2210, -2150, -2095, -2060, -2030, -2015, -2010, -2005, -2005, -2020, -2045, -2080, -2115, -2155, -2205, -2275, -2335, -2410},
            {-2450, -2449, -2446, -2443, -2437, -2429, -2418, -2404, -2378, -2350, -2300, -2245, -2195, -2135, -2080, -2050, -2020, -2005, -2000, -1995, -2000, -2015, -2040, -2075, -2110, -2150, -2200, -2270, -2330, -2408},
            {-2451, -2449, -2447, -2444, -2438, -2430, -2419, -2402, -2375, -2345, -2290, -2230, -2180, -2120, -2065, -2040, -2010, -1995, -1990, -1985, -1995, -2010, -2035, -2070, -2105, -2145, -2195, -2265, -2325, -2405},
            {-2452, -2450, -2448, -2445, -2439, -2431, -2420, -2400, -2372, -2340, -2280, -2215, -2165, -2105, -2050, -2030, -2000, -1985, -1980, -1975, -1990, -2005, -2030, -2065, -2100, -2140, -2190, -2260, -2320, -2402},
            {-2452, -2450, -2448, -2445, -2440, -2432, -2420, -2398, -2370, -2335, -2270, -2200, -2150, -2090, -2035, -2020, -1990, -1975, -1970, -1965, -1980, -2000, -2025, -2060, -2095, -2135, -2185, -2255, -2315, -2400},
            {-2453, -2451, -2449, -2446, -2441, -2433, -2421, -2396, -2368, -2330, -2260, -2185, -2135, -2075, -2020, -2010, -1980, -1965, -1960, -1955, -1975, -1995, -2020, -2055, -2090, -2130, -2180, -2250, -2310, -2398}
        };

        minElevation = -2455.0f;
        maxElevation = -1955.0f;

        for (int r = 0; r < TOTAL_ROWS; ++r) {
            for (int c = 0; c < TOTAL_COLUMNS; ++c) {
                elevationMatrix[r][c] = baseDEM[r][c];
                if (elevationMatrix[r][c] < minElevation) minElevation = elevationMatrix[r][c];
                if (elevationMatrix[r][c] > maxElevation) maxElevation = elevationMatrix[r][c];
            }
        }
    }


    void preprocessTerrain() {
        const int dr[4] = {-1, 1, 0, 0};
        const int dc[4] = {0, 0, -1, 1};

        for (int r = 0; r < TOTAL_ROWS; ++r) {
            for (int c = 0; c < TOTAL_COLUMNS; ++c) {
                float currentElev = elevationMatrix[r][c];
                float maxDiff = 0.0f;
                float sumDiff = 0.0f;
                int validNeighbors = 0;

                for (int i = 0; i < 4; ++i) {
                    int nr = r + dr[i];
                    int nc = c + dc[i];
                    if (nr >= 0 && nr < TOTAL_ROWS && nc >= 0 && nc < TOTAL_COLUMNS) {
                        float diff = std::abs(elevationMatrix[nr][nc] - currentElev);
                        if (diff > maxDiff) maxDiff = diff;
                        sumDiff += diff * diff;
                        validNeighbors++;
                    }
                }
                slopeMatrix[r][c] = maxDiff;
                roughnessMatrix[r][c] = (validNeighbors > 0) ? std::sqrt(sumDiff / validNeighbors) : 0.0f;
            }
        }

        for (int r = 0; r < TOTAL_ROWS; ++r) {
            for (int c = 0; c < TOTAL_COLUMNS; ++c) {
                float slope = slopeMatrix[r][c];
                float roughness = roughnessMatrix[r][c];


                bool isValleyPass = ((r >= 2 && r <= 4 && c >= 12 && c <= 20) ||
                                     (r >= 9 && r <= 11 && c >= 12 && c <= 20) ||
                                     (r >= 16 && r <= 17 && c >= 12 && c <= 20));
                bool isStartArea = (r <= 1 && c <= 1);

                if (!isStartArea && !isValleyPass && (slope >= 42.0f || roughness >= 38.0f)) {
                    terrainGrid[r][c] = TerrainType::OBSTACLE;
                    traversalCostGrid[r][c] = 9999.0f;
                } else if (slope >= 22.0f || roughness >= 18.0f) {
                    terrainGrid[r][c] = TerrainType::ROUGH_REGOLITH;
                    traversalCostGrid[r][c] = 3.4f + (slope / 20.0f);
                } else if (slope >= 10.0f || roughness >= 8.0f) {
                    terrainGrid[r][c] = TerrainType::MODERATE_SLOPE;
                    traversalCostGrid[r][c] = 1.8f + (slope / 30.0f);
                } else {
                    terrainGrid[r][c] = TerrainType::PLAINS;
                    traversalCostGrid[r][c] = 1.0f + (slope / 50.0f);
                }
            }
        }


        addMartianCrater(6, 17, 2);
        addMartianCrater(14, 8, 2);


        terrainGrid[0][0] = TerrainType::PLAINS;
        terrainGrid[0][1] = TerrainType::PLAINS;
        terrainGrid[1][0] = TerrainType::PLAINS;
        traversalCostGrid[0][0] = 1.0f;
        traversalCostGrid[0][1] = 1.0f;
        traversalCostGrid[1][0] = 1.0f;

        std::cout << "[TerrainEngine] Real Mars DEM tile loaded and preprocessed ("
                  << TOTAL_ROWS * TOTAL_COLUMNS << " cells in O(N) time)." << std::endl;
    }

    void addMartianCrater(int centerR, int centerC, int radius) {
        for (int r = centerR - radius; r <= centerR + radius; ++r) {
            for (int c = centerC - radius; c <= centerC + radius; ++c) {
                if (r >= 0 && r < TOTAL_ROWS && c >= 0 && c < TOTAL_COLUMNS) {
                    int distSq = (r - centerR) * (r - centerR) + (c - centerC) * (c - centerC);
                    if (distSq == radius * radius) {
                        terrainGrid[r][c] = TerrainType::OBSTACLE;
                        traversalCostGrid[r][c] = 9999.0f;
                    } else if (distSq < radius * radius) {
                        terrainGrid[r][c] = TerrainType::ROUGH_REGOLITH;
                        traversalCostGrid[r][c] = 3.8f;
                    }
                }
            }
        }
    }

    bool isPassable(int r, int c) const {
        if (r < 0 || r >= TOTAL_ROWS || c < 0 || c >= TOTAL_COLUMNS) return false;
        return terrainGrid[r][c] != TerrainType::OBSTACLE;
    }

    float getElevation(int r, int c) const {
        if (r < 0 || r >= TOTAL_ROWS || c < 0 || c >= TOTAL_COLUMNS) return -2450.0f;
        return elevationMatrix[r][c];
    }

    float getNormalizedElevation(int r, int c) const {
        float elev = getElevation(r, c);
        float norm = (elev - minElevation) / (maxElevation - minElevation + 0.001f);
        return clampValue(norm, 0.0f, 1.0f);
    }

    float getSlope(int r, int c) const {
        if (r < 0 || r >= TOTAL_ROWS || c < 0 || c >= TOTAL_COLUMNS) return 0.0f;
        return slopeMatrix[r][c];
    }

    float getRoughness(int r, int c) const {
        if (r < 0 || r >= TOTAL_ROWS || c < 0 || c >= TOTAL_COLUMNS) return 0.0f;
        return roughnessMatrix[r][c];
    }

    TerrainType getTerrainType(int r, int c) const {
        if (r < 0 || r >= TOTAL_ROWS || c < 0 || c >= TOTAL_COLUMNS) return TerrainType::OBSTACLE;
        return terrainGrid[r][c];
    }

    float getCellCost(int r, int c) const {
        if (r < 0 || r >= TOTAL_ROWS || c < 0 || c >= TOTAL_COLUMNS) return 9999.0f;
        return traversalCostGrid[r][c];
    }

    float getEdgeTraversalEnergy(Point2D from, Point2D to) const {
        if (!isPassable(to.getRow(), to.getColumn())) return 9999.0f;
        float baseCost = getCellCost(to.getRow(), to.getColumn());
        float elevDiff = findMax(0.0f, getElevation(to.getRow(), to.getColumn()) - getElevation(from.getRow(), from.getColumn()));
        return baseCost + (elevDiff * 0.12f);
    }

    sf::Color getCellVisualColor(int r, int c) const {
        TerrainType type = getTerrainType(r, c);
        if (type == TerrainType::OBSTACLE) {
            return COLOR_TERRAIN_OBSTACLE;
        }

        float normElev = getNormalizedElevation(r, c);
        sf::Color base;
        if (type == TerrainType::PLAINS) {
            base = COLOR_TERRAIN_PLAINS;
        } else if (type == TerrainType::MODERATE_SLOPE) {
            base = COLOR_TERRAIN_SLOPE;
        } else {
            base = COLOR_TERRAIN_ROUGH;
        }

        uint8_t rCol = static_cast<uint8_t>(clampValue(base.r + normElev * 40.0f, 0.0f, 255.0f));
        uint8_t gCol = static_cast<uint8_t>(clampValue(base.g + normElev * 25.0f, 0.0f, 255.0f));
        uint8_t bCol = static_cast<uint8_t>(clampValue(base.b + normElev * 15.0f, 0.0f, 255.0f));
        return sf::Color(rCol, gCol, bCol);
    }
};

#endif
