#ifndef TYPES_HPP
#define TYPES_HPP

#include <SFML/Graphics.hpp>
#include <string>
#include <cmath>
#include "CustomVector.hpp"

// grid dimensions and display sizes
constexpr int TOTAL_ROWS = 20;
constexpr int TOTAL_COLUMNS = 30;
constexpr int CELL_SIZE_PIXELS = 30; // 30x30 pixels per grid cell

constexpr int MAP_WIDTH_PIXELS = TOTAL_COLUMNS * CELL_SIZE_PIXELS; // 900 px
constexpr int MAP_HEIGHT_PIXELS = TOTAL_ROWS * CELL_SIZE_PIXELS;   // 600 px
constexpr int HUD_HEIGHT_PIXELS = 180;

constexpr int WINDOW_WIDTH_PIXELS = MAP_WIDTH_PIXELS;              // 900 px
constexpr int WINDOW_HEIGHT_PIXELS = MAP_HEIGHT_PIXELS + HUD_HEIGHT_PIXELS; // 780 px

// timing
constexpr float AUTOMATIC_MOVE_INTERVAL = 0.08f;
constexpr float MANUAL_MOVE_INTERVAL = 0.10f;
constexpr int MAXIMUM_TRAIL_LENGTH = 80;

// ui color palette
inline const sf::Color COLOR_BACKGROUND(14, 16, 22);
inline const sf::Color COLOR_GRID_LINE(28, 32, 44);
inline const sf::Color COLOR_HUD_BG(18, 22, 30);
inline const sf::Color COLOR_PANEL_BG(22, 28, 38);
inline const sf::Color COLOR_PANEL_BORDER(45, 60, 85);
inline const sf::Color COLOR_ACCENT(70, 160, 245);
inline const sf::Color COLOR_ACCENT_GLOW(120, 200, 255);
inline const sf::Color COLOR_TEXT_MAIN(225, 235, 250);
inline const sf::Color COLOR_TEXT_MUTED(110, 135, 170);
inline const sf::Color COLOR_SAFE(60, 210, 120);
inline const sf::Color COLOR_WARNING(245, 185, 45);
inline const sf::Color COLOR_DANGER(240, 65, 65);
inline const sf::Color COLOR_BATTERY_GREEN(45, 205, 115);
inline const sf::Color COLOR_BATTERY_YELLOW(240, 190, 40);
inline const sf::Color COLOR_BATTERY_RED(240, 60, 60);

// mars terrain colors
inline const sf::Color COLOR_TERRAIN_PLAINS(62, 44, 36);
inline const sf::Color COLOR_TERRAIN_SLOPE(110, 68, 48);
inline const sf::Color COLOR_TERRAIN_ROUGH(165, 82, 52);
inline const sf::Color COLOR_TERRAIN_OBSTACLE(210, 50, 40);
inline const sf::Color COLOR_TERRAIN_OUTLINE(30, 20, 18);

// 2d coordinate template
template <typename T>
class MyCoordinate {
private:
    T rowValue;
    T columnValue;
public:
    MyCoordinate() : rowValue(0), columnValue(0) {}
    MyCoordinate(T r, T c) : rowValue(r), columnValue(c) {}

    T getRow() const { return rowValue; }
    T getColumn() const { return columnValue; }
    void setRow(T r) { rowValue = r; }
    void setColumn(T c) { columnValue = c; }

    bool operator==(const MyCoordinate<T>& other) const {
        return (rowValue == other.rowValue && columnValue == other.columnValue);
    }
    bool operator!=(const MyCoordinate<T>& other) const {
        return !(*this == other);
    }
    bool operator<(const MyCoordinate<T>& other) const {
        if (rowValue != other.rowValue) return rowValue < other.rowValue;
        return columnValue < other.columnValue;
    }

    float distanceTo(const MyCoordinate<T>& other) const {
        float dr = static_cast<float>(rowValue - other.rowValue);
        float dc = static_cast<float>(columnValue - other.columnValue);
        return std::sqrt(dr * dr + dc * dc);
    }

    int manhattanDistance(const MyCoordinate<T>& other) const {
        return std::abs(static_cast<int>(rowValue) - static_cast<int>(other.rowValue)) +
               std::abs(static_cast<int>(columnValue) - static_cast<int>(other.columnValue));
    }
};

using Point2D = MyCoordinate<int>;

// terrain types
enum class TerrainType {
    PLAINS = 0,
    MODERATE_SLOPE = 1,
    ROUGH_REGOLITH = 2,
    OBSTACLE = 3
};

// rover state
enum class RoverState {
    WAITING_IDLE,
    PLANNING_MISSION,
    AUTOMATIC_NAVIGATING,
    EXCAVATING_SITE,
    SUCCESSFULLY_ARRIVED,
    BATTERY_DEPLETED,
    PATH_IS_STUCK,
    MANUALLY_EXPLORING
};

// site data model
struct SiteData {
    int siteId;
    std::string name;
    Point2D coord;
    float scienceValue;
    float excavationEnergy;
    float elevation;
    float slope;
    float roughness;
    float mineralSignature;
    bool isVisited;
    bool isSelectedInKnapsack;

    SiteData()
        : siteId(0), name("Unknown"), coord(0, 0), scienceValue(0.0f),
          excavationEnergy(0.0f), elevation(0.0f), slope(0.0f), roughness(0.0f),
          mineralSignature(0.0f), isVisited(false), isSelectedInKnapsack(false) {}

    SiteData(int id, const std::string& n, Point2D c, float sv, float energy,
             float elev, float slp, float rough, float mineral)
        : siteId(id), name(n), coord(c), scienceValue(sv), excavationEnergy(energy),
          elevation(elev), slope(slp), roughness(rough), mineralSignature(mineral),
          isVisited(false), isSelectedInKnapsack(false) {}
};

// basic utility helpers
template <typename T>
void customSwap(T& a, T& b) {
    T temp = a;
    a = b;
    b = temp;
}

template <typename T>
T findMax(T a, T b) { return (a > b) ? a : b; }

template <typename T>
T findMin(T a, T b) { return (a < b) ? a : b; }

template <typename T>
T clampValue(T val, T low, T high) {
    if (val < low) return low;
    if (val > high) return high;
    return val;
}

inline sf::Color interpolateColor(sf::Color c1, sf::Color c2, float t) {
    t = clampValue<float>(t, 0.0f, 1.0f);
    uint8_t r = static_cast<uint8_t>(c1.r + (c2.r - c1.r) * t);
    uint8_t g = static_cast<uint8_t>(c1.g + (c2.g - c1.g) * t);
    uint8_t b = static_cast<uint8_t>(c1.b + (c2.b - c1.b) * t);
    uint8_t a = static_cast<uint8_t>(c1.a + (c2.a - c1.a) * t);
    return sf::Color(r, g, b, a);
}

inline std::string padLeft(const std::string& str, int width, char pad = '0') {
    if (static_cast<int>(str.size()) >= width) return str;
    return std::string(width - str.size(), pad) + str;
}

#endif // TYPES_HPP
