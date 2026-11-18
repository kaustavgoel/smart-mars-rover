#ifndef EXCAVATION_LOG_HPP
#define EXCAVATION_LOG_HPP

#include "Types.hpp"
#include <string>
#include <iostream>

struct ExcavationRecord {
    int logIndex;
    int siteId;
    std::string siteName;
    Point2D siteGridCoord;
    float offsetRow;
    float offsetCol;
    float predictedDepthCm;
    float sampleYieldGrams;
    std::string outcomeStatus;
    float batterySpent;
    float missionTimeSeconds;

    ExcavationRecord* prev;
    ExcavationRecord* next;

    ExcavationRecord()
        : logIndex(0), siteId(0), siteName(""), siteGridCoord(0, 0),
          offsetRow(0.0f), offsetCol(0.0f), predictedDepthCm(0.0f),
          sampleYieldGrams(0.0f), outcomeStatus(""), batterySpent(0.0f),
          missionTimeSeconds(0.0f), prev(nullptr), next(nullptr) {}

    ExcavationRecord(int idx, int sId, const std::string& name, Point2D coord,
                     float offR, float offC, float depth, float yld,
                     const std::string& status, float battery, float time)
        : logIndex(idx), siteId(sId), siteName(name), siteGridCoord(coord),
          offsetRow(offR), offsetCol(offC), predictedDepthCm(depth),
          sampleYieldGrams(yld), outcomeStatus(status), batterySpent(battery),
          missionTimeSeconds(time), prev(nullptr), next(nullptr) {}
};

// custom doubly linked list logging every core drilling operation
class ExcavationLog {
private:
    ExcavationRecord* head;
    ExcavationRecord* tail;
    int totalDigCount;

public:
    ExcavationLog() : head(nullptr), tail(nullptr), totalDigCount(0) {}

    ~ExcavationLog() {
        clear();
    }

    void appendRecord(int siteId, const std::string& name, Point2D coord,
                      float offR, float offC, float depth, float yld,
                      const std::string& status, float battery, float time) {
        totalDigCount++;
        ExcavationRecord* newRecord = new ExcavationRecord(
            totalDigCount, siteId, name, coord, offR, offC, depth, yld, status, battery, time
        );

        if (tail == nullptr) {
            head = newRecord;
            tail = newRecord;
        } else {
            tail->next = newRecord;
            newRecord->prev = tail;
            tail = newRecord;
        }
    }

    int size() const {
        return totalDigCount;
    }

    bool isEmpty() const {
        return head == nullptr;
    }

    ExcavationRecord* getHead() const { return head; }
    ExcavationRecord* getTail() const { return tail; }

    CustomVector<ExcavationRecord> getRecentRecords(int maxCount = 4) const {
        CustomVector<ExcavationRecord> records;
        ExcavationRecord* curr = tail;
        while (curr != nullptr && static_cast<int>(records.size()) < maxCount) {
            records.push_back(*curr);
            curr = curr->prev;
        }
        return records;
    }

    void clear() {
        ExcavationRecord* curr = head;
        while (curr != nullptr) {
            ExcavationRecord* nxt = curr->next;
            delete curr;
            curr = nxt;
        }
        head = nullptr;
        tail = nullptr;
        totalDigCount = 0;
    }
};

#endif // EXCAVATION_LOG_HPP
