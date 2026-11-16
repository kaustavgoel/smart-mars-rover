#ifndef MAX_HEAP_HPP
#define MAX_HEAP_HPP

#include "Types.hpp"
#include <vector>
#include <stdexcept>
#include <iostream>
#include <algorithm>

/**
 * @brief Custom Max-Heap prioritizing candidate Mars excavation sites by Scientific Value.
 * Supports O(log N) insertion and extraction, keeping highest-value scientific targets at the root.
 */
class MaxHeap {
private:
    std::vector<SiteData> heapArray;

    int parent(int i) const { return (i - 1) / 2; }
    int leftChild(int i) const { return 2 * i + 1; }
    int rightChild(int i) const { return 2 * i + 2; }

    void bubbleUp(int index) {
        while (index > 0 && heapArray[index].scienceValue > heapArray[parent(index)].scienceValue) {
            std::swap(heapArray[index], heapArray[parent(index)]);
            index = parent(index);
        }
    }

    void bubbleDown(int index) {
        int maxIndex = index;
        int left = leftChild(index);
        int right = rightChild(index);
        int n = static_cast<int>(heapArray.size());

        if (left < n && heapArray[left].scienceValue > heapArray[maxIndex].scienceValue) {
            maxIndex = left;
        }

        if (right < n && heapArray[right].scienceValue > heapArray[maxIndex].scienceValue) {
            maxIndex = right;
        }

        if (index != maxIndex) {
            std::swap(heapArray[index], heapArray[maxIndex]);
            bubbleDown(maxIndex);
        }
    }

public:
    MaxHeap() = default;

    void insert(const SiteData& site) {
        heapArray.push_back(site);
        bubbleUp(static_cast<int>(heapArray.size()) - 1);
    }

    SiteData peekMax() const {
        if (isEmpty()) {
            throw std::runtime_error("Heap is empty!");
        }
        return heapArray[0];
    }

    SiteData extractMax() {
        if (isEmpty()) {
            throw std::runtime_error("Heap is empty!");
        }
        SiteData maxSite = heapArray[0];
        heapArray[0] = heapArray.back();
        heapArray.pop_back();

        if (!heapArray.empty()) {
            bubbleDown(0);
        }
        return maxSite;
    }

    bool removeSiteById(int siteId) {
        for (size_t i = 0; i < heapArray.size(); ++i) {
            if (heapArray[i].siteId == siteId) {
                heapArray[i] = heapArray.back();
                heapArray.pop_back();
                if (i < heapArray.size()) {
                    bubbleDown(static_cast<int>(i));
                    bubbleUp(static_cast<int>(i));
                }
                return true;
            }
        }
        return false;
    }

    bool isEmpty() const {
        return heapArray.empty();
    }

    size_t size() const {
        return heapArray.size();
    }

    void clear() {
        heapArray.clear();
    }

    const std::vector<SiteData>& getElements() const {
        return heapArray;
    }
};

#endif // MAX_HEAP_HPP
