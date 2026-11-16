#ifndef MAX_HEAP_HPP
#define MAX_HEAP_HPP

#include "Types.hpp"
#include <iostream>

// custom max-heap to prioritize science sites by value
class MaxHeap {
private:
    SiteData* heapArray;
    int heapCapacity;
    int heapSize;

    int parent(int i) const { return (i - 1) / 2; }
    int leftChild(int i) const { return 2 * i + 1; }
    int rightChild(int i) const { return 2 * i + 2; }

    void resize() {
        int newCap = heapCapacity * 2;
        SiteData* newArr = new SiteData[newCap];
        for (int i = 0; i < heapSize; ++i) {
            newArr[i] = heapArray[i];
        }
        delete[] heapArray;
        heapArray = newArr;
        heapCapacity = newCap;
    }

    void bubbleUp(int index) {
        while (index > 0 && heapArray[index].scienceValue > heapArray[parent(index)].scienceValue) {
            customSwap(heapArray[index], heapArray[parent(index)]);
            index = parent(index);
        }
    }

    void bubbleDown(int index) {
        int maxIndex = index;
        int left = leftChild(index);
        int right = rightChild(index);

        if (left < heapSize && heapArray[left].scienceValue > heapArray[maxIndex].scienceValue) {
            maxIndex = left;
        }
        if (right < heapSize && heapArray[right].scienceValue > heapArray[maxIndex].scienceValue) {
            maxIndex = right;
        }

        if (index != maxIndex) {
            customSwap(heapArray[index], heapArray[maxIndex]);
            bubbleDown(maxIndex);
        }
    }

public:
    MaxHeap(int initialCap = 16) : heapCapacity(initialCap), heapSize(0) {
        heapArray = new SiteData[heapCapacity];
    }

    ~MaxHeap() {
        delete[] heapArray;
    }

    // copy constructor
    MaxHeap(const MaxHeap& other) : heapCapacity(other.heapCapacity), heapSize(other.heapSize) {
        heapArray = new SiteData[heapCapacity];
        for (int i = 0; i < heapSize; ++i) {
            heapArray[i] = other.heapArray[i];
        }
    }

    // assignment operator
    MaxHeap& operator=(const MaxHeap& other) {
        if (this != &other) {
            delete[] heapArray;
            heapCapacity = other.heapCapacity;
            heapSize = other.heapSize;
            heapArray = new SiteData[heapCapacity];
            for (int i = 0; i < heapSize; ++i) {
                heapArray[i] = other.heapArray[i];
            }
        }
        return *this;
    }

    void insert(const SiteData& site) {
        if (heapSize >= heapCapacity) {
            resize();
        }
        heapArray[heapSize] = site;
        bubbleUp(heapSize);
        heapSize++;
    }

    SiteData peekMax() const {
        if (isEmpty()) return SiteData();
        return heapArray[0];
    }

    SiteData extractMax() {
        if (isEmpty()) return SiteData();
        SiteData maxSite = heapArray[0];
        heapArray[0] = heapArray[heapSize - 1];
        heapSize--;
        if (heapSize > 0) {
            bubbleDown(0);
        }
        return maxSite;
    }

    bool removeSiteById(int siteId) {
        for (int i = 0; i < heapSize; ++i) {
            if (heapArray[i].siteId == siteId) {
                heapArray[i] = heapArray[heapSize - 1];
                heapSize--;
                if (i < heapSize) {
                    bubbleDown(i);
                    bubbleUp(i);
                }
                return true;
            }
        }
        return false;
    }

    bool isEmpty() const {
        return heapSize == 0;
    }

    int size() const {
        return heapSize;
    }

    void clear() {
        heapSize = 0;
    }
};

#endif // MAX_HEAP_HPP
