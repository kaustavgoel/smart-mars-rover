#ifndef CUSTOM_VECTOR_HPP
#define CUSTOM_VECTOR_HPP

#include <cstddef>
#include <initializer_list>


template <typename T>
class CustomVector {
private:
    T* buffer;
    size_t capacityCount;
    size_t elementCount;

    void reallocate(size_t newCapacity) {
        if (newCapacity < elementCount) newCapacity = elementCount;
        if (newCapacity == 0) newCapacity = 2;

        T* newBuffer = new T[newCapacity];
        for (size_t i = 0; i < elementCount; ++i) {
            newBuffer[i] = buffer[i];
        }
        delete[] buffer;
        buffer = newBuffer;
        capacityCount = newCapacity;
    }

public:
    CustomVector() : buffer(nullptr), capacityCount(0), elementCount(0) {}

    explicit CustomVector(size_t initialSize, const T& defaultValue = T())
        : buffer(nullptr), capacityCount(initialSize), elementCount(initialSize) {
        if (capacityCount > 0) {
            buffer = new T[capacityCount];
            for (size_t i = 0; i < elementCount; ++i) {
                buffer[i] = defaultValue;
            }
        }
    }

    CustomVector(std::initializer_list<T> initList)
        : buffer(nullptr), capacityCount(initList.size()), elementCount(initList.size()) {
        if (capacityCount > 0) {
            buffer = new T[capacityCount];
            size_t idx = 0;
            for (const auto& item : initList) {
                buffer[idx++] = item;
            }
        }
    }

    ~CustomVector() {
        delete[] buffer;
        buffer = nullptr;
        capacityCount = 0;
        elementCount = 0;
    }

    CustomVector(const CustomVector& other)
        : buffer(nullptr), capacityCount(other.capacityCount), elementCount(other.elementCount) {
        if (capacityCount > 0) {
            buffer = new T[capacityCount];
            for (size_t i = 0; i < elementCount; ++i) {
                buffer[i] = other.buffer[i];
            }
        }
    }

    CustomVector& operator=(const CustomVector& other) {
        if (this != &other) {
            delete[] buffer;
            capacityCount = other.capacityCount;
            elementCount = other.elementCount;
            buffer = (capacityCount > 0) ? new T[capacityCount] : nullptr;
            for (size_t i = 0; i < elementCount; ++i) {
                buffer[i] = other.buffer[i];
            }
        }
        return *this;
    }

    CustomVector(CustomVector&& other) noexcept
        : buffer(other.buffer), capacityCount(other.capacityCount), elementCount(other.elementCount) {
        other.buffer = nullptr;
        other.capacityCount = 0;
        other.elementCount = 0;
    }

    CustomVector& operator=(CustomVector&& other) noexcept {
        if (this != &other) {
            delete[] buffer;
            buffer = other.buffer;
            capacityCount = other.capacityCount;
            elementCount = other.elementCount;
            other.buffer = nullptr;
            other.capacityCount = 0;
            other.elementCount = 0;
        }
        return *this;
    }

    CustomVector& operator=(std::initializer_list<T> initList) {
        delete[] buffer;
        capacityCount = initList.size();
        elementCount = initList.size();
        buffer = (capacityCount > 0) ? new T[capacityCount] : nullptr;
        size_t idx = 0;
        for (const auto& item : initList) {
            buffer[idx++] = item;
        }
        return *this;
    }

    void push_back(const T& element) {
        if (elementCount >= capacityCount) {
            size_t nextCap = (capacityCount == 0) ? 2 : capacityCount * 2;
            reallocate(nextCap);
        }
        buffer[elementCount++] = element;
    }

    void pop_back() {
        if (elementCount > 0) {
            elementCount--;
        }
    }

    T& operator[](size_t index) {
        return buffer[index];
    }

    const T& operator[](size_t index) const {
        return buffer[index];
    }

    T& front() { return buffer[0]; }
    const T& front() const { return buffer[0]; }
    T& back() { return buffer[elementCount - 1]; }
    const T& back() const { return buffer[elementCount - 1]; }

    size_t size() const { return elementCount; }
    bool empty() const { return elementCount == 0; }

    void clear() {
        elementCount = 0;
    }

    void reserve(size_t newCap) {
        if (newCap > capacityCount) {
            reallocate(newCap);
        }
    }

    void resize(size_t newSize, const T& val = T()) {
        if (newSize > capacityCount) {
            reallocate(newSize);
        }
        for (size_t i = elementCount; i < newSize; ++i) {
            buffer[i] = val;
        }
        elementCount = newSize;
    }

    T* data() { return buffer; }
    const T* data() const { return buffer; }

    T* begin() { return buffer; }
    const T* begin() const { return buffer; }
    T* end() { return buffer + elementCount; }
    const T* end() const { return buffer + elementCount; }
};

#endif
