#ifndef NEURAL_NETWORK_HPP
#define NEURAL_NETWORK_HPP

#include "Types.hpp"
#include <iostream>
#include <cmath>
#include <vector>

// simple vector class for linear algebra without external libraries
class MathVector {
public:
    float* data;
    size_t len;

    MathVector() : data(nullptr), len(0) {}

    explicit MathVector(size_t size, float initialValue = 0.0f) : len(size) {
        data = new float[len];
        for (size_t i = 0; i < len; ++i) data[i] = initialValue;
    }

    MathVector(std::initializer_list<float> list) : len(list.size()) {
        data = new float[len];
        size_t idx = 0;
        for (float val : list) data[idx++] = val;
    }

    ~MathVector() {
        delete[] data;
    }

    MathVector(const MathVector& other) : len(other.len) {
        data = new float[len];
        for (size_t i = 0; i < len; ++i) data[i] = other.data[i];
    }

    MathVector& operator=(const MathVector& other) {
        if (this != &other) {
            delete[] data;
            len = other.len;
            data = new float[len];
            for (size_t i = 0; i < len; ++i) data[i] = other.data[i];
        }
        return *this;
    }

    size_t size() const { return len; }
    float& operator[](size_t index) { return data[index]; }
    const float& operator[](size_t index) const { return data[index]; }

    MathVector operator+(const MathVector& other) const {
        MathVector res(len);
        for (size_t i = 0; i < len; ++i) res[i] = data[i] + other[i];
        return res;
    }

    MathVector operator-(const MathVector& other) const {
        MathVector res(len);
        for (size_t i = 0; i < len; ++i) res[i] = data[i] - other[i];
        return res;
    }

    MathVector operator*(float scalar) const {
        MathVector res(len);
        for (size_t i = 0; i < len; ++i) res[i] = data[i] * scalar;
        return res;
    }

    float dot(const MathVector& other) const {
        float sum = 0.0f;
        for (size_t i = 0; i < len; ++i) sum += data[i] * other[i];
        return sum;
    }

    std::vector<float> toVector() const {
        std::vector<float> v(len);
        for (size_t i = 0; i < len; ++i) v[i] = data[i];
        return v;
    }
};

// simple matrix class for network weights
class MathMatrix {
public:
    size_t rows;
    size_t cols;
    float** data;

    MathMatrix() : rows(0), cols(0), data(nullptr) {}

    MathMatrix(size_t r, size_t c, float initialValue = 0.0f) : rows(r), cols(c) {
        data = new float*[rows];
        for (size_t i = 0; i < rows; ++i) {
            data[i] = new float[cols];
            for (size_t j = 0; j < cols; ++j) data[i][j] = initialValue;
        }
    }

    ~MathMatrix() {
        if (data) {
            for (size_t i = 0; i < rows; ++i) delete[] data[i];
            delete[] data;
        }
    }

    MathMatrix(const MathMatrix& other) : rows(other.rows), cols(other.cols) {
        data = new float*[rows];
        for (size_t i = 0; i < rows; ++i) {
            data[i] = new float[cols];
            for (size_t j = 0; j < cols; ++j) data[i][j] = other.data[i][j];
        }
    }

    MathMatrix& operator=(const MathMatrix& other) {
        if (this != &other) {
            if (data) {
                for (size_t i = 0; i < rows; ++i) delete[] data[i];
                delete[] data;
            }
            rows = other.rows;
            cols = other.cols;
            data = new float*[rows];
            for (size_t i = 0; i < rows; ++i) {
                data[i] = new float[cols];
                for (size_t j = 0; j < cols; ++j) data[i][j] = other.data[i][j];
            }
        }
        return *this;
    }

    void randomize(float minVal = -0.5f, float maxVal = 0.5f, unsigned int seed = 42) {
        // simple lcg generator for reproducibility
        unsigned long s = seed;
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                s = s * 1664525u + 1013904223u;
                float r = static_cast<float>(s & 0xFFFF) / 65535.0f;
                data[i][j] = minVal + r * (maxVal - minVal);
            }
        }
    }

    MathVector multiply(const MathVector& vec) const {
        MathVector result(rows, 0.0f);
        for (size_t i = 0; i < rows; ++i) {
            float sum = 0.0f;
            for (size_t j = 0; j < cols; ++j) {
                sum += data[i][j] * vec[j];
            }
            result[i] = sum;
        }
        return result;
    }
};

struct ExcavationPrediction {
    float offsetRow;
    float offsetCol;
    float predictedDepthCm;
    std::vector<float> inputFeatures;
    std::vector<float> hiddenActivations;
};

// 3-layer neural network (4 -> 6 -> 3) predicting drill offset and depth
class ExcavationNeuralNetwork {
private:
    MathMatrix W1; // hidden weights (6x4)
    MathVector b1; // hidden biases (6)
    MathMatrix W2; // output weights (3x6)
    MathVector b2; // output biases (3)

    static float sigmoid(float x) {
        return 1.0f / (1.0f + std::exp(-clampValue(x, -20.0f, 20.0f)));
    }
    static float sigmoidDerivative(float outSigmoid) {
        return outSigmoid * (1.0f - outSigmoid);
    }
    static float tanhAct(float x) {
        return std::tanh(clampValue(x, -10.0f, 10.0f));
    }
    static float tanhDerivative(float outTanh) {
        return 1.0f - (outTanh * outTanh);
    }

public:
    ExcavationNeuralNetwork() : W1(6, 4), b1(6, 0.0f), W2(3, 6), b2(3, 0.0f) {
        W1.randomize(-0.6f, 0.6f, 101);
        W2.randomize(-0.6f, 0.6f, 202);
        trainOnMarsGeologicalBaseline();
    }

    ExcavationPrediction forward(const MathVector& inputs) const {
        // hidden layer
        MathVector z1 = W1.multiply(inputs) + b1;
        MathVector a1(6);
        for (size_t i = 0; i < 6; ++i) {
            a1[i] = tanhAct(z1[i]);
        }

        // output layer
        MathVector z2 = W2.multiply(a1) + b2;
        MathVector a2(3);
        for (size_t i = 0; i < 3; ++i) {
            a2[i] = sigmoid(z2[i]);
        }

        ExcavationPrediction pred;
        pred.offsetRow = (a2[0] - 0.5f) * 1.4f;
        pred.offsetCol = (a2[1] - 0.5f) * 1.4f;
        pred.predictedDepthCm = 8.0f + a2[2] * 32.0f;
        pred.inputFeatures = inputs.toVector();
        pred.hiddenActivations = a1.toVector();
        return pred;
    }

    void trainSample(const MathVector& inputs, const MathVector& targets, float lr = 0.08f) {
        MathVector z1 = W1.multiply(inputs) + b1;
        MathVector a1(6);
        for (size_t i = 0; i < 6; ++i) a1[i] = tanhAct(z1[i]);

        MathVector z2 = W2.multiply(a1) + b2;
        MathVector a2(3);
        for (size_t i = 0; i < 3; ++i) a2[i] = sigmoid(z2[i]);

        MathVector delta2(3);
        for (size_t i = 0; i < 3; ++i) {
            float err = a2[i] - targets[i];
            delta2[i] = err * sigmoidDerivative(a2[i]);
        }

        MathVector delta1(6, 0.0f);
        for (size_t j = 0; j < 6; ++j) {
            float sum = 0.0f;
            for (size_t i = 0; i < 3; ++i) {
                sum += W2.data[i][j] * delta2[i];
            }
            delta1[j] = sum * tanhDerivative(a1[j]);
        }

        // update output weights
        for (size_t i = 0; i < 3; ++i) {
            b2[i] -= lr * delta2[i];
            for (size_t j = 0; j < 6; ++j) {
                W2.data[i][j] -= lr * delta2[i] * a1[j];
            }
        }

        // update hidden weights
        for (size_t j = 0; j < 6; ++j) {
            b1[j] -= lr * delta1[j];
            for (size_t k = 0; k < 4; ++k) {
                W1.data[j][k] -= lr * delta1[j] * inputs[k];
            }
        }
    }

    void trainOnMarsGeologicalBaseline() {
        struct TrainingSample {
            MathVector in;
            MathVector out;
        };

        TrainingSample dataset[6] = {
            {{0.15f, 0.10f, 0.08f, 0.85f}, {0.45f, 0.55f, 0.88f}},
            {{0.30f, 0.25f, 0.35f, 0.70f}, {0.50f, 0.48f, 0.65f}},
            {{0.65f, 0.55f, 0.60f, 0.40f}, {0.60f, 0.40f, 0.32f}},
            {{0.20f, 0.12f, 0.15f, 0.92f}, {0.42f, 0.58f, 0.90f}},
            {{0.80f, 0.70f, 0.80f, 0.25f}, {0.68f, 0.35f, 0.20f}},
            {{0.40f, 0.20f, 0.25f, 0.78f}, {0.48f, 0.52f, 0.75f}}
        };

        for (int epoch = 0; epoch < 250; ++epoch) {
            for (int s = 0; s < 6; ++s) {
                trainSample(dataset[s].in, dataset[s].out, 0.09f);
            }
        }

        std::cout << "[NeuralNetwork] First-principles NN trained with custom backprop (4 -> 6 -> 3)." << std::endl;
    }
};

#endif // NEURAL_NETWORK_HPP
