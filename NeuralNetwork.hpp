#ifndef NEURAL_NETWORK_HPP
#define NEURAL_NETWORK_HPP

#include <vector>
#include <cmath>
#include <random>
#include <iostream>
#include <iomanip>
#include <cassert>

/**
 * @brief Custom Vector class built from first principles for linear algebra without third-party ML libraries.
 */
class MathVector {
public:
    std::vector<float> elements;

    MathVector() = default;
    explicit MathVector(size_t size, float initialValue = 0.0f) : elements(size, initialValue) {}
    MathVector(std::initializer_list<float> list) : elements(list) {}

    size_t size() const { return elements.size(); }
    float& operator[](size_t index) { return elements[index]; }
    const float& operator[](size_t index) const { return elements[index]; }

    MathVector operator+(const MathVector& other) const {
        assert(size() == other.size());
        MathVector res(size());
        for (size_t i = 0; i < size(); ++i) {
            res[i] = elements[i] + other[i];
        }
        return res;
    }

    MathVector operator-(const MathVector& other) const {
        assert(size() == other.size());
        MathVector res(size());
        for (size_t i = 0; i < size(); ++i) {
            res[i] = elements[i] - other[i];
        }
        return res;
    }

    MathVector operator*(float scalar) const {
        MathVector res(size());
        for (size_t i = 0; i < size(); ++i) {
            res[i] = elements[i] * scalar;
        }
        return res;
    }

    float dot(const MathVector& other) const {
        assert(size() == other.size());
        float sum = 0.0f;
        for (size_t i = 0; i < size(); ++i) {
            sum += elements[i] * other[i];
        }
        return sum;
    }
};

/**
 * @brief Custom Matrix class built from first principles for neural network weight tensors.
 */
class MathMatrix {
public:
    size_t rows;
    size_t cols;
    std::vector<std::vector<float>> data;

    MathMatrix() : rows(0), cols(0) {}
    MathMatrix(size_t r, size_t c, float initialValue = 0.0f)
        : rows(r), cols(c), data(r, std::vector<float>(c, initialValue)) {}

    void randomize(float minVal = -0.5f, float maxVal = 0.5f, unsigned int seed = 42) {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> dist(minVal, maxVal);
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                data[i][j] = dist(rng);
            }
        }
    }

    MathVector multiply(const MathVector& vec) const {
        assert(cols == vec.size());
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
    float offsetRow;          // Sub-cell coordinate adjustment [-0.8, +0.8]
    float offsetCol;          // Sub-cell coordinate adjustment [-0.8, +0.8]
    float predictedDepthCm;   // Drill depth recommendation [5.0 to 45.0 cm]
    std::vector<float> inputFeatures;
    std::vector<float> hiddenActivations;
};

/**
 * @brief Feed-Forward Neural Network from first principles (No external ML libraries).
 * Architecture: 4 Inputs (Elevation, Slope, Roughness, Mineral Index)
 *            -> 6 Hidden Neurons (Tanh activation)
 *            -> 3 Output Neurons (Sigmoid / Linear for excavation delta and depth).
 * Features custom backpropagation gradient descent optimizer.
 */
class ExcavationNeuralNetwork {
private:
    MathMatrix W1; // Hidden weights (6 x 4)
    MathVector b1; // Hidden biases (6)
    MathMatrix W2; // Output weights (3 x 6)
    MathVector b2; // Output biases (3)

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

    /**
     * @brief Forward propagation pass.
     */
    ExcavationPrediction forward(const MathVector& inputs) const {
        assert(inputs.size() == 4);

        // Hidden Layer: z1 = W1 * x + b1, a1 = tanh(z1)
        MathVector z1 = W1.multiply(inputs) + b1;
        MathVector a1(6);
        for (size_t i = 0; i < 6; ++i) {
            a1[i] = tanhAct(z1[i]);
        }

        // Output Layer: z2 = W2 * a1 + b2
        MathVector z2 = W2.multiply(a1) + b2;
        MathVector a2(3);
        for (size_t i = 0; i < 3; ++i) {
            a2[i] = sigmoid(z2[i]); // Outputs normalized in [0, 1]
        }

        ExcavationPrediction pred;
        // Map [0, 1] outputs to domain physical coordinates:
        pred.offsetRow = (a2[0] - 0.5f) * 1.4f; // Delta row offset in [-0.7, +0.7]
        pred.offsetCol = (a2[1] - 0.5f) * 1.4f; // Delta col offset in [-0.7, +0.7]
        pred.predictedDepthCm = 8.0f + a2[2] * 32.0f; // Drill depth in range [8.0cm, 40.0cm]

        pred.inputFeatures = inputs.elements;
        pred.hiddenActivations = a1.elements;
        return pred;
    }

    /**
     * @brief Backpropagation training loop using synthetic Mars core sample truth dataset.
     */
    void trainSample(const MathVector& inputs, const MathVector& targets, float lr = 0.08f) {
        // Forward pass
        MathVector z1 = W1.multiply(inputs) + b1;
        MathVector a1(6);
        for (size_t i = 0; i < 6; ++i) a1[i] = tanhAct(z1[i]);

        MathVector z2 = W2.multiply(a1) + b2;
        MathVector a2(3);
        for (size_t i = 0; i < 3; ++i) a2[i] = sigmoid(z2[i]);

        // Output error & delta: dL/da2 * da2/dz2
        MathVector delta2(3);
        for (size_t i = 0; i < 3; ++i) {
            float error = a2[i] - targets[i];
            delta2[i] = error * sigmoidDerivative(a2[i]);
        }

        // Hidden layer delta: (W2^T * delta2) * da1/dz1
        MathVector delta1(6, 0.0f);
        for (size_t j = 0; j < 6; ++j) {
            float sum = 0.0f;
            for (size_t i = 0; i < 3; ++i) {
                sum += W2.data[i][j] * delta2[i];
            }
            delta1[j] = sum * tanhDerivative(a1[j]);
        }

        // Gradient update for W2 and b2
        for (size_t i = 0; i < 3; ++i) {
            b2[i] -= lr * delta2[i];
            for (size_t j = 0; j < 6; ++j) {
                W2.data[i][j] -= lr * delta2[i] * a1[j];
            }
        }

        // Gradient update for W1 and b1
        for (size_t j = 0; j < 6; ++j) {
            b1[j] -= lr * delta1[j];
            for (size_t k = 0; k < 4; ++k) {
                W1.data[j][k] -= lr * delta1[j] * inputs[k];
            }
        }
    }

    /**
     * @brief Trains the network on Martian geological core excavation ground truth patterns.
     */
    void trainOnMarsGeologicalBaseline() {
        // Training data: [NormElev, NormSlope, NormRoughness, MineralSig] -> [TargetNormDx, TargetNormDy, TargetNormDepth]
        struct TrainingSample {
            MathVector in;
            MathVector out;
        };

        std::vector<TrainingSample> dataset = {
            {{0.15f, 0.10f, 0.08f, 0.85f}, {0.45f, 0.55f, 0.88f}}, // River delta clay: deep core
            {{0.30f, 0.25f, 0.35f, 0.70f}, {0.50f, 0.48f, 0.65f}}, // Sedimentary terrace: medium core
            {{0.65f, 0.55f, 0.60f, 0.40f}, {0.60f, 0.40f, 0.32f}}, // Rough basalt ridge: shallow core
            {{0.20f, 0.12f, 0.15f, 0.92f}, {0.42f, 0.58f, 0.90f}}, // Carbonate lakebed: deep core
            {{0.80f, 0.70f, 0.80f, 0.25f}, {0.68f, 0.35f, 0.20f}}, // Crater wall boulders: minimal depth
            {{0.40f, 0.20f, 0.25f, 0.78f}, {0.48f, 0.52f, 0.75f}}  // Ancient river channel: medium-deep
        };

        // Train for 250 epochs
        for (int epoch = 0; epoch < 250; ++epoch) {
            for (const auto& sample : dataset) {
                trainSample(sample.in, sample.out, 0.09f);
            }
        }

        std::cout << "[NeuralNetwork] First-principles NN trained with custom backprop (4 -> 6 -> 3)." << std::endl;
    }
};

#endif // NEURAL_NETWORK_HPP
