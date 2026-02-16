/* MIT License
 *
 * Copyright (c) 2023 - 2025 Andreas Merkle <web@blue-andi.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  Neuronal Network (NN) Model with one hidden layer
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef NN_MODEL_H1_H
#define NN_MODEL_H1_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <ArduinoEigen.h>
#include "IActivationFunction.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Convolutional Neural Network Model H1 with one hidden layer.
 */
template<int inputNeurons, int hiddenNeurons, int outputNeurons>
class NNModelH1
{
public:
    /**
     * Default constructor.
     *
     * @param[in] activationFunc Activation function for hidden layer.
     */
    NNModelH1(IActivationFunction& activationFunc) :
        m_activationFunc(activationFunc),
        m_weightsInputHidden(Eigen::Matrix<float, hiddenNeurons, inputNeurons>::Zero()),
        m_biasesHidden(Eigen::Matrix<float, hiddenNeurons, 1>::Zero()),
        m_weightsHiddenOutput(Eigen::Matrix<float, outputNeurons, hiddenNeurons>::Zero()),
        m_biasesOutput(Eigen::Matrix<float, outputNeurons, 1>::Zero())
    {
    }

    /**
     * Parameterized constructor.
     *
     * @param[in] activationFunc        Activation function for hidden layer.
     * @param[in] weightsInputHidden    Weights between input layer and hidden layer.
     * @param[in] biasesHidden          Biases of hidden layer.
     * @param[in] weightsHiddenOutput   Weights between hidden layer and output layer.
     * @param[in] biasesOutput          Biases of output layer.
     */
    NNModelH1(IActivationFunction&                                      activationFunc,
              const Eigen::Matrix<float, hiddenNeurons, inputNeurons>&  weightsInputHidden,
              const Eigen::Matrix<float, hiddenNeurons, 1>&             biasesHidden,
              const Eigen::Matrix<float, outputNeurons, hiddenNeurons>& weightsHiddenOutput,
              const Eigen::Matrix<float, outputNeurons, 1>&             biasesOutput) :
        m_activationFunc(activationFunc),
        m_weightsInputHidden(weightsInputHidden),
        m_biasesHidden(biasesHidden),
        m_weightsHiddenOutput(weightsHiddenOutput),
        m_biasesOutput(biasesOutput)
    {
    }

    /**
     * Copy constructor.
     *
     * @param[in] other Other NN model to copy from.
     */
    NNModelH1(const NNModelH1& other) :
        m_activationFunc(other.m_activationFunc),
        m_weightsInputHidden(other.m_weightsInputHidden),
        m_biasesHidden(other.m_biasesHidden),
        m_weightsHiddenOutput(other.m_weightsHiddenOutput),
        m_biasesOutput(other.m_biasesOutput)
    {
    }

    /**
     * Default destructor.
     */
    ~NNModelH1() = default;

    /**
     * Get input to hidden layer weights.
     *
     * @param[out] weights Weights matrix.
     */
    void getWeightsInputHidden(Eigen::Ref<Eigen::Matrix<float, hiddenNeurons, inputNeurons>> weights) const
    {
        weights = m_weightsInputHidden;
    }

    /**
     * Set input to hidden layer weights.
     *
     * @param[in] weights Weights matrix.
     */
    void setWeightsInputHidden(const Eigen::Matrix<float, hiddenNeurons, inputNeurons>& weights)
    {
        m_weightsInputHidden = weights;
    }

    /**
     * Get hidden layer biases.
     *
     * @param[out] biases Biases matrix.
     */
    void getBiasesHidden(Eigen::Ref<Eigen::Matrix<float, hiddenNeurons, 1>> biases) const
    {
        biases = m_biasesHidden;
    }

    /**
     * Set hidden layer biases.
     *
     * @param[in] biases Biases matrix.
     */
    void setBiasesHidden(const Eigen::Matrix<float, hiddenNeurons, 1>& biases)
    {
        m_biasesHidden = biases;
    }

    /**
     * Get hidden to output layer weights.
     *
     * @param[out] weights Weights matrix.
     */
    void getWeightsHiddenOutput(Eigen::Ref<Eigen::Matrix<float, outputNeurons, hiddenNeurons>> weights) const
    {
        weights = m_weightsHiddenOutput;
    }

    /**
     * Set hidden to output layer weights.
     *
     * @param[in] weights Weights matrix.
     */
    void setWeightsHiddenOutput(const Eigen::Matrix<float, outputNeurons, hiddenNeurons>& weights)
    {
        m_weightsHiddenOutput = weights;
    }

    /**
     * Get output layer biases.
     *
     * @param[out] biases Biases matrix.
     */
    void getBiasesOutput(Eigen::Ref<Eigen::Matrix<float, outputNeurons, 1>> biases) const
    {
        biases = m_biasesOutput;
    }

    /**
     * Set output layer biases.
     *
     * @param[in] biases Biases matrix.
     */
    void setBiasesOutput(const Eigen::Matrix<float, outputNeurons, 1>& biases)
    {
        m_biasesOutput = biases;
    }

    /**
     * Perform a forward pass through the network.
     *
     * @param[in]  inputData   Input data as Eigen vector.
     * @param[out] outputData  Output data as Eigen vector.
     */
    void forward(const Eigen::Ref<const Eigen::Matrix<float, inputNeurons, 1>>& inputData,
                 Eigen::Ref<Eigen::Matrix<float, outputNeurons, 1>>             outputData)
    {
        Eigen::Matrix<float, hiddenNeurons, 1> hiddenLayer;

        /* https://www.quora.com/Why-does-every-node-in-a-layer-use-the-same-activation-function
         */

        /* Hidden layer computation: hidden = activationFunc(weightsInputHidden * input + biasesHidden) */
        hiddenLayer = m_weightsInputHidden * inputData + m_biasesHidden;
        m_activationFunc.apply(hiddenLayer);

        /* Output layer computation: output = weightsHiddenOutput * hidden + biasesOutput */
        outputData = m_weightsHiddenOutput * hiddenLayer + m_biasesOutput;
    }

private:
    /** Activation function for hidden layer. */
    IActivationFunction& m_activationFunc;

    /** Weights between input layer and hidden layer. */
    Eigen::Matrix<float, hiddenNeurons, inputNeurons> m_weightsInputHidden;

    /** Biases of hidden layer. */
    Eigen::Matrix<float, hiddenNeurons, 1> m_biasesHidden;

    /** Weights between hidden layer and output layer. */
    Eigen::Matrix<float, outputNeurons, hiddenNeurons> m_weightsHiddenOutput;

    /** Biases of output layer. */
    Eigen::Matrix<float, outputNeurons, 1> m_biasesOutput;

    /** Disable default constructor. */
    NNModelH1() = delete;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* NN_MODEL_H1_H */
/** @} */
