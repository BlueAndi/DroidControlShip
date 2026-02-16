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
 * @brief  Leaky Rectified Linear Unit (Leaky ReLU) activation function
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef ACTIVATION_FUNCTION_LEAKY_RELU_H
#define ACTIVATION_FUNCTION_LEAKY_RELU_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "IActivationFunction.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Leaky Rectified Linear Unit (Leaky ReLU) activation function.
 * 
 * LeakyReLU(x) = x, if x > 0; alpha * x, otherwise
 */
class ActivationFunctionLeakyReLU : public IActivationFunction
{
public:
    /**
     * Default constructor.
     * 
     * If not specified, the slope for x < 0 is 1.0f.
     * 
     * @param[in] alpha Slope for x < 0.
     */
    ActivationFunctionLeakyReLU(float alpha = 1.0f) :
        m_alpha(alpha)
    {
    }

    /**
     * Default destructor.
     */
    ~ActivationFunctionLeakyReLU() final = default;

    /**
     * Apply the Leaky ReLU activation function on the given matrix.
     *
     * @param[in,out] matrix Matrix to apply the activation function on.
     */
    void apply(Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic, 1>> matrix) final
    {
        for (int idx = 0; idx < matrix.size(); ++idx)
        {
            if (0.0f > matrix(idx))
            {
                matrix(idx) = m_alpha * matrix(idx);
            }
        }
    }

private:

    /** Slope for x < 0. */
    float m_alpha;
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* ACTIVATION_FUNCTION_LEAKY_RELU_H */
/** @} */
