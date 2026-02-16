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
 * @brief  Rectified Linear Unit (ReLU) activation function
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef ACTIVATION_FUNCTION_RELU_H
#define ACTIVATION_FUNCTION_RELU_H

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
 * Rectified Linear Unit (ReLU) activation function.
 * 
 * ReLU(x) = max(0, x)
 */
class ActivationFunctionReLU : public IActivationFunction
{
public:

    /**
     * Default constructor.
     */
    ActivationFunctionReLU() = default;

    /**
     * Default destructor.
     */
    ~ActivationFunctionReLU() final = default;

    /**
     * Apply the ReLU activation function on the given matrix.
     *
     * @param[in,out] matrix Matrix to apply the activation function on.
     */
    void apply(Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic, 1>> matrix) final
    {
        for (int idx = 0; idx < matrix.size(); ++idx)
        {
            if (0.0f > matrix(idx))
            {
                matrix(idx) = 0.0f;
            }
        }
    }

};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* ACTIVATION_FUNCTION_RELU_H */
/** @} */
