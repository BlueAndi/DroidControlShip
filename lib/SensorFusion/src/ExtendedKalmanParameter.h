/* MIT License
 *
 * Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>
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
 *
 */
/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 *  @brief  Struct for Parameter used for a Linear Kalman Filter
 *  @author Juliane Kerpe <juliane.kerpe@web.de>
 */

#ifndef EXTENDEDKALMANPARAMETER_H
#define EXTENDEDKALMANPARAMETER_H

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "LinearKalmanParameter.h"
/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Struct of the Sensor Data channel payload. */
typedef struct _ExtendedKalmanParameter : public LinearKalmanParameter
{
    int16_t angleEncoder;
    int16_t turnRate;
} __attribute__((packed)) ExtendedKalmanParameter;

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* EXTENDEDKALMANPARAMETER_H */