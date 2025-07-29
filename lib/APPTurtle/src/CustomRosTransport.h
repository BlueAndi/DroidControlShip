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
 * @brief  Custom Micro-ROS transport class selector.
 * @author Norbert Schulz <github@schulznorbert.de>
 *
 * @addtogroup Application
 *
 * @{
 */
#ifndef CUSTOM_ROS_TRANSPORT_H
#define CUSTOM_ROS_TRANSPORT_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/
#if defined(TRANSPORT_USE_TCP) && TRANSPORT_USE_TCP != 0
#include "Transports/CustomRosTransportTcp.h"
#else
#include "Transports/CustomRosTransportUdp.h"
#endif /* else defined(TRANSPORT_USE_TCP) && TRANSPORT_USE_TCP != 0 */

#endif /* CUSTOM_ROS_TRANSPORT_H */
/** @} */
