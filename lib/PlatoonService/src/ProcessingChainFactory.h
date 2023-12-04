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
 */

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  Factory of ProcessingChain.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup PlatoonService
 *
 * @{
 */
#ifndef PROCESSING_CHAIN_FACTORY_H
#define PROCESSING_CHAIN_FACTORY_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>
#include "ProcessingChain.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Factory of ProcessingChain.
 */
class ProcessingChainFactory
{
public:
    /**
     * Get the factory instance.
     *
     * @return Factory instance.
     */
    static ProcessingChainFactory& getInstance();

    /**
     * Create a processing chain. The factory creates the instance, but the user is responsible for deleting it.
     *
     * @return Pointer to a ProcessingChain instance or nullptr if creation failed.
     */
    ProcessingChain* create();

    /**
     * Register ILongitudinalController create function.
     *
     * @param[in] createFunc    Create function.
     */
    void registerLongitudinalControllerCreateFunc(ILongitudinalController::CreateFunc createFunc);

    /**
     * Register ILongitudinalSafetyPolicy create function.
     *
     * @param[in] createFunc    Create function.
     */
    void registerLongitudinalSafetyPolicyCreateFunc(ILongitudinalSafetyPolicy::CreateFunc createFunc);

    /**
     * Register ILateralController create function.
     *
     * @param[in] createFunc    Create function.
     */
    void registerLateralControllerCreateFunc(ILateralController::CreateFunc createFunc);

    /**
     * Register ILateralSafetyPolicy create function.
     *
     * @param[in] createFunc    Create function.
     */
    void registerLateralSafetyPolicyCreateFunc(ILateralSafetyPolicy::CreateFunc createFunc);

private:
    /**
     * Longitudinal controller create function.
     */
    ILongitudinalController::CreateFunc m_longitudinalControllerCreateFunc;

    /**
     * Longitudinal safety policy create function.
     */
    ILongitudinalSafetyPolicy::CreateFunc m_longitudinalSafetyPolicyCreateFunc;

    /**
     * Lateral controller create function.
     */
    ILateralController::CreateFunc m_lateralControllerCreateFunc;

    /**
     * Lateral safety policy create function.
     */
    ILateralSafetyPolicy::CreateFunc m_lateralSafetyPolicyCreateFunc;

    /**
     * Default Constructor.
     */
    ProcessingChainFactory();

    /**
     * Default destructor.
     */
    ~ProcessingChainFactory();

    /**
     * Copy constructor.
     */
    ProcessingChainFactory(const ProcessingChainFactory& factory);

    /**
     * Assignment operator.
     */
    ProcessingChainFactory& operator=(const ProcessingChainFactory& factory);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* PROCESSING_CHAIN_FACTORY_H */
/** @} */
