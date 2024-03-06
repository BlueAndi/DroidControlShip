/* MIT License
 *
 * Copyright (c) 2023 - 2024 Andreas Merkle <web@blue-andi.de>
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
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "ProcessingChainFactory.h"
#include <new>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

ProcessingChainFactory& ProcessingChainFactory::getInstance()
{
    static ProcessingChainFactory instance; /* Idiom. */

    return instance;
}

ProcessingChain* ProcessingChainFactory::create()
{
    ProcessingChain* processingChain = nullptr;

    if ((nullptr != m_longitudinalControllerCreateFunc) && (nullptr != m_longitudinalSafetyPolicyCreateFunc) &&
        (nullptr != m_lateralControllerCreateFunc) && (nullptr != m_lateralSafetyPolicyCreateFunc))
    {
        ILongitudinalController*   longitudinalController   = m_longitudinalControllerCreateFunc();
        ILongitudinalSafetyPolicy* longitudinalSafetyPolicy = m_longitudinalSafetyPolicyCreateFunc();
        ILateralController*        lateralController        = m_lateralControllerCreateFunc();
        ILateralSafetyPolicy*      lateralSafetyPolicy      = m_lateralSafetyPolicyCreateFunc();

        if ((nullptr != longitudinalController) && (nullptr != longitudinalSafetyPolicy) &&
            (nullptr != lateralController) && (nullptr != lateralSafetyPolicy))
        {
            processingChain = new (std::nothrow) ProcessingChain(*longitudinalController, *longitudinalSafetyPolicy,
                                                                 *lateralController, *lateralSafetyPolicy);
        }

        /* One or more instances failed to be instanced. */
        if (nullptr == processingChain)
        {
            /* Prevent zombies. */
            delete longitudinalController;
            delete longitudinalSafetyPolicy;
            delete lateralController;
            delete lateralSafetyPolicy;
        }
    }

    return processingChain;
}

void ProcessingChainFactory::registerLongitudinalControllerCreateFunc(ILongitudinalController::CreateFunc createFunc)
{
    m_longitudinalControllerCreateFunc = createFunc;
}

void ProcessingChainFactory::registerLongitudinalSafetyPolicyCreateFunc(
    ILongitudinalSafetyPolicy::CreateFunc createFunc)
{
    m_longitudinalSafetyPolicyCreateFunc = createFunc;
}

void ProcessingChainFactory::registerLateralControllerCreateFunc(ILateralController::CreateFunc createFunc)
{
    m_lateralControllerCreateFunc = createFunc;
}

void ProcessingChainFactory::registerLateralSafetyPolicyCreateFunc(ILateralSafetyPolicy::CreateFunc createFunc)
{
    m_lateralSafetyPolicyCreateFunc = createFunc;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

ProcessingChainFactory::ProcessingChainFactory() :
    m_longitudinalControllerCreateFunc(nullptr),
    m_longitudinalSafetyPolicyCreateFunc(nullptr),
    m_lateralControllerCreateFunc(nullptr),
    m_lateralSafetyPolicyCreateFunc(nullptr)
{
}

ProcessingChainFactory::~ProcessingChainFactory()
{
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
