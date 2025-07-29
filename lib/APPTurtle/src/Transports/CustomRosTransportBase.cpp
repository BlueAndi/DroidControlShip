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
 * @brief  Custom Transport class with C-language interface for microros.
 * @author Norbert Schulz <github@schulznorbert.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "CustomRosTransportBase.h"

#include <Logging.h>
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

/**
 * Unwrap pointer to CustomRosTransportBase from uxrCustomTransport structure.
 * 
 * This is used by the static C-Languge entry points to forward the request
 * to the matching C++ member function.
 *
 * @param[in] transport The custom transport data structure pointer.
 *
 * @return The this pointer to transport owning CustomRosTransportBase class.
 */
static inline CustomRosTransportBase* toThis(const uxrCustomTransport* transport);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

bool CustomRosTransportBase::open(uxrCustomTransport* transport)
{
    CustomRosTransportBase * tp = toThis(transport);
    return (nullptr != tp) ? tp->open() : false;
}

bool CustomRosTransportBase::close(uxrCustomTransport* transport)
{
    CustomRosTransportBase * tp = toThis(transport);
    return (nullptr != tp) ? tp->close() : false;
}

size_t CustomRosTransportBase::write(uxrCustomTransport* transport, const uint8_t* buffer, size_t size, uint8_t* errorCode)
{
    CustomRosTransportBase * tp = toThis(transport);
    return (nullptr != tp) ? tp->write(buffer, size, errorCode) : 0U;
}

size_t CustomRosTransportBase::read(uxrCustomTransport* transport, uint8_t* buffer, size_t size, int timeout,
                                uint8_t* errorCode)
{
    CustomRosTransportBase * tp = toThis(transport);
    return (nullptr != tp) ? tp->read(buffer, size, timeout, errorCode) : 0U;
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/

static inline CustomRosTransportBase* toThis(const uxrCustomTransport* transport)
{
    CustomRosTransportBase* transportClass = nullptr;
    
    if (nullptr == transport)
    {
        LOG_FATAL("Invalid uxrCustomTransport pointer.");
    }
    else
    {
        transportClass = reinterpret_cast<CustomRosTransportBase*>(transport->args);
    }

    return transportClass;
}