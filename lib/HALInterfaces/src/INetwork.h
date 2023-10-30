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
 * @brief  Abstract network interface
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALInterfaces
 *
 * @{
 */

#ifndef INETWORK_H
#define INETWORK_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include <WString.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** Struct encompassing all network settings */
struct NetworkSettings
{
    String ssid;     /* SSID of the WiFi network. */
    String password; /* Password of the WiFi network. */
};

/** The abstract network interface. */
class INetwork
{
public:
    /**
     * Destroys the interface.
     */
    virtual ~INetwork()
    {
    }

    /**
     * Set network configuration.
     *
     * @param[in] settings NetworkSettings struct containing ssid and password.
     * @return If successfully set, returns true. Otherwise, false.
     */
    virtual bool setConfig(const NetworkSettings& settings) = 0;

    /**
     * Initialize the network connection.
     *
     * @return If successfully initialized, returns true. Otherwise, false.
     */
    virtual bool init() = 0;

    /**
     * Handle connection specific tasks.
     *
     * @return If connection management successfull, returns true. Otherwise, false.
     */
    virtual bool manageConnection() = 0;

protected:
    /**
     * Constructs the interface.
     */
    INetwork()
    {
    }

private:
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* INETWORK_H */
/** @} */
