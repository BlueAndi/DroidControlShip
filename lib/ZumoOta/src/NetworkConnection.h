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
 * @brief  ZumoOta application
 * @author Decareme Pauline Ngangnou <ngandeca@yahoo.fr>
 *
 * @addtogroup Application
 *
 * {
 */

#ifndef NETWORK_CONNECTION_H
#define NETWORK_CONNECTION_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/


/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

class NetworkConnection {
public:
    /**
     * Constructor for the NetworkCustome class.
     */
    NetworkConnection();

    /**
     * Destructor for the NetworkCustome class.
     */
    ~NetworkConnection();

    /**
     *  Initiates the process of connecting to a WiFi network.
     */
    void connectToWiFi();

    /**
     *  Checks the current status of the network connection.
     */
    void checkConnection();

    /**
     *  Transitions the system into Access Point (AP) mode.
     */
    void switchToAPMode();
    

private:
    /**
     *  A boolean variable indicating the current connection status (true if connected, false if not).
     */
    bool m_connected;

    /**
     *  An integer variable tracking the number of attempts made to establish a connection.
     */
    int m_connectAttempts;

};

#endif /* NETWORK_CONNECTION_H */
/** @} */
