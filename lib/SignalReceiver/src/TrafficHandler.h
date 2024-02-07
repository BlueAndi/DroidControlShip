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
 * @brief The green light phase realization.
 * @author Paul Gramescu <paul.gramescu@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef TRAFFIC_HANDLER_H
#define TRAFFIC_HANDLER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>

#include <Logging.h>
#include <Board.h>

#include <TrafficElement.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The traffic handler class.
 */
class TrafficHandler
{
public:
    /** The instance of Traffic handler. */
    static TrafficHandler& getInstance()
    {
        static TrafficHandler instance;

        return instance;
    }

    /**
     * Process traffic.
     *
     * @returns true if processing is successful.
     */
    bool process();

    /**
     * Process list of IEs.
     *
     * @returns true if the IE list is successfully processed.
     */
    bool processList();

    /**
     * Add new IE to the list of traffic participants.
     *
     * @param[in] nameAsParameter is name of the IE
     * @param[in] orientationAsParameter is orientation of IE
     * @param[in] xPosAsParameter is position of IE on the x Axis
     * @param[in] yPosAsParameter is position of IE on the y Axis
     * @param[in] defaultDistanceAsParameter is set to 0 as default.
     * @param[in] defaultPreviousDistanceAsParameter is set to 0 as default.
     * @param[in] topicNameAsParameter is the MQTT topic to subscribe to.
     */
    bool setNewInfrastructureElement(const String& nameAsParameter, int32_t orientationAsParameter,
                                     int32_t xPosAsParameter, int32_t yPosAsParameter,
                                     int32_t defaultDistanceAsParameter, int32_t defaultPreviousDistanceAsParameter,
                                     const String& topicNameAsParameter);

    /**
     * Set the value of received color.
     *
     * @param[in] receivedColor is the value of the color.
     */
    void setColorID(uint8_t receivedColor)
    {
        m_colorID = receivedColor;
    }

    /**
     * Get the latest color ID.
     *
     * @returns the value of the color ID.
     */
    uint8_t getColorID()
    {
        return m_colorID;
    }

    /**
     * Changes motor speeds based on the received color.
     */
    void processColor();

    /**
     * Check if robot-IE status is LOCKED_IN.
     *
     * @returns true if robot is locked onto IE.
     */
    bool checkLockIn();

    /**
     * Check if robot-IE status is NEAR.
     *
     * @returns true if robot is near the IE.
     */
    bool isNear();

    /**
     * Get the name of locked-onto IE.
     *
     * @returns the name of locked onto IE.
     */
    String getTargetName()
    {
        return lockedOnto;
    }

    /**
     * Clears the target name.
     */
    void clearTarget()
    {
        lockedOnto = "";
    }

    /** Traffic handler constructor. */
    TrafficHandler() : m_colorID(0U)
    {
    }

    /** Traffic handler deconstructor. */
    ~TrafficHandler()
    {
    }

private:
    /** Max number of elements in the list. */
    static const uint8_t MAX_ELEMENTS = 10;

    /** Number of enlisted IEs. */
    uint8_t m_IECounter = 0;

    /** The value of the received color. */
    uint8_t m_colorID;

    /** Current IE that robot is locked onto. */
    String lockedOnto = "";

    /** List of infrastructure elements. */
    TrafficElement listOfElements[MAX_ELEMENTS];

    TrafficHandler(const TrafficHandler& state);            /**< Copy construction of an instance. */
    TrafficHandler& operator=(const TrafficHandler& state); /**< Assignment of an instance. */
};

#endif /*TRAFFIC_HANDLER_H*/
       /** @} */