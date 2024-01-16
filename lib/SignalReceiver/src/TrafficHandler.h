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
    /** List of infrastructure elements. */
    TrafficElement listOfElements[10];

    /** The instance of Traffic handler. */
    static TrafficHandler& getInstance()
    {
        static TrafficHandler instance;

        return instance;
    }

    /**
     * Process list by cycling IEs.
     *
     * @returns true if robot-IE traffic successfully processed.
     */
    bool process();

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
    bool setNewInfrastructureElement(const String& nameAsParameter, 
                                     int32_t orientationAsParameter, 
                                     int32_t xPosAsParameter,
                                     int32_t yPosAsParameter, 
                                     int32_t defaultDistanceAsParameter,
                                     int32_t defaultPreviousDistanceAsParameter, 
                                     const String& topicNameAsParameter);

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

    /** Traffic handler constructor. */
    TrafficHandler()
    {
    }

    /** Traffic handler deconstructor. */
    ~TrafficHandler()
    {
    }

private:
    /** Max number of elements in the list. */
    int8_t MAX_ELEMENTS = 10;

    /** Number of enlisted IEs. */
    int8_t NR_OF_ELEMENTS = 0;

    /** Current IE that robot is locked onto. */
    String lockedOnto = "";

    TrafficHandler(const TrafficHandler& state);            /**< Copy construction of an instance. */
    TrafficHandler& operator=(const TrafficHandler& state); /**< Assignment of an instance. */
};

#endif /*TRAFFIC_HANDLER_H*/
       /** @} */