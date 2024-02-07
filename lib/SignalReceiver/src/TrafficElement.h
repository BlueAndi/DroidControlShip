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
 * @brief The traffic element class.
 * @author Paul Gramescu <paul.gramescu@gmail.com>
 *
 * @addtogroup Application
 *
 * @note Traffic element and IE (infrastructure element) used interchangeably
 *
 * @{
 */

#ifndef TRAFFIC_ELEMENT_H
#define TRAFFIC_ELEMENT_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>

#include <Logging.h>
#include <Board.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The traffic element.
 */
class TrafficElement
{
public:
    /** The traffic element instance. */
    static TrafficElement& getInstance()
    {
        static TrafficElement instance;

        return instance;
    }

    /**
     * Set the name of the traffic element.
     *
     * @param[in] elementName is name of IE to be set.
     */
    void setIEName(String elementName)
    {
        m_nameElement = elementName;
    }

    /**
     * Get the name of the traffic element.
     *
     * @returns name of the traffic element
     */
    String getIEName()
    {
        return m_nameElement;
    }

    /**
     * Set orientation of the traffic element.
     *
     * @param[in] requiredOrientation orientation of the traffic element
     */
    void setRequiredOrientation(int32_t requiredOrientation)
    {
        m_orientation = requiredOrientation;
    }

    /**
     * Get orientation of the traffic element.
     *
     * @returns the orientation value of the traffic element
     */
    int32_t getRequiredOrientation()
    {
        return m_orientation;
    }

    /**
     * Set position of inftastrucure element.
     *
     * @param[in] entryX is the x position of IE element.
     * @param[in] entryY is the y position of IE element.
     */
    void setEntryValues(int32_t entryX, int32_t entryY)
    {
        m_entryX = entryX;
        m_entryY = entryY;
    }

    /**
     * Get x position of IE.
     *
     * @return x value
     */
    int32_t getEntryX() const
    {
        return m_entryX;
    }

    /**
     * Get y position of IE.
     *
     * @return y value
     */
    int32_t getEntryY() const
    {
        return m_entryY;
    }

    /**
     * Set distance between robot and the IE.
     *
     * @param[in] currentDistance is value of current distance.
     */
    void setDistance(int32_t currentDistance)
    {
        m_distance = currentDistance;
    }

    /**
     * Get distance between robot and the IE.
     *
     * @returns current distance.
     */
    int32_t getDistance()
    {
        return m_distance;
    }

    /**
     * Set previous distance.
     *
     * @param[in] previousDistance is distance from previous cycle.
     */
    void setPreviousDistance(int32_t previousDistance)
    {
        m_previousDistance = previousDistance;
    }

    /**
     * Get previous distance.
     *
     * @returns previous distance
     */
    int32_t getPreviousDistance()
    {
        return m_previousDistance;
    }

    /**
     * Set MQTT topic of IE.
     *
     * @param[in] topic is topic name.
     */
    void setTopicName(const String& topic)
    {
        m_topicName = topic;
    }

    /**
     * Get topic name of IE.
     *
     * @returns the topic name.
     */
    String& getTopicName()
    {
        return m_topicName;
    }

    /**
     * Sets the status.
     *
     * @param[in] status is the status to be set.
     */
    void setStatus(uint8_t status)
    {
        m_trafficStatus = status;
    }

    /**
     * Get the status. Matches the status in CoordinateHandler.
     *
     * @returns the status between robot and IE.
     */
    uint8_t getStatus()
    {
        return m_trafficStatus;
    }

    /**
     * Check if IE name is empty.
     *
     * @returns true if empty.
     */
    bool isEmpty()
    {
        bool isTrue;

        /** Only name shall be different than empty, rest CAN be received as 0. */
        if (getIEName() == "")
        {
            isTrue = true;
        }
        else
        {
            isTrue = false;
        }

        return isTrue;
    }

    /** Traffic element constructor. */
    TrafficElement() :
        m_nameElement(),
        m_orientation(0),
        m_entryX(0),
        m_entryY(0),
        m_distance(0),
        m_previousDistance(0),
        m_topicName(),
        m_trafficStatus(0U)
    {
    }

    /** Participant deconstructor. */
    ~TrafficElement()
    {
    }

private:
    /** Name of infrastructure element. */
    String m_nameElement;

    /** Orientation of the infrastructure element. */
    int32_t m_orientation;

    /** x position of the IE. */
    int32_t m_entryX;

    /** y position of the IE. */
    int32_t m_entryY;

    /** Distance between robot and IE. */
    int32_t m_distance;

    /** Previous distance. */
    int32_t m_previousDistance;

    /** Topic name of IE. */
    String m_topicName;

    /** Status between robot and IE. */
    uint8_t m_trafficStatus;

    TrafficElement(const TrafficElement& state);            /**< Copy construction of an instance. */
    TrafficElement& operator=(const TrafficElement& state); /**< Assignment of an instance. */
};

#endif /*TRAFFIC_ELEMENT_H*/
       /** @} */