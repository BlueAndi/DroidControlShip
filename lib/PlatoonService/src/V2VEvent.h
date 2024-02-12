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
 * @brief  V2V Events definition.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup PlatoonService
 *
 * @{
 */
#ifndef V2V_EVENT_H
#define V2V_EVENT_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** V2V Event type. */
enum V2VEventType : uint8_t
{
    V2V_EVENT_TYPE_UNKNOWN = 0,  /**< Unknown event type. */
    V2V_EVENT_WAYPOINT,          /**< Waypoint event. */
    V2V_EVENT_FEEDBACK,          /**< Feedback event. */
    V2V_EVENT_EMERGENCY,         /**< Emergency event. */
    V2V_EVENT_VEHICLE_HEARTBEAT, /**< Vehicle heartbeat event. */
    V2V_EVENT_PLATOON_HEARTBEAT  /**< Platoon heartbeat event. */
};

/** V2V Event definition. */
struct V2VEvent
{
    uint8_t      vehicleId; /**< Vehicle ID. */
    V2VEventType type;      /**< Event type. */
    uint32_t     timestamp; /**< Timestamp. */
    void*        data;      /**< Pointer to event data. */

    /**
     * Construct a V2V event.
     *
     * @param[in] vehicleId Vehicle ID.
     * @param[in] type      Event type.
     * @param[in] timestamp Timestamp.
     * @param[in] data      Pointer to event data.
     */
    V2VEvent(uint8_t vehicleId, V2VEventType type, uint32_t timestamp, void* data) :
        vehicleId(vehicleId),
        type(type),
        timestamp(timestamp),
        data(data)
    {
    }

    /**
     * Construct a V2V event.
     */
    V2VEvent() : V2VEvent(0, V2V_EVENT_TYPE_UNKNOWN, 0, nullptr)
    {
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* V2V_EVENT_H */
/** @} */
