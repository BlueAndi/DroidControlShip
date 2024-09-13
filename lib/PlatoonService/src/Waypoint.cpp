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
 * @brief  Definition of a Waypoint.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/

#include "Waypoint.h"
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

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Default size of the JSON Document for parsing. */
static const uint32_t JSON_DOC_DEFAULT_SIZE = 1024U;

/******************************************************************************
 * Public Methods
 *****************************************************************************/

Waypoint* Waypoint::deserialize(const String& serializedWaypoint)
{
    Waypoint*            waypoint = nullptr;
    JsonDocument         jsonPayload;
    DeserializationError error = deserializeJson(jsonPayload, serializedWaypoint.c_str());

    if (DeserializationError::Ok != error)
    {
        LOG_ERROR("JSON Deserialization Error %d.", error);
    }
    else
    {
        JsonObject jsonWaypointObject = jsonPayload.as<JsonObject>();
        waypoint                      = fromJsonObject(jsonWaypointObject);
    }

    return waypoint;
}

Waypoint* Waypoint::fromJsonObject(const JsonObject& jsonWaypoint)
{
    Waypoint* waypoint = nullptr;

    if (jsonWaypoint.containsKey("X") && jsonWaypoint.containsKey("Y") && jsonWaypoint.containsKey("Orientation") &&
        jsonWaypoint.containsKey("Left") && jsonWaypoint.containsKey("Right") && jsonWaypoint.containsKey("Center"))
    {
        JsonVariant jsonXPos        = jsonWaypoint["X"];           /* X position [mm]. */
        JsonVariant jsonYPos        = jsonWaypoint["Y"];           /* Y position [mm]. */
        JsonVariant jsonOrientation = jsonWaypoint["Orientation"]; /* Orientation [mrad]. */
        JsonVariant jsonLeft        = jsonWaypoint["Left"];        /* Left motor speed [mm/s]. */
        JsonVariant jsonRight       = jsonWaypoint["Right"];       /* Right motor speed [mm/s]. */
        JsonVariant jsonCenter      = jsonWaypoint["Center"];      /* Center speed [mm/s]. */

        if ((false == jsonXPos.isNull()) && (false == jsonYPos.isNull()) && (false == jsonOrientation.isNull()) &&
            (false == jsonLeft.isNull()) && (false == jsonRight.isNull()) && (false == jsonCenter.isNull()))
        {

            int32_t xPos        = jsonXPos.as<int32_t>();
            int32_t yPos        = jsonYPos.as<int32_t>();
            int32_t orientation = jsonOrientation.as<int32_t>();
            int32_t left        = jsonLeft.as<int32_t>();
            int32_t right       = jsonRight.as<int32_t>();
            int32_t center      = jsonCenter.as<int32_t>();

            waypoint = new (std::nothrow) Waypoint(xPos, yPos, orientation, left, right, center);
        }
    }

    return waypoint;
}

void Waypoint::serialize(String& serializedWaypoint) const
{
    JsonDocument jsonPayload;

    jsonPayload["X"]           = xPos;        /* X position [mm]. */
    jsonPayload["Y"]           = yPos;        /* Y position [mm]. */
    jsonPayload["Orientation"] = orientation; /* Orientation [mrad]. */
    jsonPayload["Left"]        = left;        /* Left motor speed [mm/s]. */
    jsonPayload["Right"]       = right;       /* Right motor speed [mm/s]. */
    jsonPayload["Center"]      = center;      /* Center speed [mm/s]. */

    size_t jsonBufferSize = measureJson(jsonPayload) + 1U;
    char   jsonBuffer[jsonBufferSize];

    if ((jsonBufferSize - 1U) != serializeJson(jsonPayload, jsonBuffer, jsonBufferSize))
    {
        LOG_ERROR("JSON serialization failed.");
        serializedWaypoint.clear();
    }
    else
    {
        serializedWaypoint = jsonBuffer;
    }
}

void Waypoint::debugPrint() const
{
    LOG_DEBUG("X: %d, Y: %d, Orientation: %d, Left: %d, Right: %d, Center: %d", xPos, yPos, orientation, left, right,
              center);
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
