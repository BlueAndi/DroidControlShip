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
 * @brief  Keys of the JSON structure of the configuration file.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup Service
 *
 * @{
 */
#ifndef CONFIGURAION_KEYS_H
#define CONFIGURAION_KEYS_H

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

/**
 * Configuration keys found in the config file.
 */
namespace ConfigurationKeys
{
    /** Robot name */
    static const char ROBOT_NAME[] = "robotName";

    /** WiFi */
    static const char WIFI[] = "wifi";

    /** MQTT */
    static const char MQTT[] = "mqtt";

    /** Access point */
    static const char AP[] = "ap";

    /** WebServer */
    static const char WEBSERVER[] = "webServer";

    /** Platoon */
    static const char PLATOON[] = "platoon";

    /** SSID */
    static const char SSID[] = "ssid";

    /** Passphrase */
    static const char PASSWORD[] = "pswd";

    /** Host */
    static const char HOST[] = "host";

    /** Port */
    static const char PORT[] = "port";

    /** User */
    static const char USER[] = "user";

    /** Platoon ID */
    static const char PLATOON_ID[] = "platoonId";

    /** Vehicle ID */
    static const char VEHICLE_ID[] = "vehicleId";

} // namespace ConfigurationKeys

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* CONFIGURAION_KEYS_H */
/** @} */
