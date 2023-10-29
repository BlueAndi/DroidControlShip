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
 * @brief  SensorFusion application
 * @author Juliane Kerpe <juliane.kerpe@web.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "App.h"
#include <Logging.h>
#include <LogSinkPrinter.h>
#include <Util.h>
#include <Settings.h>
#include <ArduinoJson.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

#ifndef CONFIG_LOG_SEVERITY
#define CONFIG_LOG_SEVERITY (Logging::LOG_LEVEL_INFO)
#endif /* CONFIG_LOG_SEVERITY */

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

static void App_sensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Serial interface baudrate. */
static const uint32_t SERIAL_BAUDRATE = 115200U;

/** Serial log sink */
static LogSinkPrinter gLogSinkSerial("Serial", &Serial);

/* Initialize channel name for receiving sensor data. */
const char* App::CH_NAME_SENSORDATA = "SENSOR_DATA";

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void App::setup()
{
    Serial.begin(SERIAL_BAUDRATE);

    SensorFusion::getInstance().init();

    /* Register serial log sink and select it per default. */
    if (true == Logging::getInstance().registerSink(&gLogSinkSerial))
    {
        (void)Logging::getInstance().selectSink("Serial");

        /* Set severity of logging system. */
        Logging::getInstance().setLogLevel(CONFIG_LOG_SEVERITY);

        LOG_DEBUG("LOGGER READY");
    }

    /* Initialize HAL. */
    if (false == Board::getInstance().init())
    {
        /* Log and Handle Board initialization error */
        LOG_FATAL("HAL init failed.");
        fatalErrorHandler();
    }
    else
    {
        if (false == Settings::getInstance().isConfigLoaded())
        {
            /* Settings shall be loaded from configuration file. */
            if (false == Settings::getInstance().loadConfigurationFile(CONFIG_FILE_PATH))
            {
                /* Log Settings error */
                LOG_ERROR("Settings could not be loaded from file. ");
                fatalErrorHandler();
            }
            else
            {
                /* Log Settings loaded */
                LOG_DEBUG("Settings loaded from file.");
            }
        }
        else
        {
            LOG_DEBUG("Settings set externally.");
        }

         /* Setup SerialMuxProt Channel. */
        m_smpServer.subscribeToChannel(CH_NAME_SENSORDATA, App_sensorChannelCallback);

    }
}

void App::loop()
{
    /* Process Battery, Device and Network. */
    if (false == Board::getInstance().process())
    {
        /* Log and Handle Board processing error */
        LOG_FATAL("HAL process failed.");
        fatalErrorHandler();
    }

    /* Process SerialMuxProt. */
    m_smpServer.process(millis());
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

void App::fatalErrorHandler()
{
    /* Turn on Red LED to signal fatal error. */
    Board::getInstance().getRedLed().enable(true);

    while (true)
    {
        ;
    }
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * Receives sensor data for sensor fusion over SerialMuxProt channel in this order:
 * Acceleration in X
 * Acceleration in Y
 * TurnRate around Z
 * Magnetometer value in X 
 * Magnetometer value in Y 
 * Angle calculated by Odometry
 * Position in X calculated by Odometry
 * Position in Y calculated by Odometry
 * @param[in]   payload         Sensor data
 * @param[in]   payloadSize     Size of 8 sensor data
 */
void App_sensorChannelCallback(const uint8_t* payload, const uint8_t payloadSize)
{
    SensorData newSensorData;

    Util::byteArrayToInt16(&payload[0 * sizeof(int16_t)], sizeof(int16_t), newSensorData.accelerationX);
    Util::byteArrayToInt16(&payload[1 * sizeof(int16_t)], sizeof(int16_t), newSensorData.accelerationY);
    Util::byteArrayToInt16(&payload[2 * sizeof(int16_t)], sizeof(int16_t), newSensorData.turnRateZ);
    Util::byteArrayToInt16(&payload[3 * sizeof(int16_t)], sizeof(int16_t), newSensorData.magnetometerValueX);
    Util::byteArrayToInt16(&payload[4 * sizeof(int16_t)], sizeof(int16_t), newSensorData.magnetometerValueY);
    Util::byteArrayToInt16(&payload[5 * sizeof(int16_t)], sizeof(int16_t), newSensorData.angleOdometry);
    Util::byteArrayToInt16(&payload[6 * sizeof(int16_t)], sizeof(int16_t), newSensorData.positionOdometryX);
    Util::byteArrayToInt16(&payload[7 * sizeof(int16_t)], sizeof(int16_t), newSensorData.positionOdometryY);

    newSensorData.accelerationX = newSensorData.accelerationX   *   SensorConstants::ACCELEROMETER_SENSITIVITY_FACTOR;
    newSensorData.accelerationY = newSensorData.accelerationY   *   SensorConstants::ACCELEROMETER_SENSITIVITY_FACTOR;
    newSensorData.turnRateZ     = newSensorData.turnRateZ       *   SensorConstants::GYRO_SENSITIVITY_FACTOR;
    
    SensorFusion::getInstance().estimateNewState(newSensorData);
}
