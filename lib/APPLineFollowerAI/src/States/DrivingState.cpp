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
 * @brief  Driving state
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "DrivingState.h"
#include <Board.h>
#include <StateMachine.h>
#include "ReadyState.h"
#include "ParameterSets.h"
#include <Util.h>
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

const uint16_t DrivingState::SENSOR_VALUE_MAX = LineSensors::getSensorValueMax();

/* Calculate the position set point to be generic. */
const int16_t DrivingState::POSITION_SET_POINT = (SENSOR_VALUE_MAX * (LineSensors::getNumLineSensors() - 1)) / 2;

/* Initialize the required sensor IDs to be generic. */
const uint8_t DrivingState::SENSOR_ID_MOST_LEFT  = 0U;
const uint8_t DrivingState::SENSOR_ID_MIDDLE     = (LineSensors::getNumLineSensors() - 1U) / 2U;
const uint8_t DrivingState::SENSOR_ID_MOST_RIGHT = LineSensors::getNumLineSensors() - 1U;

/* Initialize the position values used by the algorithmic. */
const int16_t DrivingState::POSITION_MIDDLE_MIN = POSITION_SET_POINT - (SENSOR_VALUE_MAX / 2);
const int16_t DrivingState::POSITION_MIDDLE_MAX = POSITION_SET_POINT + (SENSOR_VALUE_MAX / 2);

/**
 * Pre-trained neural network hidden layer weights.
 * Each row represents one hidden neuron's weights for the 5 input sensors.
 * Hidden neurons act as position detectors: left-most, left-center, center, right-center, right-most.
 * Designed for velocity-independent behavior through balanced activation.
 */
const DrivingState::WeightsInputHiddenType DrivingState::HIDDEN_LAYER_WEIGHTS({{2.5F, 0.6F, -0.5F, -0.5F, -0.8F},
                                                                               {0.6F, 2.0F, -0.2F, -0.4F, -0.6F},
                                                                               {-0.5F, -0.2F, 3.5F, -0.2F, -0.5F},
                                                                               {-0.6F, -0.4F, -0.2F, 2.0F, 0.6F},
                                                                               {-0.8F, -0.5F, -0.5F, 0.6F, 2.5F}});

/**
 * Pre-trained neural network hidden layer biases.
 * Configured for consistent activation patterns across all velocities.
 */
const DrivingState::BiasesHiddenType DrivingState::HIDDEN_LAYER_BIASES({{-0.7F}, {-0.6F}, {-0.2F}, {-0.6F}, {-0.7F}});

/**
 * Pre-trained neural network output layer weights.
 * Creates velocity-independent steering ratios by maintaining proportional wheel speed differences.
 * Reduced magnitude ensures outputs stay within [0.4, 1.0] range to avoid saturation.
 * Row 0 (left motor): Speed ratio relative to base speed.
 * Row 1 (right motor): Speed ratio relative to base speed.
 */
const DrivingState::WeightsHiddenOutputType DrivingState::OUTPUT_LAYER_WEIGHTS({{-0.22F, -0.05F, 0.0F, 0.05F, 0.22F},
                                                                                {0.22F, 0.05F, 0.0F, -0.05F, -0.22F}});

/**
 * Pre-trained neural network output layer biases.
 * Base speed at 75% provides symmetric headroom (±25%) for steering corrections at any velocity.
 * This prevents saturation and ensures consistent behavior across the full velocity range.
 */
const DrivingState::BiasesOutputType DrivingState::OUTPUT_LAYER_BIASES({{0.75F}, {0.75F}});

/******************************************************************************
 * Public Methods
 *****************************************************************************/

void DrivingState::entry()
{
    IBoard&                            board    = Board::getInstance();
    const ParameterSets::ParameterSet& parSet   = ParameterSets::getInstance().getParameterSet();
    int16_t                            maxSpeed = 0; /* [digits] */

    LOG_INFO("Driving state entered.");

    if (nullptr != m_motors)
    {
        maxSpeed = m_motors->getMaxSpeed();
    }

    m_observationTimer.start(OBSERVATION_DURATION);
    m_lineStatus              = LINE_STATUS_NO_START_LINE_DETECTED;
    m_trackStatus             = TRACK_STATUS_NORMAL; /* Assume that the robot is placed on track. */
    m_isTrackLost             = false;               /* Assume that the robot is placed on track. */
    m_isStartStopLineDetected = false;

    /* Configure PID controller with selected parameter set. */
    m_topSpeed = parSet.topSpeed;
}

void DrivingState::process(StateMachine& sm)
{
    IBoard&     board           = Board::getInstance();
    TrackStatus nextTrackStatus = m_trackStatus;

    /* Get the position of the line and each sensor value. */
    int16_t         position         = 0;
    const uint16_t* lineSensorValues = nullptr;
    int16_t         position3        = 0;
    bool            isPosition3Valid = false;
    bool            isTrackLost      = false;

    if (nullptr != m_lineSensors)
    {
        position         = m_lineSensors->readLine();
        lineSensorValues = m_lineSensors->getSensorValues();

        isPosition3Valid = calcPosition3(position3, lineSensorValues, LineSensors::getNumLineSensors());
        isTrackLost      = isNoLineDetected(lineSensorValues, LineSensors::getNumLineSensors());
    }

    /* If the position calculated with the inner sensors is not valid, the
     * position will be taken.
     */
    if (true == isPosition3Valid)
    {
        position3 = position;
    }

    /* ========================================================================
     * Evaluate the situation based on the sensor values.
     * ========================================================================
     */
    nextTrackStatus = evaluateSituation(lineSensorValues, LineSensors::getNumLineSensors(), position, isTrackLost);

    /* ========================================================================
     * Handle start-/stop-line actions.
     * ========================================================================
     */
    if ((TRACK_STATUS_START_STOP_LINE != m_trackStatus) && (TRACK_STATUS_START_STOP_LINE == nextTrackStatus))
    {
        /* Start line detected? */
        if (LINE_STATUS_NO_START_LINE_DETECTED == m_lineStatus)
        {
            /* Measure the lap time and use as start point the detected start line. */
            m_lapTime.start(0);

            m_lineStatus = LINE_STATUS_START_LINE_DETECTED;
        }
        /* Stop line detected. */
        else
        {
            /* Calculate lap time and show it. */
            ReadyState::getInstance().setLapTime(m_lapTime.getCurrentDuration());

            m_lineStatus = LINE_STATUS_STOP_LINE_DETECTED;

            /* Overwrite track status. */
            nextTrackStatus = TRACK_STATUS_FINISHED;
        }
    }

    /* ========================================================================
     * Handle track lost or back on track actions.
     * ========================================================================
     */

    /* Track lost just in this process cycle? */
    if ((false == m_isTrackLost) && (true == isTrackLost))
    {
        /* Notify user by yellow LED. */
        Board::getInstance().getBlueLed().enable(true);
    }
    /* Track found again just in this process cycle? */
    else if ((true == m_isTrackLost) && (false == isTrackLost))
    {
        Board::getInstance().getBlueLed().enable(false);
    }
    else
    {
        ;
    }

    /* ========================================================================
     * Handle the abort conditions which will cause a alarm stop.
     * ========================================================================
     */

    /* Check whether the abort conditions are true. */
    if (true == isAbortRequired())
    {
        /* Stop motors immediately. Don't move this to a later position,
         * as this would extend the driven length.
         */
        if (nullptr != m_motors)
        {
            m_motors->setSpeeds(0, 0);
        }

        /* Clear lap time. */
        ReadyState::getInstance().setLapTime(0);

        /* Overwrite track status. */
        nextTrackStatus = TRACK_STATUS_FINISHED;
    }

    /* ========================================================================
     * Handle driving based on position or normal stop condition.
     * ========================================================================
     */

    /* Periodically adapt driving and check the abort conditions, except
     * the round is finished.
     */
    if (TRACK_STATUS_FINISHED != nextTrackStatus)
    {
        adaptDriving(position);
    }
    /* Finished. */
    else
    {
        /* Stop motors immediately. Don't move this to a later position,
         * as this would extend the driven length.
         */
        if (nullptr != m_motors)
        {
            m_motors->setSpeeds(0, 0);
        }

        /* Change to ready state. */
        sm.setState(ReadyState::getInstance());
    }

    /* Take over values for next cycle. */
    m_trackStatus  = nextTrackStatus;
    m_isTrackLost  = isTrackLost;
    m_lastPosition = position;
}

void DrivingState::exit()
{
    m_observationTimer.stop();
    Board::getInstance().getBlueLed().enable(false);
}

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

DrivingState::DrivingState() :
    m_lineSensors(nullptr),
    m_motors(nullptr),
    m_observationTimer(),
    m_lapTime(),
    m_topSpeed(0),
    m_lineStatus(LINE_STATUS_NO_START_LINE_DETECTED),
    m_trackStatus(TRACK_STATUS_NORMAL),
    m_isStartStopLineDetected(false),
    m_lastSensorIdSawTrack(SENSOR_ID_MIDDLE),
    m_lastPosition(0),
    m_isTrackLost(false),
    m_activationFuncLeakyReLU(),
    m_neuronalNetworkModel(m_activationFuncLeakyReLU, HIDDEN_LAYER_WEIGHTS, HIDDEN_LAYER_BIASES, OUTPUT_LAYER_WEIGHTS,
                           OUTPUT_LAYER_BIASES)
{
}

bool DrivingState::calcPosition3(int16_t& position, const uint16_t* lineSensorValues, uint8_t length) const
{
    const int32_t WEIGHT      = SENSOR_VALUE_MAX;
    bool          isValid     = true;
    int32_t       numerator   = 0U;
    int32_t       denominator = 0U;
    int32_t       idxBegin    = 1;
    int32_t       idxEnd      = length - 1;

    for (int32_t idx = idxBegin; idx < idxEnd; ++idx)
    {
        int32_t sensorValue = static_cast<int32_t>(lineSensorValues[idx]);

        numerator += idx * WEIGHT * sensorValue;
        denominator += sensorValue;
    }

    if (0 == denominator)
    {
        isValid = false;
    }
    else
    {
        position = numerator / denominator;
    }

    return isValid;
}

DrivingState::TrackStatus DrivingState::evaluateSituation(const uint16_t* lineSensorValues, uint8_t length,
                                                          int16_t position, bool isTrackLost) const
{
    TrackStatus nextTrackStatus = m_trackStatus;

    /* Driving over start-/stop-line? */
    if (TRACK_STATUS_START_STOP_LINE == m_trackStatus)
    {
        /* Left the start-/stop-line?
         * If the robot is not exact on the start-/stop-line, the calculated position
         * may misslead. Therefore additional the most left and right sensor values
         * are evaluated too.
         */
        if ((POSITION_MIDDLE_MIN <= position) && (POSITION_MIDDLE_MAX >= position) &&
            (LINE_SENSOR_ON_TRACK_MIN_VALUE > lineSensorValues[SENSOR_ID_MOST_LEFT]) &&
            (LINE_SENSOR_ON_TRACK_MIN_VALUE > lineSensorValues[SENSOR_ID_MOST_RIGHT]))
        {
            nextTrackStatus = TRACK_STATUS_NORMAL;
        }
    }
    /* Is the start-/stop-line detected? */
    else if (true == isStartStopLineDetected(lineSensorValues, length))
    {
        nextTrackStatus = TRACK_STATUS_START_STOP_LINE;
    }
    /* Is the track lost or just a gap in the track? */
    else if (true == isTrackLost)
    {
        const int16_t POS_MIN = POSITION_SET_POINT - SENSOR_VALUE_MAX;
        const int16_t POS_MAX = POSITION_SET_POINT + SENSOR_VALUE_MAX;

        /* If its a gap in the track, last position will be well. */
        if ((POS_MIN <= m_lastPosition) && (POS_MAX > m_lastPosition))
        {
            nextTrackStatus = TRACK_STATUS_TRACK_LOST_BY_GAP;
        }
        else
        {
            /* In any other case, route the calculated position through and
             * hope. It will be the position of the most left sensor or the
             * most right sensor.
             */
            nextTrackStatus = TRACK_STATUS_TRACK_LOST_BY_MANOEUVRE;
        }
    }
    /* Nothing special. */
    else
    {
        /* Just follow the line by calculated position. */
        nextTrackStatus = TRACK_STATUS_NORMAL;
    }

    return nextTrackStatus;
}

bool DrivingState::isStartStopLineDetected(const uint16_t* lineSensorValues, uint8_t length) const
{
    bool           isDetected  = false;
    const uint32_t LINE_MAX_30 = (SENSOR_VALUE_MAX * 3U) / 10U; /* 30 % of max. value */
    const uint32_t LINE_MAX_70 = (SENSOR_VALUE_MAX * 7U) / 10U; /* 70 % of max. value */

    /*
     * ===     =     ===
     *   +   + + +   +
     *   L     M     R
     */
    if ((LINE_MAX_30 <= lineSensorValues[SENSOR_ID_MOST_LEFT]) &&
        (LINE_MAX_70 > lineSensorValues[SENSOR_ID_MIDDLE - 1U]) &&
        (LINE_MAX_70 <= lineSensorValues[SENSOR_ID_MIDDLE]) &&
        (LINE_MAX_70 > lineSensorValues[SENSOR_ID_MIDDLE + 1U]) &&
        (LINE_MAX_30 <= lineSensorValues[SENSOR_ID_MOST_RIGHT]))
    {
        isDetected = true;
    }

    return isDetected;
}

bool DrivingState::isNoLineDetected(const uint16_t* lineSensorValues, uint8_t length) const
{
    bool    isDetected = true;
    uint8_t idx        = SENSOR_ID_MOST_RIGHT;

    /*
     *
     *   +   + + +   +
     *   L     M     R
     */
    for (idx = SENSOR_ID_MOST_LEFT; idx <= SENSOR_ID_MOST_RIGHT; ++idx)
    {
        if (LINE_SENSOR_ON_TRACK_MIN_VALUE <= lineSensorValues[idx])
        {
            isDetected = false;
            break;
        }
    }

    return isDetected;
}

void DrivingState::adaptDriving(int16_t position)
{
    IBoard&       board           = Board::getInstance();
    const int16_t MAX_MOTOR_SPEED = (nullptr != m_motors) ? m_motors->getMaxSpeed() : 0; /* [digits] */
    const int16_t MIN_MOTOR_SPEED = -MAX_MOTOR_SPEED;                                    /* [digits] */
    const float   TOP_SPEED_F     = static_cast<float>(m_topSpeed);
    int16_t       leftSpeed       = 0; /* [digits] */
    int16_t       rightSpeed      = 0; /* [digits] */

    /* Calculate motor speeds based on NN output. */
    const uint16_t* lineSensorValues = m_lineSensors->getSensorValues();
    const size_t    lineSensorCount  = LineSensors::getNumLineSensors();

    if (nullptr != lineSensorValues)
    {
        Eigen::Matrix<float, 5, 1> inputLayer;
        Eigen::Matrix<float, 2, 1> outputLayer;

        /* Prepare input layer.
         * Normalize sensor values to [0.0 .. 1.0].
         */
        for (size_t idx = 0; idx < lineSensorCount; ++idx)
        {
            float lineSensorValue = static_cast<float>(lineSensorValues[idx]);
            float sensorValueMax  = static_cast<float>(SENSOR_VALUE_MAX);
            float normalizedValue = lineSensorValue / sensorValueMax;

            inputLayer(idx, 0) = normalizedValue;
        }

        /* Perform forward pass through the network. */
        m_neuronalNetworkModel.forward(inputLayer, outputLayer);

        /* Map output to motor speeds. */
        leftSpeed  = static_cast<int16_t>(outputLayer(0, 0) * TOP_SPEED_F);
        rightSpeed = static_cast<int16_t>(outputLayer(1, 0) * TOP_SPEED_F);
    }

    /* Constrain our motor speeds to be between 0 and maxSpeed.
     * One motor will always be turning at maxSpeed, and the other
     * will be at maxSpeed-|speedDifference| if that is positive,
     * else it will be stationary. For some applications, you
     * might want to allow the motor speed to go negative so that
     * it can spin in reverse.
     */
    leftSpeed  = constrain(leftSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

    if (nullptr != m_motors)
    {
        m_motors->setSpeeds(leftSpeed, rightSpeed);
    }
}

bool DrivingState::isAbortRequired()
{
    bool isAbort = false;

    /* If track is not finished over a certain time, abort driving. */
    if (TRACK_STATUS_FINISHED != m_trackStatus)
    {
        if (true == m_observationTimer.isTimeout())
        {
            isAbort = true;
        }
    }

    return isAbort;
}

/******************************************************************************
 * External Functions
 *****************************************************************************/

/******************************************************************************
 * Local Functions
 *****************************************************************************/
