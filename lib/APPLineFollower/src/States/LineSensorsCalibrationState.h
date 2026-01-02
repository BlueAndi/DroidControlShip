/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
 * @file
 * @brief  Line sensors calibration state
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef LINE_SENSORS_CALIBRATION_STATE_H
#define LINE_SENSORS_CALIBRATION_STATE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <IState.h>
#include <SerMuxChannelProvider.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The line sensors calibration state. */
class LineSensorsCalibrationState : public IState
{
public:
    /**
     * Get state instance.
     *
     * @return State instance.
     */
    static LineSensorsCalibrationState& getInstance()
    {
        static LineSensorsCalibrationState instance;

        /* Singleton idiom to force initialization during first usage. */

        return instance;
    }

    /**
     * If the state is entered, this method will called once.
     */
    void entry() final;

    /**
     * Processing the state.
     *
     * @param[in] sm State machine, which is calling this state.
     */
    void process(StateMachine& sm) final;

    /**
     * If the state is left, this method will be called once.
     */
    void exit() final;

    /**
     * Inject dependencies.
     *
     * @param[in] serMuxChannelProvider Serial multiplexer channel provider.
     */
    void injectDependencies(SerMuxChannelProvider& serMuxChannelProvider)
    {
        m_serMuxChannelProvider = &serMuxChannelProvider;
    }

private:
    /**
     * Timeout in ms to wait for line sensor calibration to complete.
     */
    static const uint32_t TIMEOUT_MS = 5000U;

    /**
     * Serial multiplexer channel provider.
     */
    SerMuxChannelProvider* m_serMuxChannelProvider;

    /**
     * Indicates that an error occurred.
     */
    bool m_isError;

    /**
     * Indicates whether the line sensor calibration is finished.
     */
    bool m_isFinished;

    /**
     * Default constructor.
     */
    LineSensorsCalibrationState() : m_serMuxChannelProvider(nullptr), m_isError(false), m_isFinished(false)
    {
        /* Nothing to do. */
    }

    /**
     * Default destructor.
     */
    ~LineSensorsCalibrationState()
    {
    }

    /**
     * Copy construction of an instance.
     * Not allowed.
     *
     * @param[in] state Source instance.
     */
    LineSensorsCalibrationState(const LineSensorsCalibrationState& state);

    /**
     * Assignment of an instance.
     * Not allowed.
     *
     * @param[in] state Source instance.
     *
     * @returns Reference to LineSensorsCalibrationState.
     */
    LineSensorsCalibrationState& operator=(const LineSensorsCalibrationState& state);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* LINE_SENSORS_CALIBRATION_STATE_H */
/** @} */
