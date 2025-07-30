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
 * @brief  Startup state
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef STARTUP_STATE_H
#define STARTUP_STATE_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdint.h>
#include <StateMachine.h>
#include <IState.h>
#include <SimpleTimer.hpp>
#include "SerMuxChannelProvider.h"
#include "Motors.h"

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/** The system startup state. */
class StartupState : public IState
{
public:
    /**
     * Get state instance.
     *
     * @return State instance.
     */
    static StartupState& getInstance()
    {
        static StartupState instance;

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
     * @param[in] motors                Motors instance.
     */
    void injectDependencies(SerMuxChannelProvider& serMuxChannelProvider, Motors& motors)
    {
        m_serMuxChannelProvider = &serMuxChannelProvider;
        m_motors                = &motors;
    }

private:
    /**
     * Sub state machine for the startup state.
     */
    typedef enum
    {
        SUB_STATE_WAIT_FOR_SYNC,            /**< Wait for synchronization with RU. */
        SUB_STATE_WAIT_FOR_MAX_MOTOR_SPEED, /**< Wait for max. motor speed request response */
        SUB_STATE_FINISHED,                 /**< Startup finished */
        SUB_STATE_ERROR                     /**< Error occurred */

    } SubState;

    /**
     * Serial multiplexer channel provider.
     */
    SerMuxChannelProvider* m_serMuxChannelProvider;

    /**
     * Motors.
     */
    Motors* m_motors;

    /**
     * Sub state of the startup state.
     */
    SubState m_subState;

    /**
     * Default constructor.
     */
    StartupState() : m_serMuxChannelProvider(nullptr), m_motors(nullptr), m_subState(SUB_STATE_WAIT_FOR_SYNC)
    {
        /* Nothing to do. */
    }

    /**
     * Default destructor.
     */
    ~StartupState()
    {
    }

    /**
     * Copy construction of an instance.
     * Not allowed.
     *
     * @param[in] state Source instance.
     */
    StartupState(const StartupState& state);

    /**
     * Assignment of an instance.
     * Not allowed.
     *
     * @param[in] state Source instance.
     *
     * @returns Reference to StartupState instance.
     */
    StartupState& operator=(const StartupState& state);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* STARTUP_STATE_H */
/** @} */
