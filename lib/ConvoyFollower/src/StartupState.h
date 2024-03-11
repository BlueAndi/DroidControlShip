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
 * @brief  Startup State.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
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

#include <IState.h>
#include <StateMachine.h>
#include "SerialMuxChannels.h"

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
    /** Commands to send and process in the Startup State. */
    enum StartupCommands : uint8_t
    {
        CMD_GET_MAX_SPEED = 0, /**< Get maximum motor speed. */
        CMD_SET_INIT_POS,      /**< Set initial position. */
        CMD_NONE               /**< No pending command. Required to signal the end of the enum. */
    };

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
     * Get pending command. If there is no pending command or the state is not active, it will return nullptr.
     *
     * @param[out] cmd Buffer to write the pending command.
     *
     * return If there is a pending command, it will return true otherwise false.
     */
    bool getPendingCommand(Command& cmd);

    /**
     * Notify the state, that the pending command is successfully processed by RU.
     */
    void notifyCommandProcessed();

protected:
private:
    /** Flag: State is active. */
    bool m_isActive;

    /** Pending command. */
    Command* m_pendingCommand;

    /** Pending command counter. */
    uint8_t m_pendingCommandCounter;

    /**
     * Default constructor.
     */
    StartupState() : IState(), m_isActive(false), m_pendingCommand(nullptr), m_pendingCommandCounter(CMD_GET_MAX_SPEED)
    {
    }

    /**
     * Default destructor.
     */
    virtual ~StartupState()
    {
    }

    /* Not allowed. */
    StartupState(const StartupState& state);            /**< Copy construction of an instance. */
    StartupState& operator=(const StartupState& state); /**< Assignment of an instance. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* STARTUP_STATE_H */
/** @} */
