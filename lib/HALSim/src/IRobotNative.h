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
 * @brief  Abstract native robot interface
 * @author Andreas Merkle <web@blue-andi.de>
 *
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef IROBOTNATIVE_H
#define IROBOTNATIVE_H

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

/** The abstract native robot interface. */
class IRobotNative
{
public:
    /**
     * Destroys the interface.
     */
    virtual ~IRobotNative()
    {
    }

    /**
     * Set the serial receive channel id.
     *
     * @param[in] channelId Channel ID, shall be positive for inter-robot communication.
     */
    virtual void setRxChannel(int32_t channelId) = 0;

    /**
     * Set the serial sender channel id.
     *
     * @param[in] channelId Channel ID, shall be positive for inter-robot communication.
     */
    virtual void setTxChannel(int32_t channelId) = 0;

protected:
    /**
     * Constructs the interface.
     */
    IRobotNative()
    {
    }

private:
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* IROBOTNATIVE_H */
/** @} */
