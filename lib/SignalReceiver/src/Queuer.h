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
 * @brief The green light phase realization.
 * @author Paul Gramescu <paul.gramescu@gmail.com>
 *
 * @addtogroup Application
 *
 * @{
 */

#ifndef QUEUER_H
#define QUEUER_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>

#include <Logging.h>
#include <Board.h>

#include <Participants.h>
#include <queue>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * The green light phase.
 */
class Queuer
{
public:
    /** Queue of infrastructure elements. */
    std::queue<InfrastructureElement*> m_IEQueue;

    static Queuer& getInstance()
    {
        static Queuer instance;

        return instance;
    }

    void process();

    /***
     * Check if queue is full
     *
     * @returns true if full
     */
    bool isFull();

    /**
     * Enqueue infrastructure element.
     *
     * @param[in] elementToEnqueue the infrastructure element
     */
    bool enqueueParticipant(InfrastructureElement* elementToEnqueue);

    /**
     * Dequeue infrastructure element.
     *
     */
    void dequeueParticipant();

    /**
     * Adds the infrastructure element to the list of traffic participants.
     *
     * @param[in] infrastructureElement the traffic participant
     */
    void addParticipant(InfrastructureElement infrastructureElement);

private:
    /** Max number of elements in the queue. */
    static const int8_t MAX_QUEUE_ELEMENTS = 10;

    /** Green state constructor. */
    Queuer()
    {
    }

    /** Green state deconstructor. */
    ~Queuer()
    {
    }

    Queuer(const Queuer& state);
    Queuer& operator=(const Queuer& state);
};

#endif /*QUEUER_H*/
       /** @} */