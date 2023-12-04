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
<<<<<<<< HEAD:lib/ZumoOta/src/FlashManager.cpp
 * @brief  FlashManager realization
 *
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "FlashManager.h"
#include <LittleFS.h>
#include <Logging.h>
#include <string.h>
/******************************************************************************
 * Compiler Switches
 *****************************************************************************/
========
 * @brief  FileHandler implementation.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 *
 * @addtogroup HALSim
 *
 * @{
 */

#ifndef FILE_HANDLER_H_
#define FILE_HANDLER_H_

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <IFileHandler.h>
>>>>>>>> main:lib/HALSim/src/FileHandler.h

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

<<<<<<<< HEAD:lib/ZumoOta/src/FlashManager.cpp
========
/**
 * File Handler class.
 */
class FileHandler : public IFileHandler
{
public:
    /**
     * Constructs the concrete FileHandler.
     */
    FileHandler();

    /**
     * Destroys the concrete FileHandler.
     */
    virtual ~FileHandler();

    /**
     * Read a file from the filesystem.
     * @param[in] fileName Name of the file to read. Name must be an absolute path.
     * @param[out] outBuffer Buffer to write file to.
     * @param[in] maxBufferSize Max. number of bytes in the buffer.
     * @returns number of bytes read.
     */
    size_t readFile(const String& fileName, char* outBuffer, const uint32_t maxBufferSize) const final;

    /**
     * Write a file to the filesystem.
     * @param[in] fileName Name of the file to write. Name must be an absolute path.
     * @param[in] buffer Buffer to write file from.
     * @param[in] bufferSize Number of bytes in the buffer.
     * @returns number of bytes written.
     */
    size_t writeFile(const String& fileName, const char* buffer, const uint32_t bufferSize) final;

private:
    /* Not allowed. */
    FileHandler(const FileHandler& src);            /**< Copy construction of an instance. */
    FileHandler& operator=(const FileHandler& rhs); /**< Assignment of an instance. */
};

>>>>>>>> main:lib/HALSim/src/FileHandler.h
/******************************************************************************
 * Prototypes
 *****************************************************************************/

<<<<<<<< HEAD:lib/ZumoOta/src/FlashManager.cpp
/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

FlashManager::FlashManager()
{
}

FlashManager::~FlashManager()
{
}
========
#endif /* FILE_HANDLER_H_ */
/** @} */
>>>>>>>> main:lib/HALSim/src/FileHandler.h
