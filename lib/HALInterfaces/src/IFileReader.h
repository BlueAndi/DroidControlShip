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
 *  @brief  File Reader Interface
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

#ifndef IFILE_READER_H_
#define IFILE_READER_H_

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <stdint.h>
#include <WString.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * Abstracts the File Reader interface.
 */
class IFileReader
{
public:
    /**
     * Destructs the File Reader Interface.
     */
    virtual ~IFileReader()
    {
    }

    /**
     * Read a file from the filesystem.
     * @param[in] fileName Name of the file to read. Name must be an absolute path.
     * @param[out] outBuffer Buffer to write file to.
     * @param[in] maxBufferSize Max. number of bytes in the buffer.
     * @returns number of bytes read.
     */
    virtual size_t readFile(const String& fileName, char* outBuffer, const uint32_t maxBufferSize) = 0;

    /**
     * Write a file to the filesystem.
     * @param[in] fileName Name of the file to write. Name must be an absolute path.
     * @param[in] buffer Buffer to write file from.
     * @param[in] bufferSize Number of bytes in the buffer.
     * @returns number of bytes written.
     */
    virtual size_t writeFile(const String& fileName, const char* buffer, const uint32_t bufferSize) = 0;

protected:
    /**
     * Contructs the File Reader Interface.
     */
    IFileReader()
    {
    }
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* IFILE_READER_H_ */