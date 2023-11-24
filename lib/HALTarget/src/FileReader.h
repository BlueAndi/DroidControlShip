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
 *  @brief  File Reader
 *  @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

#ifndef FILE_READER_H_
#define FILE_READER_H_

/******************************************************************************
 * Includes
 *****************************************************************************/

#include <IFileReader.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * File Reader class.
 */
class FileReader : public IFileReader
{
public:
    /**
     * Constructs the concrete FileReader.
     */
    FileReader();

    /**
     * Destroys the concrete FileReader.
     */
    virtual ~FileReader();

    /**
     * Read a file from the filesystem.
     * @param[in] fileName Name of the file to read. Name must be an absolute path.
     * @param[out] outBuffer Buffer to write file to.
     * @param[in] maxBufferSize Max. number of bytes in the buffer.
     * @returns number of bytes read.
     */
    size_t readFile(const String& fileName, char* outBuffer, const uint32_t maxBufferSize) final;

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
    FileReader(const FileReader& src);            /**< Copy construction of an instance. */
    FileReader& operator=(const FileReader& rhs); /**< Assignment of an instance. */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* FILE_READER_H_ */