/* MIT License

Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

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
     * Get FileReader instance.
     * @returns FileReader instance.
     */
    static FileReader& getInstance()
    {
        static FileReader instance; /* Idiom. */
        return instance;
    }

    /**
     * Read a file from the filesystem.
     * @param[in] fileName Name of the file to read. Name must be an absolute path.
     * @param[out] outBuffer Buffer to write file to.
     * @param[in] maxBufferSize Max. number of bytes in the buffer.
     * @returns true if file has been read to the buffer succesfully. Otherwise, false.
     */
    bool readFile(const String& fileName, char* outBuffer, const uint32_t maxBufferSize) final;

private:
    /**
     * Constructs the concrete FileReader.
     */
    FileReader();

    /**
     * Destroys the concrete FileReader.
     */
    virtual ~FileReader();

private:
    FileReader(const FileReader& src);
    FileReader& operator=(const FileReader& rhs);
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* FILE_READER_H_ */