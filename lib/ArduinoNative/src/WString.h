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
 * @brief  String implementation for test
 * @author Andreas Merkle <web@blue-andi.de>
 * @author Luca Dubies <luca.dubies@newtec.de>
 *
 * @addtogroup test
 *
 * @{
 */

#ifndef WSTRING_H
#define WSTRING_H

/******************************************************************************
 * Compile Switches
 *****************************************************************************/

#ifndef ARDUINOJSON_ENABLE_ARDUINO_STRING
#define ARDUINOJSON_ENABLE_ARDUINO_STRING 1
#endif

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdlib.h>
#include <string.h>

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and Classes
 *****************************************************************************/

/**
 * String class for test purposes only.
 */
class String
{
public:
    /**
     * Constructs a string.
     */
    String() : m_size(1U), m_buffer(new char[m_size])
    {
        if (nullptr == m_buffer)
        {
            m_size = 0U;
        }
        else
        {
            m_buffer[0] = '\0';
        }
    }

    /**
     * Destroys a string.
     */
    ~String()
    {
        if (nullptr != m_buffer)
        {
            delete[] m_buffer;
            m_buffer = nullptr;
            m_size   = 0U;
        }
    }

    /**
     * Constructs a string by copying another.
     *
     * @param[in] str String to copy
     */
    String(const String& str) : m_size(str.m_size), m_buffer(nullptr)
    {
        if ((0 == str.m_size) || (nullptr == str.m_buffer))
        {
            m_buffer = new char[1u];

            if (nullptr == m_buffer)
            {
                m_size = 0U;
            }
            else
            {
                m_size       = 1U;
                m_buffer[0u] = '\0';
            }
        }
        else
        {
            m_buffer = new char[str.m_size];

            if (nullptr == m_buffer)
            {
                m_size = 0U;
            }
            else
            {
                m_size = str.m_size;
                strcpy(m_buffer, str.m_buffer);
            }
        }
    }

    /**
     * Constructs a string by copying another.
     *
     * @param[in] str String to copy
     */
    String(const char* str) : m_size((nullptr == str) ? 1u : (strlen(str) + 1U)), m_buffer(new char[m_size])
    {
        if (nullptr == m_buffer)
        {
            m_size = 0U;
        }
        else if (nullptr == str)
        {
            m_buffer[0u] = '\0';
        }
        else
        {
            strcpy(m_buffer, str);
        }
    }

    /**
     * Constructs a string by copying a single character.
     *
     * @param[in] c Single character
     */
    String(char c) : m_size(2U), m_buffer(new char[m_size])
    {
        if (nullptr == m_buffer)
        {
            m_size = 0U;
        }
        else
        {
            m_buffer[0] = c;
            m_buffer[1] = '\0';
        }
    }

    /**
     * Constructs a string by copying another.
     *
     * @param[in] str String to copy
     * @param[in] length String length
     */
    String(const char* str, unsigned int length) :
        m_size((nullptr == str) ? 1U : (length + 1U)),
        m_buffer(new char[m_size])
    {
        if (nullptr == m_buffer)
        {
            m_size = 0U;
        }
        else if (nullptr == str)
        {
            m_buffer[0u] = '\0';
        }
        else
        {
            strncpy(m_buffer, str, length);
            m_buffer[m_size - 1U] = '\0';
        }
    }

    /**
     * Assign a string.
     *
     * @param[in] str String, which to assign.
     *
     * @return String
     */
    String& operator=(const String& str)
    {
        if (this != &str)
        {
            if (nullptr != m_buffer)
            {
                delete[] m_buffer;
                m_buffer = nullptr;
            }

            m_size = str.m_size;

            if (0u < m_size)
            {
                m_buffer = new char[m_size];

                if (nullptr == m_buffer)
                {
                    m_size = 0U;
                }
                else
                {
                    memcpy(m_buffer, str.m_buffer, m_size);
                }
            }
        }

        return *this;
    }

    /**
     * Compare two strings.
     *
     * @param[in] str String, which to compare with.
     *
     * @return If the strings are equal, it will return true otherwise false.
     */
    bool operator==(const String& str) const
    {
        bool result = false;

        if (0 == strcmp(str.c_str(), m_buffer))
        {
            result = true;
        }

        return result;
    }

    /**
     * Compare two strings.
     *
     * @param[in] str String, which to compare with.
     *
     * @return If the strings are equal, it will return true otherwise false.
     */
    bool operator!=(const String& str) const
    {
        return (*this == str) ? false : true;
    }

    /**
     * Get character at given index.
     * If the index is out of bounds, it will return '\0'.
     *
     * @param[in] index Character index in the string.
     *
     * @return Character
     */
    char operator[](unsigned int index) const
    {
        char singleChar = '\0';

        if (length() > index)
        {
            singleChar = m_buffer[index];
        }

        return singleChar;
    }

    String& operator+=(const String& str)
    {
        if (nullptr != str.m_buffer)
        {
            char* tmp = new char[m_size + str.m_size - 1];

            if (nullptr != tmp)
            {
                strcpy(tmp, m_buffer);
                strcat(tmp, str.m_buffer);

                delete[] m_buffer;
                m_buffer = tmp;
                m_size += (str.m_size - 1);
            }
        }

        return *this;
    }

    String& operator+=(const char* cstr)
    {
        size_t cstrLen = (nullptr != cstr) ? strlen(cstr) : 0U;
        if (nullptr != cstr)
        {
            char* tmp = new char[m_size + cstrLen];

            if (nullptr != tmp)
            {
                strcpy(tmp, m_buffer);
                strcat(&tmp[m_size - 1], cstr);

                delete[] m_buffer;
                m_buffer = tmp;
                m_size   = m_size + cstrLen;
            }
        }

        return *this;
    }

    String& operator+=(char c)
    {
        char* tmp = new char[m_size + 1];

        if (nullptr != tmp)
        {
            const char cBuff[2] = {c, '\0'};

            strcpy(tmp, m_buffer);
            strcpy(&tmp[m_size - 1], cBuff);

            delete[] m_buffer;
            m_buffer = tmp;
            ++m_size;
        }

        return *this;
    }

    String operator+(const String& str) const
    {
        String tmp = *this;
        tmp += str;

        return tmp;
    }

    /**
     * Get string as char array.
     *
     * @return Char array
     */
    const char* c_str() const
    {
        static const char* emptyStr = "";
        const char*        buffer   = m_buffer;

        if (nullptr == buffer)
        {
            buffer = emptyStr;
        }

        return buffer;
    }

    /**
     * Get string length.
     *
     * @return String length
     */
    size_t length() const
    {
        unsigned int length = 0;
        const char*  ptr    = m_buffer;

        if (nullptr != m_buffer)
        {
            while ('\0' != *ptr)
            {
                ++length;
                ++ptr;
            }
        }

        return length;
    }

    /**
     * Return the substring from index to the end.
     *
     * @param[in] index Index.
     *
     * @return Substring
     */
    String substring(unsigned int index) const
    {
        return substring(index, length());
    }

    /**
     * Return the substring between left and right index.
     *
     * @param[in] left  Index left
     * @param[in] right Index right
     *
     * @return Substring
     */
    String substring(unsigned int left, unsigned int right) const
    {
        String             out;
        const unsigned int len = length();

        if (left > right)
        {
            unsigned int temp = right;
            right             = left;
            left              = temp;
        }

        if (len > left)
        {
            if (length() < right)
            {
                right = len;
            }

            char temp       = m_buffer[right];
            m_buffer[right] = '\0';

            out = &m_buffer[left];

            m_buffer[right] = temp;
        }

        return out;
    }

    /**
     * Starts string with given pattern?
     *
     * @param[in] s2    Pattern
     *
     * @return If string starts with pattern, it will return true otherwise false.
     */
    unsigned char startsWith(const String& s2) const
    {
        if (length() < s2.length())
        {
            return 0U;
        }

        return startsWith(s2, 0);
    }

    /**
     * Starts string with given pattern from offset?
     *
     * @param[in] s2        Pattern
     * @param[in] offset    Offset
     *
     * @return If string starts with pattern, it will return true otherwise false.
     */
    unsigned char startsWith(const String& s2, unsigned int offset) const
    {
        if ((offset > static_cast<unsigned int>(length() - s2.length())) || (nullptr == m_buffer) ||
            (nullptr == s2.m_buffer))
        {
            return 0;
        }

        return 0 == strncmp(&m_buffer[offset], s2.m_buffer, s2.length());
    }

    /**
     * Clear string.
     */
    void clear()
    {
        if (nullptr != m_buffer)
        {
            m_buffer[0] = '\0';
        }
    }

    /**
     * Is string empty?
     *
     * @return If empty, it will return true otherwise false.
     */
    bool isEmpty() const
    {
        bool isEmptyFlag = true;

        if ((nullptr != m_buffer) && (0U < m_size) && ('\0' != m_buffer[0]))
        {
            isEmptyFlag = false;
        }

        return isEmptyFlag;
    }

    /**
     * If the find pattern is found in the String, it is replaced by the String replace.
     *
     * @param[in] find      Pattern
     * @param[in] replace   Replacement for Pattern
     */
    void replace(const String& find, const String& replace)
    {
        unsigned int replaceLen = replace.length();
        unsigned int findLen    = find.length();

        if ((0 == length()) || (0 == findLen) || (nullptr == m_buffer))
        {
            /* return early if this or find pattern have length 0 */
            return;
        }

        int   diff                  = replaceLen - findLen;
        int   accumulatedLengthDiff = 0;
        char* readFrom              = m_buffer;
        char* foundAt;

        if (0 == diff)
        {
            while (nullptr != (foundAt = strstr(readFrom, find.m_buffer)))
            {
                memcpy(foundAt, replace.m_buffer, replaceLen);
                readFrom = foundAt + replaceLen;
            }
        }
        else if (0 > diff)
        {
            char* writeTo = m_buffer;

            /* copy original and replacement inplace */
            while (nullptr != (foundAt = strstr(readFrom, find.m_buffer)))
            {
                unsigned int numberCharsToCopy = foundAt - readFrom;
                memcpy(writeTo, readFrom, numberCharsToCopy);
                writeTo += numberCharsToCopy;
                memcpy(writeTo, replace.m_buffer, replaceLen);
                writeTo += replaceLen;
                readFrom = foundAt + findLen;
                accumulatedLengthDiff += diff;
            }
            strcpy(writeTo, readFrom);

            /* reduce m_buffer to new size */
            if (0 > accumulatedLengthDiff)
            {
                char* tmp = new char[m_size + accumulatedLengthDiff];
                if (nullptr != tmp)
                {
                    strcpy(tmp, m_buffer);
                }
                delete[] m_buffer;
                m_buffer = tmp;
                m_size += accumulatedLengthDiff;
            }
        }
        else
        {
            /* calculate new buffer size */
            while (nullptr != (foundAt = strstr(readFrom, find.m_buffer)))
            {
                readFrom = foundAt + findLen;
                accumulatedLengthDiff += diff;
            }

            readFrom = m_buffer;
            if (0 < accumulatedLengthDiff)
            {
                /* replace m_buffer with increased size buffer */
                char* tmp = new char[m_size + accumulatedLengthDiff];
                if (nullptr != tmp)
                {
                    char* writeTo = tmp;

                    /* write new buffer from original and replacement */
                    while (nullptr != (foundAt = strstr(readFrom, find.m_buffer)))
                    {
                        unsigned int numberCharsToCopy = foundAt - readFrom;
                        memcpy(writeTo, readFrom, numberCharsToCopy);
                        writeTo += numberCharsToCopy;
                        memcpy(writeTo, replace.m_buffer, replaceLen);
                        writeTo += replaceLen;
                        readFrom = foundAt + findLen;
                    }
                    strcpy(writeTo, readFrom);
                }
                delete[] m_buffer;
                m_buffer = tmp;
                m_size += accumulatedLengthDiff;
            }
        }
    }

    /**
     * Remove all characters from the String starting at the index until the end of the String.
     * @param[in] index Index at which to remove characters.
     */
    void remove(unsigned int index)
    {
        remove(index, (unsigned int)-1);
    }

    /**
     * Remove a number of characters from the String starting at the index.
     * @param[in] index Index at which to remove characters.
     * @param[in] count Number of characters to remove.
     */
    void remove(unsigned int index, unsigned int count)
    {
        if (index >= length())
        {
            return;
        }

        if (0U == count)
        {
            return;
        }

        if (nullptr == m_buffer)
        {
            return;
        }

        if (count > length() - index)
        {
            count = length() - index;
        }

        unsigned int newlen = length() - count;
        char*        tmp    = new char[newlen + 1];

        if (nullptr != tmp)
        {
            memcpy(tmp, m_buffer, index);
            memcpy(tmp + index, m_buffer + index + count, newlen - index);
            tmp[newlen] = '\0';
            delete[] m_buffer;

            m_buffer = tmp;
            m_size   = newlen + 1;
        }
    }

    bool concat(const String& s)
    {
        (void)this->operator+=(s);
        return true;
    }

    bool concat(const char* cstr)
    {
        if ((nullptr == cstr) || (0U == strlen(cstr)))
        {
            return false;
        }

        (void)this->operator+=(cstr);
        return true;
    }

    bool concat(char c)
    {
        (void)this->operator+=(c);
        return true;
    }

private:
    size_t m_size;   /**< String buffer size */
    char*  m_buffer; /**< String buffer */
};

/******************************************************************************
 * Functions
 *****************************************************************************/

#endif /* WSTRING_H */

/** @} */