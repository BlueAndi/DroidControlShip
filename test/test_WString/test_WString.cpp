/* MIT License
 *
 * Copyright (c) 2019 - 2023 Andreas Merkle <web@blue-andi.de>
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
 * @brief  Test (some) WString functions.
 * @author Luca Dubies <luca.dubies@newtec.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <unity.h>
#include <WString.h>

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

static void testWStringReplacement(void);
static void testWStringAppend(void);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

/******************************************************************************
 * Protected Methods
 *****************************************************************************/

/******************************************************************************
 * Private Methods
 *****************************************************************************/

/******************************************************************************
 * External Functions
 *****************************************************************************/

/**
 * Main entry point
 *
 * @param[in] argc  Number of command line arguments
 * @param[in] argv  Command line arguments
 *
 * @returns Test failure count
 */
extern int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(testWStringReplacement);
    RUN_TEST(testWStringAppend);

    return UNITY_END();
}

/**
 * Setup a test. This function will be called before every test by unity.
 */
extern void setUp(void)
{
    /* Not used. */
}

/**
 * Clean up test. This function will be called after every test by unity.
 */
extern void tearDown(void)
{
    /* Not used. */
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * Test WString replace.
 */
static void testWStringReplacement(void)
{
    String original = String("I am a long string that will feature some replacements!");
    String tester;

    /* same length replacement */
    tester = String(original);
    tester.replace(String("long"), String("same"));
    TEST_ASSERT_EQUAL_STRING("I am a same string that will feature some replacements!", tester.c_str());

    /* shorter replacement */
    tester = String(original);
    tester.replace(String("feature"), String("have"));
    TEST_ASSERT_EQUAL_STRING("I am a long string that will have some replacements!", tester.c_str());
    TEST_ASSERT_EQUAL(original.length() - 3, tester.length());

    /* longer replacement */
    tester = String(original);
    tester.replace("a", "AAA");
    TEST_ASSERT_EQUAL_STRING("I AAAm AAA long string thAAAt will feAAAture some replAAAcements!", tester.c_str());
    TEST_ASSERT_EQUAL(original.length() + 10, tester.length());

    /* replacement with empty string */
    tester = String(original);
    tester.replace(" ", "");
    TEST_ASSERT_EQUAL_STRING("Iamalongstringthatwillfeaturesomereplacements!", tester.c_str());

    /* test original empty string */
    tester = String("");
    tester.replace("never", "happens");
    TEST_ASSERT_EQUAL_STRING("", tester.c_str());
}

/**
 * Test WString += implementation.
 */
static void testWStringAppend(void)
{
    String original = String("Im short");
    char classic[4];
    classic[0]      = 'H';
    classic[1]      = 'I';
    classic[2]      = '!';
    classic[3]      = '\0';

    original += String("- first amendment -");
    original += classic;
    original += '-';

    TEST_ASSERT_EQUAL_STRING("Im short- first amendment -HI!-", original.c_str());

    String tester = String("");
    tester += "Im new";
    TEST_ASSERT_EQUAL_STRING("Im new", tester.c_str());
}
