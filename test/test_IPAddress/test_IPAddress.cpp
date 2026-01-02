/* MIT License
 *
 * Copyright (c) 2023 - 2026 Andreas Merkle <web@blue-andi.de>
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
 * @file
 * @brief  Test (some) IPAddress functions.
 * @author Norbert Schulz <schulz.norbert@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <unity.h>
#include <IPAddress.cpp>

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

static void testIPAddrConstruction(void);
static void testIPAddrString(void);
static void testIPAddrOperator(void);

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

    RUN_TEST(testIPAddrConstruction);
    RUN_TEST(testIPAddrString);
    RUN_TEST(testIPAddrOperator);

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
 * Test IPAddr construction.
 */
static void testIPAddrConstruction(void)
{
    IPAddress ipDefault;
    TEST_ASSERT_EQUAL_UINT32(0x00000000, static_cast<uint32_t>(ipDefault));

    IPAddress ipInt32(0xBADCAFFE);
    TEST_ASSERT_EQUAL_UINT32(0xBADCAFFE, static_cast<uint32_t>(ipInt32));

    IPAddress ip;
    ip.fromString(String("192.168.1.42"));
    TEST_ASSERT_EQUAL_UINT32(0xC0A8012A, static_cast<uint32_t>(ip));

    IPAddress other(ip);
    TEST_ASSERT_EQUAL_UINT32(0xC0A8012A, static_cast<uint32_t>(other));

    other = ipInt32;
    TEST_ASSERT_EQUAL_UINT32(0xBADCAFFE, static_cast<uint32_t>(other));
}

static void testIPAddrString(void)
{
    IPAddress ip;

    String str("192.168.1.42");
    TEST_ASSERT_TRUE(ip.fromString(str));
    TEST_ASSERT_EQUAL_UINT32(0xC0A8012A, static_cast<uint32_t>(ip));

    String prn(ip.toString());
    TEST_ASSERT_EQUAL_STRING(str.c_str(), prn.c_str());

    String crab("The quick brown fox jumps over the lazy dog");
    TEST_ASSERT_FALSE(ip.fromString(crab));
}

static void testIPAddrOperator(void)
{
    IPAddress ip0;

    TEST_ASSERT_TRUE(ip0 == IPAddress());

    IPAddress ip1(11, 12, 19, 94);
    IPAddress ip2(25, 04, 19, 68);

    TEST_ASSERT_TRUE(ip1 != ip2);
    TEST_ASSERT_FALSE(ip1 == ip2);

    ip1 = ip2;
    TEST_ASSERT_FALSE(ip1 != ip2);
    TEST_ASSERT_TRUE(ip1 == ip2);

    ip1 = ip1; /* self assignmet check */
    TEST_ASSERT_TRUE(ip1 == ip2);
}