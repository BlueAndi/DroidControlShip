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
 * @file   test_SettingsHandler.cpp
 * @brief  Test the settings handler.
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <unity.h>
#include <ArduinoJson.h>
#include <ConfigurationKeys.h>
#include <FileSystem.h>
#include <SettingsHandler.h>
#include <Util.h>

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

static void testSaveAndLoad(void);
static void testLoadNotExistingFile(void);
static void testLoadInvalidFile(void);
static void testLoadPartialFile(void);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Name of the configuration file, which is used for the tests. */
static const char TEST_FILE_NAME[] = "/test_SettingsHandler.tmp";

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
    UTIL_NOT_USED(argc);
    UTIL_NOT_USED(argv);

    UNITY_BEGIN();

    RUN_TEST(testSaveAndLoad);
    RUN_TEST(testLoadNotExistingFile);
    RUN_TEST(testLoadInvalidFile);
    RUN_TEST(testLoadPartialFile);

    return UNITY_END();
}

/**
 * Setup a test. This function will be called before every test by unity.
 */
extern void setUp(void)
{
    /* The files are mapped to the working directory. */
    TEST_ASSERT_TRUE(FILESYSTEM.begin());
}

/**
 * Clean up test. This function will be called after every test by unity.
 */
extern void tearDown(void)
{
    if (true == FILESYSTEM.exists(TEST_FILE_NAME))
    {
        (void)FILESYSTEM.remove(TEST_FILE_NAME);
    }

    FILESYSTEM.end();
}

/**
 * Arduino setup function. Its required by the Arduino core, which is linked
 * because of the logging, but its never called.
 */
extern void setup()
{
    /* Not used. */
}

/**
 * Arduino loop function. Its required by the Arduino core, which is linked
 * because of the logging, but its never called.
 */
extern void loop()
{
    /* Not used. */
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

/**
 * Test saving the configuration and loading it again.
 */
static void testSaveAndLoad(void)
{
    SettingsHandler& settings = SettingsHandler::getInstance();

    settings.setRobotName("zumo");
    settings.setWiFiSSID("ssid");
    settings.setWiFiPassword("password");

    TEST_ASSERT_TRUE(settings.saveConfigurationFile(TEST_FILE_NAME));
    TEST_ASSERT_TRUE(FILESYSTEM.exists(TEST_FILE_NAME));

    settings.setRobotName("other");
    settings.setWiFiSSID("other");
    settings.setWiFiPassword("other");

    TEST_ASSERT_TRUE(settings.loadConfigurationFile(TEST_FILE_NAME));
    TEST_ASSERT_EQUAL_STRING("zumo", settings.getRobotName().c_str());
    TEST_ASSERT_EQUAL_STRING("ssid", settings.getWiFiSSID().c_str());
    TEST_ASSERT_EQUAL_STRING("password", settings.getWiFiPassword().c_str());

    /* Saving again replaces the file, even if the content is shorter. */
    settings.setRobotName("z");

    TEST_ASSERT_TRUE(settings.saveConfigurationFile(TEST_FILE_NAME));

    settings.setRobotName("other");

    TEST_ASSERT_TRUE(settings.loadConfigurationFile(TEST_FILE_NAME));
    TEST_ASSERT_EQUAL_STRING("z", settings.getRobotName().c_str());
}

/**
 * Test loading a configuration file, which doesn't exist.
 */
static void testLoadNotExistingFile(void)
{
    SettingsHandler& settings = SettingsHandler::getInstance();

    settings.setRobotName("zumo");

    TEST_ASSERT_FALSE(FILESYSTEM.exists(TEST_FILE_NAME));
    TEST_ASSERT_FALSE(settings.loadConfigurationFile(TEST_FILE_NAME));
    TEST_ASSERT_EQUAL_STRING("zumo", settings.getRobotName().c_str());
}

/**
 * Test loading a configuration file, which contains no JSON.
 */
static void testLoadInvalidFile(void)
{
    SettingsHandler& settings = SettingsHandler::getInstance();
    File             file     = FILESYSTEM.open(TEST_FILE_NAME, FILE_WRITE);

    TEST_ASSERT_TRUE(file);
    TEST_ASSERT_EQUAL_UINT32(7U, file.write(reinterpret_cast<const uint8_t*>("no json"), 7U));
    file.close();

    settings.setRobotName("zumo");

    TEST_ASSERT_FALSE(settings.loadConfigurationFile(TEST_FILE_NAME));
    TEST_ASSERT_EQUAL_STRING("zumo", settings.getRobotName().c_str());
}

/**
 * Test loading a configuration file, which contains only a part of the settings.
 */
static void testLoadPartialFile(void)
{
    SettingsHandler& settings = SettingsHandler::getInstance();
    JsonDocument     jsonDoc;
    File             file = FILESYSTEM.open(TEST_FILE_NAME, FILE_WRITE);

    jsonDoc[ConfigurationKeys::ROBOT_NAME] = "partial";

    TEST_ASSERT_TRUE(file);
    TEST_ASSERT_TRUE(0U < serializeJson(jsonDoc, file));
    file.close();

    settings.setRobotName("zumo");
    settings.setWiFiSSID("ssid");

    /* Only the settings of the file are overwritten. */
    TEST_ASSERT_TRUE(settings.loadConfigurationFile(TEST_FILE_NAME));
    TEST_ASSERT_EQUAL_STRING("partial", settings.getRobotName().c_str());
    TEST_ASSERT_EQUAL_STRING("ssid", settings.getWiFiSSID().c_str());
}
