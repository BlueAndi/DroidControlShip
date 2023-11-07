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
 * @brief  Arduino native
 * @author Andreas Merkle <web@blue-andi.de>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <time.h>
#include "Terminal.h"

#ifndef UNIT_TEST
#include <Board.h>
#include <Device.h>
#include <getopt.h>
#include <direct.h>
#include <ArduinoJson.h>
#endif

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/** This type defines the possible program arguments. */
typedef struct
{
    const char* robotName;         /**< Unique robot name */
    const char* mqttHost;          /**< MQTT broker host */
    const char* mqttPort;          /**< MQTT broker port */
    const char* radonUlzerAddress; /**< Radon Ulzer (socket) address */
    const char* radonUlzerPort;    /**< Radon Ulzer (socket) port */
    bool        verbose;           /**< Show verbose information */

} PrgArguments;

/******************************************************************************
 * Prototypes
 *****************************************************************************/

extern void setup();
extern void loop();
static int  handleCommandLineArguments(PrgArguments& prgArguments, int argc, char** argv);
static void showPrgArguments(const PrgArguments& prgArgs);
static bool fileExists(const char* filePath);
static int  createConfigFile(const PrgArguments& prgArgs);
static int  makeDirRecursively(const char* path);
static void extractDirectoryPath(const char* filePath, char* buffer, size_t bufferSize);

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Terminal/Console stream. */
static Terminal gTerminalStream;

/** Serial driver, used by Arduino applications. */
Serial_ Serial(gTerminalStream);

/** Supported long program arguments. */
static const struct option LONG_OPTIONS[] = {{"help", no_argument, nullptr, 0},
                                             {"mqttAddr", required_argument, nullptr, 0},
                                             {"mqttPort", required_argument, nullptr, 0},
                                             {"radonUlzerAddr", required_argument, nullptr, 0},
                                             {"radonUlzerPort", required_argument, nullptr, 0},
                                             {nullptr, no_argument, nullptr, 0}}; /* Marks the end. */

/** Program argument default value of the robot name. */
static const char* PRG_ARG_ROBOT_NAME_DEFAULT = "";

/** Program argument default value of the MQTT broker address. */
static const char* PRG_ARG_MQTT_ADDR_DEFAULT = "localhost";

/** Program argument default value of the MQTT broker port. */
static const char* PRG_ARG_MQTT_PORT_DEFAULT = "1883";

/** Program argument default value of the Radon Ulzer address. */
static const char* PRG_ARG_RADON_ULZER_ADDR_DEFAULT = "localhost";

/** Program argument default value of the Radon Ulzer port. */
static const char* PRG_ARG_RADON_ULZER_PORT_DEFAULT = "1883";

/** Program argument default value of the verbose flag. */
static bool PRG_ARG_VERBOSE_DEFAULT = false;

/** Default value of the wifi SSID. */
static const char* WIFI_SSID_DEFAULT = "";

/** Default value of the wifi passphrase. */
static const char* WIFI_PASSPHRASE_DEFAULT = "";

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

#ifdef UNIT_TEST

extern int main(int argc, char** argv)
{
    setup(); /* Prepare test */
    loop();  /* Run test once */

    return 0;
}

#else

extern int main(int argc, char** argv)
{
    int          status = 0;
    PrgArguments prgArguments;

    printf("*** Droid Control Ship ***\n");

    /* Remove any buffering from stout and stderr to get the printed information immediately. */
    (void)setvbuf(stdout, NULL, _IONBF, 0);
    (void)setvbuf(stderr, NULL, _IONBF, 0);

    status = handleCommandLineArguments(prgArguments, argc, argv);

    if (0 == status)
    {
        /* Show used arguments only in verbose mode. */
        if (true == prgArguments.verbose)
        {
            showPrgArguments(prgArguments);
        }

        /* If no configuration file exists, create one. */
        if (false == fileExists(CONFIG_FILE_PATH))
        {
            status = createConfigFile(prgArguments);
        }

        if (0 == status)
        {
            /*
             * Set Device Server from command line arguments.
             * Uses the Device Native Interface IDeviceNative instead of IDevice from HALInterfaces.
             */
            IDeviceNative& deviceNativeInterface = Board::getInstance().getDeviceNative();
            deviceNativeInterface.setServer(prgArguments.radonUlzerAddress, prgArguments.radonUlzerPort);

            setup();

            while (true)
            {
                loop();
            }
        }
    }

    return status;
}

#endif

extern unsigned long millis()
{
    clock_t now = clock();

    return (now * 1000UL) / CLOCKS_PER_SEC;
}

extern void delay(unsigned long ms)
{
    unsigned long timestamp = millis();

    while ((millis() - timestamp) < ms)
    {
        ;
    }
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

#ifdef UNIT_TEST

/**
 * Handle the arguments passed to the programm.
 *
 * @param[out]  prgArguments Parsed program arguments
 * @param[in]   argc         Program argument count
 * @param[in]   argv         Program argument vector
 *
 * @returns 0 if handling was succesful. Otherwise, -1
 */
static int handleCommandLineArguments(PrgArguments& prgArguments, int argc, char** argv)
{
    /* Not implemented. */
    (void)prgArguments;
    (void)argc;
    (void)argv;

    return 0;
}

#else

/**
 * Handle the arguments passed to the programm.
 * If a argument is not given via command line interface, its default value will be used.
 *
 * @param[out]  prgArguments    Parsed program arguments
 * @param[in]   argc            Program argument count
 * @param[in]   argv            Program argument vector
 *
 * @returns 0 if handling was succesful. Otherwise, -1
 */
static int handleCommandLineArguments(PrgArguments& prgArguments, int argc, char** argv)
{
    int         status           = 0;
    const char* availableOptions = "n:v";
    const char* programName      = argv[0];
    int         optionIndex      = 0;
    int         option           = getopt_long(argc, argv, availableOptions, LONG_OPTIONS, &optionIndex);

    /* Set default values */
    prgArguments.robotName         = PRG_ARG_ROBOT_NAME_DEFAULT;
    prgArguments.mqttHost          = PRG_ARG_MQTT_ADDR_DEFAULT;
    prgArguments.mqttPort          = PRG_ARG_MQTT_PORT_DEFAULT;
    prgArguments.radonUlzerAddress = PRG_ARG_RADON_ULZER_ADDR_DEFAULT;
    prgArguments.radonUlzerPort    = PRG_ARG_RADON_ULZER_PORT_DEFAULT;
    prgArguments.verbose           = PRG_ARG_VERBOSE_DEFAULT;

    while ((-1 != option) && (0 == status))
    {
        switch (option)
        {
        case 0: /* Long option */
            if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "help"))
            {
                status = -1;
            }
            else if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "mqttAddr"))
            {
                prgArguments.mqttHost = optarg;
            }
            else if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "mqttPort"))
            {
                prgArguments.mqttPort;
            }
            else if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "radonUlzerAddr"))
            {
                prgArguments.radonUlzerAddress = optarg;
            }
            else if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "radonUlzerPort"))
            {
                prgArguments.radonUlzerPort;
            }
            else
            {
                status = -1;
            }
            break;

        case 'n': /* Name */
            prgArguments.robotName = optarg;
            break;

        case 'v': /* Verbose */
            prgArguments.verbose = true;
            break;

        case '?': /* Unknown */
            /* fallthrough */

        case 'h': /* Help */
            /* fallthrough */

        default: /* Default */
            status = -1;
            break;
        }

        option = getopt_long(argc, argv, availableOptions, LONG_OPTIONS, &optionIndex);
    }

    /* Does the user need help? */
    if (0 > status)
    {
        printf("Usage: %s <option(s)>\nOptions:\n", programName);
        printf("\t-h, --help\t\t\tShow this help message.\n");                     /* Help */
        printf("\t-n <NAME>\t\t\tSet robot name, which shall be unique.\n");       /* Robot name */
        printf("\t-v\t\t\t\tSet verbose mode.\n");                                 /* Verbose mode */
        printf("\t--mqttAddr <MQTT-ADDR>\t\tSet MQTT broker address.\n");          /* MQTT broker address */
        printf("\t--mqttPort <MQTT-PORT>\t\tSet MQTT broker port.\n");             /* MQTT broker port */
        printf("\t--radonUlzerAddr <RU-ADDR>\tSet Radon Ulzer server address.\n"); /* Radon Ulzer server address */
        printf("\t--radonUlzerPort <RU-PORT>\tSet Radon Ulzer server port.\n");    /* Radon Ulzer server port */
    }

    return status;
}

/**
 * Show program arguments on the console.
 *
 * @param[in] prgArgs   Program arguments
 */
static void showPrgArguments(const PrgArguments& prgArgs)
{
    printf("Robot name         : %s\n", prgArgs.robotName);
    printf("MQTT broker address: %s\n", prgArgs.mqttHost);
    printf("MQTT broker port   : %s\n", prgArgs.mqttPort);
    printf("Radon Ulzer address: %s\n", prgArgs.radonUlzerAddress);
    printf("Radon Ulzer port   : %s\n", prgArgs.radonUlzerPort);
    /* Skip verbose flag. */
}

/**
 * Checks whether a file exists in the filesystem.
 *
 * @param[in] filePath  Path with filename.
 *
 * @return If file exists, it will return true otherwise false.
 */
static bool fileExists(const char* filePath)
{
    bool  isAvailable = false;
    FILE* fd          = fopen(filePath, "r");

    if (nullptr != fd)
    {
        fclose(fd);
        isAvailable = true;
    }

    return isAvailable;
}

/**
 * Create configuration settings file with program arguments.
 *
 * @param[in] prgArgs   Program arguments
 *
 * @return If successful created, it will return 0 otherwise -1;
 */
static int createConfigFile(const PrgArguments& prgArgs)
{
    int    retValue         = 0;
    size_t len              = strlen(CONFIG_FILE_PATH);
    char*  pathToConfigFile = new char[len + 1U];

    if (nullptr == pathToConfigFile)
    {
        retValue = -1;
    }
    else
    {
        pathToConfigFile[0] = '\0';
        extractDirectoryPath(CONFIG_FILE_PATH, pathToConfigFile, sizeof(pathToConfigFile));

        if (0 != makeDirRecursively(pathToConfigFile))
        {
            printf("Failed to create config file directory.\n");
            retValue = -1;
        }
        else
        {
            FILE* fd = fopen(CONFIG_FILE_PATH, "w");

            if (nullptr == fd)
            {
                printf("Failed to create config file.\n");
                retValue = -1;
            }
            else
            {
                const size_t        JSON_DOC_SIZE = 2048U;
                DynamicJsonDocument jsonDoc(JSON_DOC_SIZE);

                jsonDoc["robotName"]    = prgArgs.robotName;
                jsonDoc["WIFI"]["SSID"] = WIFI_SSID_DEFAULT;
                jsonDoc["WIFI"]["PSWD"] = WIFI_PASSPHRASE_DEFAULT;
                jsonDoc["MQTT"]["HOST"] = prgArgs.mqttHost;
                jsonDoc["MQTT"]["PORT"] = prgArgs.mqttPort;

                {
                    size_t jsonBufferSize = measureJsonPretty(jsonDoc) + 1U;
                    char   jsonBuffer[jsonBufferSize];

                    (void)serializeJsonPretty(jsonDoc, jsonBuffer, jsonBufferSize);

                    fprintf(fd, "%s", jsonBuffer);
                }

                fclose(fd);
            }
        }
    }

    return retValue;
}

/**
 * Create directories recursively.
 *
 * @param[in] path  Path which to create.
 *
 * @return If successful, it will return 0 otherwise -1.
 */
static int makeDirRecursively(const char* path)
{
    int    retVal = 0;
    size_t len    = strlen(path);
    char*  dir    = new char[len + 1];

    if (nullptr == dir)
    {
        printf("Directory buffer creation failed.\n");
        retVal = -1;
    }
    else
    {
        strcpy(dir, path);

        for (size_t idx = 0U; idx < len; ++idx)
        {
            if ('/' == dir[idx])
            {
                /* Temporarily truncate the string. */
                dir[idx] = '\0';

                if ((_mkdir(dir) != 0) && (errno != EEXIST))
                {
                    printf("Failed to create directory: %s\n", dir);
                    retVal = -1;
                }

                dir[idx] = '/';
            }

            if (0 > retVal)
            {
                break;
            }
        }

        if (0 == retVal)
        {
            if ((_mkdir(dir) != 0) && (errno != EEXIST))
            {
                printf("Failed to create directory: %s\n", dir);
                retVal = -1;
            }
        }

        delete[] dir;
    }

    return 0;
}

/**
 * Extract the directory path from a file path.
 *
 * @param[in]   filePath    Path which includes the filename.
 * @param[out]  buffer      Buffer with extracted directories.
 * @param[in]   bufferSize  Buffer size in byte.
 */
static void extractDirectoryPath(const char* filePath, char* buffer, size_t bufferSize)
{
    if ((nullptr != buffer) && (0 < bufferSize))
    {
        /* Find the last occurrence of the directory separator character (e.g., '/') */
        const char* lastSeparator = strrchr(filePath, '/');

        if (lastSeparator != nullptr)
        {
            /* Calculate the length of the directory path. */
            size_t length = lastSeparator - filePath;

            if (length < bufferSize)
            {
                strncpy(buffer, filePath, length);
                buffer[length] = '\0';
            }
        }
    }
}

#endif