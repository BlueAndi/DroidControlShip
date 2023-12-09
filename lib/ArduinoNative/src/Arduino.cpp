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
#include <ConfigurationKeys.h>
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

#ifndef UNIT_TEST

/** This type defines the possible program arguments. */
typedef struct
{
    const char* robotName;         /**< Unique robot name */
    const char* mqttHost;          /**< MQTT broker host */
    const char* mqttPort;          /**< MQTT broker port */
    const char* radonUlzerAddress; /**< Radon Ulzer (socket) address */
    const char* radonUlzerPort;    /**< Radon Ulzer (socket) port */
    const char* cfgFilePath;       /**< Configuration file path */
    bool        verbose;           /**< Show verbose information */
    const char* platoonId;         /**< Platoon ID */
    const char* vehicleId;         /**< Vehicle ID */
} PrgArguments;

#endif

/******************************************************************************
 * Prototypes
 *****************************************************************************/

extern void setup();
extern void loop();

#ifndef UNIT_TEST

static int  handleCommandLineArguments(PrgArguments& prgArguments, int argc, char** argv);
static void showPrgArguments(const PrgArguments& prgArgs);
static bool fileExists(const char* filePath);
static int  createConfigFile(const PrgArguments& prgArgs);
static int  makeDirRecursively(const char* path);
static void extractDirectoryPath(const char* filePath, char* buffer, size_t bufferSize);

#endif

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/** Terminal/Console stream. */
static Terminal gTerminalStream;

/** Serial driver, used by Arduino applications. */
Serial_ Serial(gTerminalStream);

#ifndef UNIT_TEST

/** Supported long program arguments. */
static const struct option LONG_OPTIONS[] = {{"help", no_argument, nullptr, 0},
                                             {"cfgFilePath", required_argument, nullptr, 0},
                                             {"mqttAddr", required_argument, nullptr, 0},
                                             {"mqttPort", required_argument, nullptr, 0},
                                             {"radonUlzerAddr", required_argument, nullptr, 0},
                                             {"radonUlzerPort", required_argument, nullptr, 0},
                                             {"platoonId", required_argument, nullptr, 0},
                                             {"vehicleId", required_argument, nullptr, 0},
                                             {nullptr, no_argument, nullptr, 0}}; /* Marks the end. */

/** Program argument default value of the robot name. */
static const char PRG_ARG_ROBOT_NAME_DEFAULT[] = "";

/** Program argument default value of the MQTT broker address. */
static const char PRG_ARG_MQTT_ADDR_DEFAULT[] = "localhost";

/** Program argument default value of the MQTT broker port. */
static const char PRG_ARG_MQTT_PORT_DEFAULT[] = "1883";

/** Program argument default value of the Radon Ulzer address. */
static const char PRG_ARG_RADON_ULZER_ADDR_DEFAULT[] = "localhost";

/** Program argument default value of the Radon Ulzer port. */
static const char PRG_ARG_RADON_ULZER_PORT_DEFAULT[] = "65432";

/** Program argument default value of the configuration file. */
static const char PRG_ARG_CFG_FILE_DEFAULT[] = "./data/config/config.json";

/** Program argument default value of the Platoon ID. */
static const char PRG_ARG_PLATOON_ID_DEFAULT[] = "0";

/** Program argument default value of the Vehicle ID. */
static const char PRG_ARG_VEHICLE_ID_DEFAULT[] = "0";

/** Program argument default value of the verbose flag. */
static bool PRG_ARG_VERBOSE_DEFAULT = false;

/** Default value of the wifi SSID. */
static const char WIFI_SSID_DEFAULT[] = "";

/** Default value of the wifi passphrase. */
static const char WIFI_PASSPHRASE_DEFAULT[] = "";

/** Default value of the access point SSID. */
static const char AP_SSID_DEFAULT[] = "DCS_AP";

/** Default value of the access point passphrase. */
static const char AP_PASSPHRASE_DEFAULT[] = "hanshotfirst";

/** Default value of the webserver user. */
static const char WEBSERVER_USER_DEFAULT[] = "admin";

/** Default value of the webserver passphrase. */
static const char WEBSERVER_PASSPHRASE_DEFAULT[] = "admin";

#endif

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

    printf("\n*** Droid Control Ship ***\n");

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
        if (false == fileExists(prgArguments.cfgFilePath))
        {
            if (true == prgArguments.verbose)
            {
                printf("Create config file %s.\n", prgArguments.cfgFilePath);
            }

            status = createConfigFile(prgArguments);
        }
        else
        {
            if (true == prgArguments.verbose)
            {
                printf("Use existing config file %s.\n", prgArguments.cfgFilePath);
            }
        }

        if (0 == status)
        {
            Board::getInstance().setConfigFilePath(prgArguments.cfgFilePath);

            /* Set Device Server from command line arguments.
             * Uses the Device Native Interface IDeviceNative instead of IDevice from HALInterfaces.
             */
            IDeviceNative& deviceNativeInterface = Board::getInstance().getDeviceNative();
            deviceNativeInterface.setServer(prgArguments.radonUlzerAddress, prgArguments.radonUlzerPort);

            /* Print a empty line before the main application starts to print log messages. */
            printf("\n");

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

extern bool convertToJson(const String& str, JsonVariant variant)
{
    return variant.set(str.c_str());
}

/******************************************************************************
 * Local Functions
 *****************************************************************************/

#ifndef UNIT_TEST

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
    const char* availableOptions = "n:vh";
    const char* programName      = argv[0];
    int         optionIndex      = 0;
    int         option           = getopt_long(argc, argv, availableOptions, LONG_OPTIONS, &optionIndex);

    /* Set default values */
    prgArguments.cfgFilePath       = PRG_ARG_CFG_FILE_DEFAULT;
    prgArguments.mqttHost          = PRG_ARG_MQTT_ADDR_DEFAULT;
    prgArguments.mqttPort          = PRG_ARG_MQTT_PORT_DEFAULT;
    prgArguments.radonUlzerAddress = PRG_ARG_RADON_ULZER_ADDR_DEFAULT;
    prgArguments.radonUlzerPort    = PRG_ARG_RADON_ULZER_PORT_DEFAULT;
    prgArguments.robotName         = PRG_ARG_ROBOT_NAME_DEFAULT;
    prgArguments.verbose           = PRG_ARG_VERBOSE_DEFAULT;
    prgArguments.platoonId         = PRG_ARG_PLATOON_ID_DEFAULT;
    prgArguments.vehicleId         = PRG_ARG_VEHICLE_ID_DEFAULT;

    while ((-1 != option) && (0 == status))
    {
        switch (option)
        {
        case 0: /* Long option */

            if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "cfgFilePath"))
            {
                prgArguments.cfgFilePath = optarg;
            }
            else if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "help"))
            {
                status = -1;
            }
            else if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "mqttAddr"))
            {
                prgArguments.mqttHost = optarg;
            }
            else if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "mqttPort"))
            {
                prgArguments.mqttPort = optarg;
            }
            else if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "radonUlzerAddr"))
            {
                prgArguments.radonUlzerAddress = optarg;
            }
            else if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "radonUlzerPort"))
            {
                prgArguments.radonUlzerPort = optarg;
            }
            else if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "platoonId"))
            {
                prgArguments.platoonId = optarg;
            }
            else if (0 == strcmp(LONG_OPTIONS[optionIndex].name, "vehicleId"))
            {
                prgArguments.vehicleId = optarg;
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
        printf("\t-h, --help\t\t\tShow this help message.\n");              /* Help */
        printf("\t-n <NAME>\t\t\tSet robot name, which shall be unique.");  /* Robot name */
        printf(" Default: Derived from the process PID.\n");                /* Robot name default value */
        printf("\t-v\t\t\t\tSet verbose mode. Default: Disabled\n");        /* Verbose mode */
        printf("\t--cfgFilePath <CFG-FILE>\tSet configuration file path."); /* Configuration file path */
        printf(" Default: %s\n", PRG_ARG_CFG_FILE_DEFAULT);                 /* Configuration file path default value */
        printf("\t--mqttAddr <MQTT-ADDR>\t\tSet MQTT broker address.");     /* MQTT broker address */
        printf(" Default: %s\n", PRG_ARG_MQTT_ADDR_DEFAULT);                /* MQTT broker address default value */
        printf("\t--mqttPort <MQTT-PORT>\t\tSet MQTT broker port.");        /* MQTT broker port */
        printf(" Default: %s\n", PRG_ARG_MQTT_PORT_DEFAULT);                /* MQTT broker port default value */
        printf("\t--radonUlzerAddr <RU-ADDR>\tSet Radon Ulzer server address."); /* Radon Ulzer server address */
        printf(" Default: %s\n", PRG_ARG_RADON_ULZER_ADDR_DEFAULT); /* Radon Ulzer server address default value*/
        printf("\t--radonUlzerPort <RU-PORT>\tSet Radon Ulzer server port."); /* Radon Ulzer server port */
        printf(" Default: %s\n", PRG_ARG_RADON_ULZER_PORT_DEFAULT);           /* Radon Ulzer server port default value*/
        printf("\t--platoonId <PLATOON-ID>\tSet platoon ID.");                /* Platoon ID */
        printf(" Default: %s\n", PRG_ARG_PLATOON_ID_DEFAULT);                 /* Platoon ID default value */
        printf("\t--vehicleId <VEHICLE-ID>\tSet vehicle ID.");                /* Vehicle ID */
        printf(" Default: %s\n", PRG_ARG_VEHICLE_ID_DEFAULT);                 /* Vehicle ID default value */
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
    printf("Configuration file path: %s\n", prgArgs.cfgFilePath);
    printf("Robot name             : %s\n", prgArgs.robotName);
    printf("MQTT broker address    : %s\n", prgArgs.mqttHost);
    printf("MQTT broker port       : %s\n", prgArgs.mqttPort);
    printf("Radon Ulzer address    : %s\n", prgArgs.radonUlzerAddress);
    printf("Radon Ulzer port       : %s\n", prgArgs.radonUlzerPort);
    printf("Platoon ID             : %s\n", prgArgs.platoonId);
    printf("Vehicle ID             : %s\n", prgArgs.vehicleId);
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
 * The location of the configuration settings is defined inside
 * the program arguments!
 *
 * @param[in] prgArgs   Program arguments
 *
 * @return If successful created, it will return 0 otherwise -1;
 */
static int createConfigFile(const PrgArguments& prgArgs)
{
    int    retValue         = 0;
    size_t len              = strlen(prgArgs.cfgFilePath);
    char*  pathToConfigFile = new char[len + 1U];

    if (nullptr == pathToConfigFile)
    {
        retValue = -1;
    }
    else
    {
        pathToConfigFile[0] = '\0';
        extractDirectoryPath(prgArgs.cfgFilePath, pathToConfigFile, sizeof(pathToConfigFile));

        if (('\0' != pathToConfigFile[0]) && (0 != makeDirRecursively(pathToConfigFile)))
        {
            printf("Failed to create config file directory.\n");
            retValue = -1;
        }
        else
        {
            FILE* fd = fopen(prgArgs.cfgFilePath, "w");

            if (nullptr == fd)
            {
                printf("Failed to create config file.\n");
                retValue = -1;
            }
            else
            {
                const size_t        JSON_DOC_SIZE = 2048U;
                DynamicJsonDocument jsonDoc(JSON_DOC_SIZE);

                jsonDoc[ConfigurationKeys::ROBOT_NAME]                             = prgArgs.robotName;
                jsonDoc[ConfigurationKeys::WIFI][ConfigurationKeys::SSID]          = WIFI_SSID_DEFAULT;
                jsonDoc[ConfigurationKeys::WIFI][ConfigurationKeys::PASSWORD]      = WIFI_PASSPHRASE_DEFAULT;
                jsonDoc[ConfigurationKeys::MQTT][ConfigurationKeys::HOST]          = prgArgs.mqttHost;
                jsonDoc[ConfigurationKeys::MQTT][ConfigurationKeys::PORT]          = prgArgs.mqttPort;
                jsonDoc[ConfigurationKeys::AP][ConfigurationKeys::SSID]            = AP_SSID_DEFAULT;
                jsonDoc[ConfigurationKeys::AP][ConfigurationKeys::PASSWORD]        = AP_PASSPHRASE_DEFAULT;
                jsonDoc[ConfigurationKeys::WEBSERVER][ConfigurationKeys::USER]     = WEBSERVER_USER_DEFAULT;
                jsonDoc[ConfigurationKeys::WEBSERVER][ConfigurationKeys::PASSWORD] = WEBSERVER_PASSPHRASE_DEFAULT;
                jsonDoc[ConfigurationKeys::PLATOON][ConfigurationKeys::PLATOON_ID] = prgArgs.platoonId;
                jsonDoc[ConfigurationKeys::PLATOON][ConfigurationKeys::VEHICLE_ID] = prgArgs.vehicleId;

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