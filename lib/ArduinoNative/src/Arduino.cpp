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
    const char* robotName;           /**< Unique robot name */
    const char* socketServerAddress; /**< Socket server address */
    const char* socketServerPort;    /**< Socket server port */

} PrgArguments;

/******************************************************************************
 * Prototypes
 *****************************************************************************/

extern void setup();
extern void loop();
static int  handleCommandLineArguments(PrgArguments& prgArguments, int argc, char** argv);
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

    /* Remove any buffering from stout and stderr to get the printed information immediately. */
    (void)setvbuf(stdout, NULL, _IONBF, 0);
    (void)setvbuf(stderr, NULL, _IONBF, 0);

    status = handleCommandLineArguments(prgArguments, argc, argv);

    if (0 == status)
    {
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
            deviceNativeInterface.setServer(prgArguments.socketServerAddress, prgArguments.socketServerPort);

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
    const char* availableOptions = "n:a:p:h";
    const char* programName      = argv[0];
    int         option           = getopt(argc, argv, availableOptions);

    /* Set default values */
    prgArguments.robotName           = nullptr;
    prgArguments.socketServerAddress = nullptr;
    prgArguments.socketServerPort    = nullptr;

    while ((-1 != option) && (0 == status))
    {
        switch (option)
        {
        case 'n': /* Name */
            printf("Robot has been named \"%s\".\n", optarg);
            prgArguments.robotName = optarg;
            break;

        case 'a': /* Address */
        {
            printf("Using Socket Client to connect to \"%s\".\n", optarg);
            prgArguments.socketServerAddress = optarg;
            break;
        }

        case 'p': /* Port */
        {
            printf("Using Socket Client in Port \"%s\".\n", optarg);
            prgArguments.socketServerPort = optarg;
            break;
        }

        case '?': /* Unknown */
            /* fallthrough */

        case 'h': /* Help */
            /* fallthrough */

        default: /* Default */
            printf("Usage: %s <option(s)>\nOptions:\n", programName);
            printf("\t-h\t\t\tShow this help message.\n");          /* Help */
            printf("\t-p <PORT NUMBER>\tSet SocketServer port.\n"); /* Port */
            printf("\t-n <NAME>\t\tSet instace name.");             /* Name */
            status = -1;
            break;
        }

        option = getopt(argc, argv, availableOptions);
    }

    return status;
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
                const char* robotName      = (nullptr != prgArgs.robotName) ? prgArgs.robotName : "";
                const char* wifiSSID       = "";
                const char* wifiPassphrase = "";
                const char* mqttHost  = (nullptr != prgArgs.socketServerAddress) ? prgArgs.socketServerAddress : "";
                const char* mqttPort  = (nullptr != prgArgs.socketServerPort) ? prgArgs.socketServerPort : "1883";
                const char* formatStr = "{\n"
                                        "    \"robotName\": \"%s\",\n"
                                        "    \"WIFI\": {\n"
                                        "        \"SSID\": \"%s\",\n"
                                        "        \"PSWD\": \"%s\"\n"
                                        "    },\n"
                                        "    \"MQTT\": {\n"
                                        "        \"HOST\": \"%s\",\n"
                                        "        \"PORT\": %s\n"
                                        "    }\n"
                                        "}\n";

                fprintf(fd, formatStr, robotName, wifiSSID, wifiPassphrase, mqttHost, mqttPort);
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