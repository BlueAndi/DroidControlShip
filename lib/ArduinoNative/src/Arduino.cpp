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
#include <Settings.h>
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
    const char* instanceName;        /**< Instance name */
    const char* socketServerAddress; /**< Socket server address */
    const char* socketServerPort;    /**< Socket server port */

} PrgArguments;

/******************************************************************************
 * Prototypes
 *****************************************************************************/

extern void setup();
extern void loop();
static int  handleCommandLineArguments(PrgArguments& prgArguments, int argc, char** argv);

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

    status = handleCommandLineArguments(prgArguments, argc, argv);

    /*
     * Set Device Server from command line arguments.
     * Uses the Device Native Interface IDeviceNative instead of IDevice from HALInterfaces.
     */
    IDeviceNative& deviceNativeInterface = Board::getInstance().getDeviceNative();
    deviceNativeInterface.setServer(prgArguments.socketServerAddress, prgArguments.socketServerPort);

    /* Set Settings. */
    if (nullptr != prgArguments.instanceName)
    {
        String clientId(prgArguments.instanceName);
        if (false == Settings::getInstance().setConfiguration(clientId, "", "", "localhost", 1883U))
        {
            status = -1;
        }
    }

    if (0 == status)
    {
        setup();

        while (true)
        {
            loop();
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
    prgArguments.instanceName        = nullptr;
    prgArguments.socketServerAddress = nullptr;
    prgArguments.socketServerPort    = nullptr;

    while ((-1 != option) && (0 == status))
    {
        switch (option)
        {
        case 'n': /* Name */
            printf("Instance has been named \"%s\".\n", optarg);
            prgArguments.instanceName = optarg;
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

#endif