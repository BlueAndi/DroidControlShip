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
 * @brief  Test PlatoonUtils.
 * @author Gabryel Reyes <gabryelrdiaz@gmail.com>
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <Arduino.h>
#include <math.h>
#include <FPMath.h>
#include <unity.h>
#include <Util.h>
#include <PlatoonUtils.h>

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

static void testCalculateAbsoluteDistance(void);
static void testCalculateHeading(void);
static void testCalculateRelativeHeading(void);
static void testCalculateEquivalentHeading(void);
static void testAreWaypointsEqual(void);

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
    UTIL_NOT_USED(argc);
    UTIL_NOT_USED(argv);

    UNITY_BEGIN();

    RUN_TEST(testCalculateAbsoluteDistance);
    RUN_TEST(testCalculateHeading);
    RUN_TEST(testCalculateRelativeHeading);
    RUN_TEST(testCalculateEquivalentHeading);
    RUN_TEST(testAreWaypointsEqual);

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

void testCalculateAbsoluteDistance()
{
    /* Values calculated using GeoGebra graphic calculator. https://www.geogebra.org/calculator */

    Waypoint waypointA(-77, 35, 0, 0, 0, 0);  /* First quadrant. */
    Waypoint waypointB(-78, -55, 0, 0, 0, 0); /* Fourth quadrant. */
    Waypoint waypointC(17, -15, 0, 0, 0, 0);  /* Third quadrant. */
    Waypoint waypointD(60, 60, 0, 0, 0, 0);   /* Second quadrant. */
    Waypoint waypointE(0, 0, 0, 0, 0, 0);     /* Origin. */

    const size_t NUMBER_OF_CASES                = 5U;
    Waypoint     waypointArray[NUMBER_OF_CASES] = {waypointA, waypointB, waypointC, waypointD, waypointE};
    int32_t      distanceMatrix[NUMBER_OF_CASES][NUMBER_OF_CASES] = {{0, 90, 106, 139, 85},
                                                                     {90, 0, 103, 180, 95},
                                                                     {106, 103, 0, 86, 23},
                                                                     {139, 180, 86, 0, 85},
                                                                     {85, 95, 23, 85, 0}};

    for (size_t firstWaypoint = 0; firstWaypoint < NUMBER_OF_CASES; ++firstWaypoint)
    {
        for (size_t secondWaypoint = 0; secondWaypoint < NUMBER_OF_CASES; ++secondWaypoint)
        {
            int32_t distance =
                PlatoonUtils::calculateAbsoluteDistance(waypointArray[firstWaypoint], waypointArray[secondWaypoint]);
            TEST_ASSERT_EQUAL_INT32(distanceMatrix[firstWaypoint][secondWaypoint], distance);
        }
    }
}

void testCalculateHeading()
{
    /* Values calculated using GeoGebra graphic calculator. https://www.geogebra.org/calculator */

    Waypoint waypointA(-77, 35, 0, 0, 0, 0);  /* First quadrant. */
    Waypoint waypointB(-78, -55, 0, 0, 0, 0); /* Fourth quadrant. */
    Waypoint waypointC(17, -15, 0, 0, 0, 0);  /* Third quadrant. */
    Waypoint waypointD(60, 60, 0, 0, 0, 0);   /* Second quadrant. */
    Waypoint waypointE(0, 0, 0, 0, 0, 0);     /* Origin. */

    const size_t NUMBER_OF_CASES                = 5U;
    Waypoint     waypointArray[NUMBER_OF_CASES] = {waypointA, waypointB, waypointC, waypointD, waypointE};
    int32_t      headingMatrix[NUMBER_OF_CASES][NUMBER_OF_CASES] = {{0, 1560, 2653, -2960, 2715},
                                                                    {-1581, 0, -2742, -2446, -2526},
                                                                    {-488, 399, 0, -2090, -722},
                                                                    {180, 695, 1050, 0, 785},
                                                                    {-426, 614, 2419, -2355, 0}};

    for (size_t targetWaypoint = 0; targetWaypoint < NUMBER_OF_CASES; ++targetWaypoint)
    {
        for (size_t referenceWaypoint = 0; referenceWaypoint < NUMBER_OF_CASES; ++referenceWaypoint)
        {
            int32_t resultHeading = 0;
            bool    isSuccessful  = PlatoonUtils::calculateHeading(waypointArray[targetWaypoint],
                                                                   waypointArray[referenceWaypoint], resultHeading);

            if (targetWaypoint == referenceWaypoint)
            {
                TEST_ASSERT_FALSE(isSuccessful);
            }
            else
            {
                TEST_ASSERT_TRUE(isSuccessful);
            }

            TEST_ASSERT_EQUAL_INT32(headingMatrix[targetWaypoint][referenceWaypoint], resultHeading);
        }
    }
}

void testCalculateRelativeHeading()
{
    /* Values calculated using GeoGebra graphic calculator. https://www.geogebra.org/calculator */

    Waypoint targetWaypoint(10, 20, 0, 0, 0, 0);
    Waypoint referenceWaypoint(30, 40, 0, 0, 0, 0);
    int32_t  relativeHeading;

    bool result = PlatoonUtils::calculateRelativeHeading(targetWaypoint, referenceWaypoint, relativeHeading);

    TEST_ASSERT_TRUE(result);
    TEST_ASSERT_EQUAL_INT32(-2355, relativeHeading);
}

void testCalculateEquivalentHeading()
{
    const int32_t smallDelta                                          = 10;
    const size_t  NUMBER_OF_REFERENCE_HEADINGS                        = 7U;
    const size_t  NUMBER_OF_CASES                                     = 19U;
    int32_t       referenceHeadingArray[NUMBER_OF_REFERENCE_HEADINGS] = {
        0,              /* 0 */
        (FP_PI() / 2),  /* 1 */
        -(FP_PI() / 2), /* 2 */
        FP_PI(),        /* 3 */
        -FP_PI(),       /* 4 */
        FP_2PI(),       /* 5 */
        -FP_2PI()       /* 6 */
    };
    int32_t targetHeadingArray[NUMBER_OF_CASES] = {
        0,                           /* 0 */
        smallDelta,                  /* 1 */
        (FP_PI() / 2),               /* 2 */
        (FP_PI() - smallDelta),      /* 3 */
        FP_PI(),                     /* 4 */
        (FP_PI() + smallDelta),      /* 5 */
        (FP_2PI() - (FP_PI() / 2)),  /* 6 */
        (FP_2PI() - smallDelta),     /* 7 */
        FP_2PI(),                    /* 8 */
        (FP_2PI() + smallDelta),     /* 9 */
        -smallDelta,                 /* 10 */
        -(FP_PI() / 2),              /* 11 */
        -(FP_PI() - smallDelta),     /* 12 */
        -FP_PI(),                    /* 13 */
        -(FP_PI() + smallDelta),     /* 14 */
        -(FP_2PI() - (FP_PI() / 2)), /* 15 */
        -(FP_2PI() - smallDelta),    /* 16 */
        -FP_2PI(),                   /* 17 */
        -(FP_2PI() + smallDelta)     /* 18 */
    };

    int32_t equivalentHeadingArray[NUMBER_OF_REFERENCE_HEADINGS][NUMBER_OF_CASES] = {
        {
            /* 0 */
            0,                           /* 0 */
            smallDelta,                  /* 1 */
            (FP_PI() / 2),               /* 2 */
            (FP_PI() - smallDelta),      /* 3 */
            FP_PI(),                     /* 4 */
            -(FP_PI() - smallDelta) - 1, /* 5 */
            -(FP_PI() / 2),              /* 6 */
            -smallDelta,                 /* 7 */
            0,                           /* 8 */
            smallDelta,                  /* 9 */
            -smallDelta,                 /* 10 */
            -(FP_PI() / 2),              /* 11 */
            -(FP_PI() - smallDelta),     /* 12 */
            -FP_PI(),                    /* 13 */
            (FP_PI() - smallDelta) + 1,  /* 14 */
            (FP_PI() / 2),               /* 15 */
            smallDelta,                  /* 16 */
            0,                           /* 17 */
            -smallDelta                  /* 18 */
        },
        {
            /* 1 */
            0,                          /* 0 */
            smallDelta,                 /* 1 */
            (FP_PI() / 2),              /* 2 */
            (FP_PI() - smallDelta),     /* 3 */
            FP_PI(),                    /* 4 */
            (FP_PI() + smallDelta),     /* 5 */
            -(FP_PI() / 2),             /* 6 */
            -smallDelta,                /* 7 */
            0,                          /* 8 */
            smallDelta,                 /* 9 */
            -smallDelta,                /* 10 */
            -(FP_PI() / 2),             /* 11 */
            (FP_PI() + smallDelta) + 1, /* 12 */
            FP_PI() + 1,                /* 13 */
            (FP_PI() - smallDelta) + 1, /* 14 */
            (FP_PI() / 2),              /* 15 */
            smallDelta,                 /* 16 */
            0,                          /* 17 */
            -smallDelta                 /* 18 */
        },
        {
            /* 2 */
            0,                           /* 0 */
            smallDelta,                  /* 1 */
            (FP_PI() / 2),               /* 2 */
            -(FP_PI() + smallDelta) - 1, /* 3 */
            -FP_PI() - 1,                /* 4 */
            -(FP_PI() - smallDelta) - 1, /* 5 */
            -(FP_PI() / 2),              /* 6 */
            -smallDelta,                 /* 7 */
            0,                           /* 8 */
            smallDelta,                  /* 9 */
            -smallDelta,                 /* 10 */
            -(FP_PI() / 2),              /* 11 */
            -(FP_PI() - smallDelta),     /* 12 */
            -FP_PI(),                    /* 13 */
            -(FP_PI() + smallDelta),     /* 14 */
            (FP_PI() / 2),               /* 15 */
            smallDelta,                  /* 16 */
            0,                           /* 17 */
            -smallDelta                  /* 18 */
        },
        {
            /* 3 */
            0,                          /* 0 */
            smallDelta,                 /* 1 */
            (FP_PI() / 2),              /* 2 */
            (FP_PI() - smallDelta),     /* 3 */
            FP_PI(),                    /* 4 */
            (FP_PI() + smallDelta),     /* 5 */
            (FP_2PI() - (FP_PI() / 2)), /* 6 */
            (FP_2PI() - smallDelta),    /* 7 */
            0,                          /* 8 */
            smallDelta,                 /* 9 */
            (FP_2PI() - smallDelta),    /* 10 */
            (FP_2PI() - (FP_PI() / 2)), /* 11 */
            (FP_PI() + smallDelta) + 1, /* 12 */
            FP_PI() + 1,                /* 13 */
            (FP_PI() - smallDelta) + 1, /* 14 */
            (FP_PI() / 2),              /* 15 */
            smallDelta,                 /* 16 */
            0,                          /* 17 */
            (FP_2PI() - smallDelta)     /* 18 */
        },
        {
            /* 4 */
            0,                           /* 0 */
            -(FP_2PI() - smallDelta),    /* 1 */
            -(FP_2PI() - (FP_PI() / 2)), /* 2 */
            -(FP_PI() + smallDelta) - 1, /* 3 */
            -FP_PI() - 1,                /* 4 */
            -(FP_PI() - smallDelta) - 1, /* 5 */
            -(FP_PI() / 2),              /* 6 */
            -smallDelta,                 /* 7 */
            0,                           /* 8 */
            -(FP_2PI() - smallDelta),    /* 9 */
            -10,                         /* 10 */
            -(FP_PI() / 2),              /* 11 */
            -(FP_PI() - smallDelta),     /* 12 */
            -FP_PI(),                    /* 13 */
            -(FP_PI() + smallDelta),     /* 14 */
            -(FP_2PI() - (FP_PI() / 2)), /* 15 */
            -(FP_2PI() - smallDelta),    /* 16 */
            0,                           /* 17 */
            -10                          /* 18 */
        },
        {
            /* 5 */
            FP_2PI(),                              /* 0 */
            (FP_2PI() + smallDelta),               /* 1 */
            (FP_2PI() + (FP_PI() / 2)),            /* 2 */
            (FP_2PI() + FP_PI() - smallDelta),     /* 3 */
            (FP_2PI() + FP_PI()),                  /* 4 */
            (FP_PI() + smallDelta),                /* 5 */
            (FP_2PI() - (FP_PI() / 2)),            /* 6 */
            (FP_2PI() - smallDelta),               /* 7 */
            FP_2PI(),                              /* 8 */
            (FP_2PI() + smallDelta),               /* 9 */
            (FP_2PI() - smallDelta),               /* 10 */
            (FP_2PI() - (FP_PI() / 2)),            /* 11 */
            (FP_PI() + smallDelta) + 1,            /* 12 */
            FP_PI() + 1,                           /* 13 */
            (FP_2PI() + FP_PI() - smallDelta) + 1, /* 14 */
            (FP_2PI() + FP_PI() / 2),              /* 15 */
            (FP_2PI() + smallDelta),               /* 16 */
            FP_2PI(),                              /* 17 */
            (FP_2PI() - smallDelta)                /* 18 */
        },
        {
            /* 6 */
            -FP_2PI(),                              /* 0 */
            -(FP_2PI() - smallDelta),               /* 1 */
            -(FP_2PI() - (FP_PI() / 2)),            /* 2 */
            -(FP_2PI() - FP_PI() + smallDelta),     /* 3 */
            -FP_PI() - 1,                           /* 4 */
            -(FP_2PI() + FP_PI() - smallDelta) - 1, /* 5 */
            -(FP_2PI() + (FP_PI() / 2)),            /* 6 */
            -(FP_2PI() + smallDelta),               /* 7 */
            -FP_2PI(),                              /* 8 */
            -(FP_2PI() - smallDelta),               /* 9 */
            -(FP_2PI() + smallDelta),               /* 10 */
            -(FP_2PI() + FP_PI() / 2),              /* 11 */
            -(FP_2PI() + FP_PI() - smallDelta),     /* 12 */
            -(FP_2PI() + FP_PI()),                  /* 13 */
            -(FP_2PI() - FP_PI() + smallDelta) + 1, /* 14 */
            -(FP_2PI() - FP_PI() / 2),              /* 15 */
            -(FP_2PI() - smallDelta),               /* 16 */
            -FP_2PI(),                              /* 17 */
            -(FP_2PI() + smallDelta)                /* 18 */
        }};

    for (size_t referenceHeadingIdx = 0; referenceHeadingIdx < NUMBER_OF_REFERENCE_HEADINGS; referenceHeadingIdx++)
    {
        for (size_t targetHeadingIdx = 0; targetHeadingIdx < NUMBER_OF_CASES; ++targetHeadingIdx)
        {
            char    message[100];
            int32_t result = PlatoonUtils::calculateEquivalentHeading(targetHeadingArray[targetHeadingIdx],
                                                                      referenceHeadingArray[referenceHeadingIdx]);

            sprintf(message, "Reference heading[%u]: %d ; Target heading[%u]: %d", referenceHeadingIdx,
                    referenceHeadingArray[referenceHeadingIdx], targetHeadingIdx, targetHeadingArray[targetHeadingIdx]);

            TEST_ASSERT_EQUAL_INT32_MESSAGE(equivalentHeadingArray[referenceHeadingIdx][targetHeadingIdx], result,
                                            message);
        }
    }
}

void testAreWaypointsEqual()
{
    Waypoint waypointA(10, 20, 0, 0, 0, 0);
    Waypoint waypointB(10, 20, 0, 0, 0, 0);
    Waypoint waypointC(10, 20, 3142, 0, 0, 0);
    Waypoint waypointD(20, 30, 0, 0, 0, 0);

    /* Equal waypoints. */
    TEST_ASSERT_TRUE(PlatoonUtils::areWaypointsEqual(waypointA, waypointB));

    /* Equal waypoints with different orientation. */
    TEST_ASSERT_TRUE(PlatoonUtils::areWaypointsEqual(waypointB, waypointC));

    /* Different waypoints. */
    TEST_ASSERT_FALSE(PlatoonUtils::areWaypointsEqual(waypointC, waypointD));
}
