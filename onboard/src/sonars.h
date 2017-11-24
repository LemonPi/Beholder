#ifndef SONARS_H
#define SONARS_H

#include <NewPing.h>
#include "types.h"

class Sonars {

    /**
     * @brief How quickly the NewPing library and sonar sensors support sampling
     */
    static constexpr auto MEASUREMENT_PERIOD_MS = 33;
    /**
     * @brief How many rounds of sensor readings of too far before we consider
     * it as being actually too far. Roughly acts as a median filter against
     * missing echos
     */
    static constexpr auto CONSECUTIVE_ROUNDS_TOO_FAR = 3;

  public:
    static constexpr auto MAX_DIST_MM = 500;
    enum SonarIndex { LEFT, FRONT, RIGHT, NUM_SONAR };

    /**
     * @brief Call in setup to stagger the times we try to ping with sonar to
     * avoid cross contamination
     */
    static void setupPingTimers();

    /**
     * @brief Main loop function to be run as fast as possible
     */
    static void run();

    /**
     * @brief Get last sonar reading [mm]
     * @param index Index of sonar
     */
    static sonar_reading_t getReading(SonarIndex index);
    /**
     * @brief Check whether a reading is too far or at a very far distance
     * @param index Index of sonar
     * @return True if a reading of 0 means it's too far
     */
    static bool isTooFar(SonarIndex index);

  private:
    static void echoCheck();

    static unsigned long _nextRunMs;
    static sonar_reading_t _mm[NUM_SONAR];
    //    static unsigned int _lastMm[NUM_SONAR];
    static unsigned long _pingTimer[NUM_SONAR];
    static int _consecutiveRoundsTooFar[NUM_SONAR];
    static int _currentSonar;

    static NewPing _sonars[NUM_SONAR];
};

#endif // SONARS_H
