#ifndef SONARS_H
#define SONARS_H

#include <NewPing.h>

class Sonars {

    /**
     * @brief How quickly the NewPing library and sonar sensors support sampling
     */
    static constexpr auto MEASUREMENT_PERIOD_MS = 33;

  public:
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
    static unsigned int getReading(SonarIndex index);

  private:
    static void echoCheck();

    static unsigned long _nextRunMs;
    static unsigned int _mm[NUM_SONAR];
    static unsigned long _pingTimer[NUM_SONAR];
    static int _currentSonar;

    static NewPing _sonars[NUM_SONAR];
};

#endif // SONARS_H
