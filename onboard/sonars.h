#ifndef SONARS_H
#define SONARS_H

#include <NewPing.h>

class Sonars {
    static constexpr auto NUM_SONAR = 3;

    /**
     * @brief How quickly the NewPing library and sonar sensors support sampling
     */
    static constexpr auto MEASUREMENT_PERIOD_MS = 35;

  public:
    /**
     * @brief Add a sonar to be read from
     * @param triggerPin
     * @param echoPin
     * @param maxDistance [mm] beyond which distance is reported as 0
     * @return Whether sonar was successfully added (if not full).
     */
    static bool addSonar(int triggerPin, int echoPin, int maxDistance = 200);

    /**
     * @brief Main loop function to be run as fast as possible
     */
    static void run();

  private:
    static void echoCheck();

    static unsigned long _nextRunMs;
    static unsigned int _currentNumSonar;
    static unsigned int _mm[NUM_SONAR];
    static NewPing _sonars[NUM_SONAR];
};

#endif // SONARS_H
