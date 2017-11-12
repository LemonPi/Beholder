#ifndef SONARNONBLOCKING_H
#define SONARNONBLOCKING_H

class SonarNonBlocking {
    enum SensorPhase {
        TRIGGER, // sensor is triggered to send pings
        SENDING, // wait until all pings have been sent by the sensor
        RECEIVE  // await the echo of the sensor pings
    };
    // error if it takes longer than this time to send pings [us]
    static constexpr auto PING_MAX_DURATION_US = 500UL;
    static constexpr auto PULSE_MAX_DURATION_US = 20000UL;
    static constexpr float SPEED_OF_SOUND_MM_PER_S = 0.0034 / 2;

  public:
    SonarNonBlocking(int triggerPin, int echoPin);
    /**
     * @brief Continue reading from sensor without blocking; should be run
     * as fast as possible
     * @return Whether a new reading was retrieved
     */
    bool run();
    /**
     * @brief Get latest completed distance reading
     * @return Reading in [mm]
     */
    unsigned int getReading() const;

  private:
    int _triggerPin, _echoPin;
    // The current phase of the sensor
    enum SensorPhase _phase;
    // Start time of this phase [us]
    unsigned long _phaseStartUs;
    // The time between triggering the pulse and receiving the echo
    unsigned long _pulseDuration;
    // The currently measured distance [mm]
    unsigned int _distMm;
    unsigned int _lastDistMm;
};

#endif // SONARNONBLOCKING_H
