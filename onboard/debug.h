#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

#ifndef NDEBUG
#define PRINT(x)                                                               \
    { Serial.print(x); }
#define PRINTLN(x)                                                             \
    { Serial.println(x); }
#else
#define PRINT(x) ;
#define PRINTLN(x) ;
#endif

#endif // DEBUG_H
