#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

#ifndef NDEBUG
#define PRINT(x)                                                               \
    { Serial.print(x); }
#define PRINTLN(x)                                                             \
    { Serial.println(x); }
#define ERROR(x)                                                               \
    {                                                                          \
        Serial.print("ERR ");                                                  \
        Serial.println(x);                                                     \
    }
#else
#define PRINT(x) ;
#define PRINTLN(x) ;
#define ERROR(x) ;
#endif

#endif // DEBUG_H
