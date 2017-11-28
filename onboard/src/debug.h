#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

#ifndef NDEBUG
#define PRINT(x)                                                               \
    { Serial3.print(x); }
#define PRINTLN(x)                                                             \
    { Serial3.println(x); }
#define ERROR(x)                                                               \
    {                                                                          \
        Serial3.print("ERR ");                                                 \
        Serial3.println(x);                                                    \
    }
#else
#define PRINT(x) ;
#define PRINTLN(x) ;
#define ERROR(x) ;
#endif

#endif // DEBUG_H
