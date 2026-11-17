#ifndef _PROS_AUTOS_H_
#define _PROS_AUTOS_H_
#include "main.h"

#define AUTO_MODES \
    X(auto1) \
    X(auto2) \
    X(autoTest) \
    X(autoOffset)

#define X(name) name,
enum class AutoMode {
    AUTO_MODES
};
#undef X

#define X(name) #name,
inline const char* AutoModeNames[] = {
    AUTO_MODES
};
#undef X

#define X(name) + 1
const int TOTAL = 0 AUTO_MODES;
#undef X

extern AutoMode autoMode;

#endif