#ifndef UTIL_H
#define UTIL_H

// avoid manually switching between abs and fabs
template <typename T>
T myfabs(T v) {
    return (v < 0) ? -v : v;
}

#endif // UTIL_H
