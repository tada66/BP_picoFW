#ifndef DEBUGPRINT_H
#define DEBUGPRINT_H
#define DEBUGENABLED 0

#include <stdio.h>

#if DEBUGENABLED
#define DEBUG_PRINT(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...)
#endif

#endif // DEBUGPRINT_H
