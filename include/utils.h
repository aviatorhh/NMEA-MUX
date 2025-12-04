#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

#define HTML_BUFFER_SIZE (256 + 128)

uint32_t hexStr2Int(char str[]);
uint8_t checkChecksum(const char* nmea_line);
uint8_t calcChecksum(const char* sentence);

#ifdef WEB_GUI
void str_replace(char* src, char* oldchars, char* newchars);
#endif

#endif