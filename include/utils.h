#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

#define HTML_BUFFER_SIZE (256 + 128)
char* trim(char* string);
char* next_field(char** p);
uint32_t hexStr2Int(char str[]);
uint8_t checkChecksum(const char* nmea_line);
uint8_t calcChecksum(const char* sentence);
uint8_t checkChecksum(char* nmea_line);
float deg2rad(float d);
float nmea_to_deg(float nmea);
#ifdef WEB_GUI
void str_replace(char* src, char* oldchars, char* newchars);
#endif

#endif