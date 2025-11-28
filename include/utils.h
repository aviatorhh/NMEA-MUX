#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

#define HTML_BUFFER_SIZE (256 + 128)

uint8_t calcChecksum(char* sentence);

#ifdef WEB_GUI
void str_replace(char* src, char* oldchars, char* newchars);
#endif

#endif