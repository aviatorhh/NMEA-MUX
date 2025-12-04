#include "utils.h"

#include <string.h>
#include <stdlib.h>

/**
 *
 */
uint8_t calcChecksum(const char* sentence) {
    // We calculate between $ and *
    uint8_t xsum = 0;
    const char* ptr = strchr(sentence, '*');
    if (ptr) {
        int16_t i = ptr - sentence - 1;
        for (; i > 0; i--) {
            xsum ^= sentence[i];
        }
    }
    return xsum;
}

uint32_t hexStr2Int(char str[]) { return (uint32_t)strtoul(str, 0, 16); }

uint8_t checkChecksum(const char* nmea_line) {
    if (nmea_line[0] != '$' && nmea_line[0] != '!') {
        return 0;
    }

    uint8_t cs = calcChecksum(nmea_line);

    // Get the checksum characters
    char t[3];
    const char* ptr = strchr(nmea_line, '*');
    if (ptr) {
        int16_t i = ptr - nmea_line;
        t[0] = nmea_line[i + 1];
        t[1] = nmea_line[i + 2];
        t[2] = '\0';
        if (cs != hexStr2Int(t)) {
            return 0;
        }
    } else {
        return 0;
    }
    return 1;
}

#ifdef WEB_GUI

void str_replace(char* src, char* oldchars, char* newchars) {  // utility string function
    char* p = strstr(src, oldchars);
    char buf[HTML_BUFFER_SIZE];
    do {
        if (p) {
            memset(buf, '\0', strlen(buf));
            if (src == p) {
                strcpy(buf, newchars);
                strcat(buf, p + strlen(oldchars));
            } else {
                strncpy(buf, src, strlen(src) - strlen(p));
                strcat(buf, newchars);
                strcat(buf, p + strlen(oldchars));
            }
            memset(src, '\0', strlen(src));
            strcpy(src, buf);
        }
    } while (p && (p = strstr(src, oldchars)));
}

#endif