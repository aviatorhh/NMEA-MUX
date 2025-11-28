#include "utils.h"

#include <string.h>

/**
 *
 */
uint8_t calcChecksum(char* sentence) {
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