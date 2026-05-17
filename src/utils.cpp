#include "utils.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define WGS84_A 6378137.0f
#define WGS84_F 1.0f / 298.257223563f
#define WGS84_E2 (WGS84_F * (2.0f - WGS84_F))


char* next_field(char** p) {
    if (*p == NULL) return NULL;

    char* start = *p;
    char* comma = strchr(start, ',');

    if (comma) {
        *comma = '\0';
        *p = comma + 1;
    } else {
        *p = NULL;
    }

    return start;
}

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
char* trim(char* string) {
    char* ptr = NULL;
    while (*string == ' ') string++;    // chomp away space at the start
    ptr = string + strlen(string) - 1;  // jump to the last char (-1 because '\0')
    while (*ptr == ' ') {
        *ptr = '\0';
        ptr--;
    };  // overwrite with end of string
    return string;  // return pointer to the modified start
}
uint8_t checkChecksum(char* nmea_line) {
    if ((nmea_line[0] & '$') != '$' && nmea_line[0] != '!') {
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

uint32_t hexStr2Int(char str[]) { return (uint32_t)strtoul(str, 0, 16); }
float deg2rad(float d) {
    return d * M_PI / 180.0f;
}
float nmea_to_deg(float nmea) {
    int deg = (int)(nmea / 100);
    float min = nmea - (deg * 100);
    return deg + (min / 60.0f);
}
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

void ecef_to_llh(float x, float y, float z,
                 float* lat, float* lon, float* alt) {
    float lon_r = atan2f(y, x);

    float p = sqrtf(x * x + y * y);

    float lat_r = atan2f(z, p * (1.0f - WGS84_E2));

    float prev_lat;

    // iterative refinement (2–3 iterations is enough)
    for (int i = 0; i < 3; i++) {
        float sin_lat = sinf(lat_r);
        float N = WGS84_A / sqrtf(1.0f - WGS84_E2 * sin_lat * sin_lat);

        prev_lat = lat_r;
        lat_r = atan2f(z + WGS84_E2 * N * sin_lat, p);

        if (fabsf(lat_r - prev_lat) < 1e-10f)
            break;
    }

    float sin_lat = sinf(lat_r);
    float N = WGS84_A / sqrtf(1.0f - WGS84_E2 * sin_lat * sin_lat);

    *alt = p / cosf(lat_r) - N;
    *lat = lat_r * 180.0f / M_PI;
    *lon = lon_r * 180.0f / M_PI;
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