#include "dgps.h"
#define DGPS_TABLE_SIZE (sizeof(dgps_table) / sizeof(dgps_table[0]))
const char* dgps_lookup(uint16_t id) {
    int16_t low = 0;
    int16_t high = DGPS_TABLE_SIZE - 1;

    while (low <= high) {
        int16_t mid = (low + high) >> 1;

        uint16_t mid_id =
            pgm_read_word(&dgps_table[mid].id);

        if (id == mid_id) {
            return (const char*)
                pgm_read_ptr(&dgps_table[mid].name);
        } else if (id < mid_id) {
            high = mid - 1;
        } else {
            low = mid + 1;
        }
    }

    return s_unknown;
}