#include "dgps.h"

#define DGPS_TABLE_SIZE (sizeof(dgps_table) / sizeof(dgps_table[0]))

float spoof_score = 0.0;

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

void sp_init(spoof_score_t* sp) {
    sp->count = 0;
    sp->total = 0.0f;
    sp->max = 0.0f;
}

void sp_add(spoof_score_t* sp, const char* name, float value, float weight) {
    if (sp->count >= MAX_SIGNALS) return;

    if (value < 0) value = 0;
    if (value > 1) value = 1;

    spoof_signal_t* sig = &sp->s[sp->count++];
    sig->name = name;
    sig->value = value;
    sig->weight = weight;
    sig->contribution = value * weight;

    sp->total += sig->contribution;
    sp->max += weight;
}

float sp_final(spoof_score_t* sp) {
    if (sp->max < 0.0001f) return 0.0f;
    return sp->total / sp->max;
}
