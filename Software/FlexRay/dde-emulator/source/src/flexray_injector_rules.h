#ifndef FLEXRAY_INJECTOR_RULES_H
#define FLEXRAY_INJECTOR_RULES_H

#include <stdint.h>

#define INJECT_DIRECTION_TO_FR1 0
#define INJECT_DIRECTION_TO_FR2 1
#define INJECT_DIRECTION_TO_FR3 2
#define INJECT_DIRECTION_TO_FR4 3
typedef struct {
	uint16_t trigger_id;    // when this id arrives...
	uint16_t target_id;  // ...inject using cached template of this id (if available)
	uint8_t cycle_mask;
	uint8_t cycle_base;
	uint8_t e2e_offset;
	uint8_t e2e_len;
	uint8_t e2e_init_value;
	uint8_t replace_offset;
	uint8_t replace_len;
	uint8_t direction;
} trigger_rule_t;

static const trigger_rule_t INJECT_TRIGGERS[] = {
	// Pass-through build: no injection rules (table intentionally empty).
};

#define NUM_TRIGGER_RULES (sizeof(INJECT_TRIGGERS)/sizeof(INJECT_TRIGGERS[0]))

#endif // FLEXRAY_INJECTOR_RULES_H
