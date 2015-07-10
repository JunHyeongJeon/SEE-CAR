/*
 * core_ai.h
 *
 *  Created on: Jul 7, 2015
 *      Author: luaviskang
 */

#ifndef CORE_AI_H_
#define CORE_AI_H_

typedef enum {
	
	DangerLevelSafeMode = 0,

	DangerLevelCaution,

	DangerLevelRescue,
} DangerLevel;

#define COUNT_NOT_FOUND_LIMIT 500

static DangerLevel danger_level = DangerLevelSafeMode;

void core_ai_think();

#endif /* CORE_AI_H_ */
