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

#define COUNT_NOT_FOUND_LIMIT 10

static long int kp = 8010;//3000;
static long int kd = 272;//60;
static long int ki = 911;//51;

static DangerLevel danger_level = DangerLevelSafeMode;

void core_ai_think();

int get_ref_speed();

void set_ref_speed(int speed);

int get_kp();
int get_kd();
int get_ki();

void set_kp(int i);
void set_kd(int i);
void set_ki(int i);

#endif /* CORE_AI_H_ */
