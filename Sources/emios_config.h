/*
 * emios_config.h
 *
 *  Created on: Jun 23, 2015
 *      Author: luaviskang
 */

#ifndef EMIOS_CONFIG_H_
#define EMIOS_CONFIG_H_

#define EMIOS_0_SERVO_MOTOR 5 // PB13 PCR : 0x0601

#define EMIOS_0_LEFT_MOTOR 6 // PB14 PCR : 0x0601
#define EMIOS_0_RIGHT_MOTOR 7 // PB15 PCR : 0x0601


#define EMIOS_0_CLOCK_MAX 0x0000ffff	  // 65536
#define EMIOS_0_CLOCK_COUNTER 24

#define EMIOS_1_LEFT_ENCODER 24 // PG7 PCR : 0x0501
#define EMIOS_1_LEFT_ENCODER_MAX 0x0000ffff // 65536

#define EMIOS_1_RIGHT_ENCODER 16 // PG1 PCR : 0x0501
#define EMIOS_1_RIGHT_ENCODER_MAX 0x0000ffff // 65536

#define EMIOS_0_SONA_SENSOR 1

#define EMIOS_0_SLOPE_SONA_SENSOR 1

#endif /* EMIOS_CONFIG_H_ */
