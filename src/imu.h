/*
 * imu.h
 *
 *  Created on: 30 mar 2016
 *      Author: solaris
 */

#ifndef IMU_H_
#define IMU_H_

void imuInit(void);
void computeIMU(void);

extern int16_t gyroADC[3];
extern int16_t accADC[3];
extern int16_t accSmooth[3];
extern int16_t magADC[3];

extern int16_t gyroData[3];
extern int16_t angle[2];
extern float anglerad[2];

#endif /* IMU_H_ */
