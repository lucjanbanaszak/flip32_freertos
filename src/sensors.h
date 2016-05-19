/*
 * sensor.h
 *
 *  Created on: 30 mar 2016
 *      Author: solaris
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "drv_baro.h"

enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};

typedef enum {
    X = 0,
    Y,
    Z
} sensor_axis_e;

typedef void (*sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype
typedef void (*sensorInitFuncPtr)(void);          // sensor read and align prototype

typedef struct sensor_t {
	sensorInitFuncPtr init;
    sensorReadFuncPtr read;                                 // read 3 axis data function
    sensorReadFuncPtr temperature;                          // read temperature if available
    float scale;                                            // scalefactor (currently used for gyro only, todo for accel)
} sensor_t;

HAL_StatusTypeDef sensorsAutodetect(void);
void ACC_getADC(void);
void Gyro_getADC(void);

extern uint16_t calibratingA;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
extern uint16_t calibratingB;      // baro calibration = get new ground pressure value
extern uint16_t calibratingG;
extern uint16_t acc_1G;          // this is the 1G measured acceleration.
extern int16_t heading, magHold;

extern sensor_t acc;                       // acc access functions
extern sensor_t gyro;                      // gyro access functions
extern sensor_t mag;                       // mag access functions
extern baro_t baro;                        // barometer access functions

#endif /* SENSOR_H_ */
