/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "platform_cfg.h"
#include "cmsis_os.h"
#include "math.h"
#include "printf.h"
#include "drv_pwm.h"
#include "drv_mpu.h"
#include "drv_baro.h"
#include "drv_mag.h"
#include "sensors.h"
#include "imu.h"

void baro_update_task(void const * argument);
void mpu_update_task(void const * argument);
void mag_update_task(void const * argument);

uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 0;      // baro calibration = get new ground pressure value
uint16_t calibratingG = 0;
int16_t heading, magHold;

sensor_t acc;                       // acc access functions
sensor_t gyro;                      // gyro access functions
sensor_t mag;                       // mag access functions
baro_t baro;                        // barometer access functions

#define CALIBRATING_GYRO_CYCLES             1000
#define CALIBRATING_ACC_CYCLES              400
#define CALIBRATING_BARO_CYCLES             200

int32_t baroPressure = 0;
int32_t baroTemperature = 0;
extern uint32_t baroPressureSum;
extern int16_t gyroADC[3], accADC[3];

int16_t magData[3];
int16_t accZero[3];
int16_t gyroZero[3];

HAL_StatusTypeDef sensorsAutodetect(void)
{
	if( mpuDetect( &acc, &gyro ) != HAL_OK ) return HAL_ERROR;
	if( baroDetect( &baro ) != HAL_OK ) return HAL_ERROR;
	if( magDetect( &mag ) != HAL_OK ) return HAL_ERROR;

	osThreadDef(MpuTask, mpu_update_task, osPriorityNormal, 0, 128);
	osThreadCreate(osThread(MpuTask), NULL);

	osThreadDef(BaroTask, baro_update_task, osPriorityNormal, 0, 128);
	osThreadCreate(osThread(BaroTask), NULL);

	osThreadDef(MagTask, mag_update_task, osPriorityNormal, 0, 128);
	osThreadCreate(osThread(MagTask), NULL);

	return HAL_OK;
}

#define Kp	8000.0f
#define Ki	100.0f
#define Kd	10000.00f

extern TIM_HandleTypeDef Timer1Handle;
extern TIM_HandleTypeDef Timer4Handle;

extern __IO int32_t pwmi1_val;
extern __IO int32_t pwmi2_val;
extern __IO int32_t pwmi3_val;
extern __IO int32_t pwmi4_val;

extern int16_t angle[2];
extern float anglerad[2];

#define LOOP_FREQ	10

void mpu_update_task(void const * argument)
{
//	bool Output_enable = true;
	const TickType_t xFrequency = LOOP_FREQ;
	TickType_t xLastWakeTime = xTaskGetTickCount();

//	float Error = 0;
//	float _Error = 0;
//	float ITerm = 0;
//	float DTerm = 0;

	for(;;)
	{
	  xSemaphoreTake( MPU_DataReady_sem, portMAX_DELAY );
	  vTaskDelayUntil( &xLastWakeTime, xFrequency );

	  LEDToggle( PIN_LED0 );
	  PIN_Set( PIN_PWMI7 );

	  computeIMU();

//	  printf("\n\r%d.%d.%d_%d.%d.%d_%d.%d", gyroADC[0], gyroADC[1], gyroADC[2], accADC[0], accADC[1], accADC[2], angle[ROLL], angle[PITCH] );

//	  Error = anglerad[PITCH];
//
//	  if( (Error > 0.5f) || (Error < -0.5f) ) Output_enable = false;
//
////	  if( ITerm >  10.f ) ITerm = 10000.0f;
////	  if( ITerm < -10000.f ) ITerm = -10000.0f;
//
//	  ITerm = ITerm + Error;
//
//	  float Output = Kp*Error + Ki*ITerm + Kd*(Error -_Error);
//
//
//	  _Error =  Error;
//
//	  int32_t OutputL = (int32_t)Output/* + (pwmi1_val-1500)/5*/;
//	  int32_t OutputR = (int32_t)Output/* - (pwmi1_val-1500)/5*/;
//
//	  if( OutputL >  800 ) OutputL = 800;
//	  if( OutputL < -800 ) OutputL = -800;
//
//	  if( OutputR >  800 ) OutputR = 800;
//	  if( OutputR < -800 ) OutputR = -800;
//
//	  if( Output_enable ){
//		  if( OutputL > 0 )
//		  {
//			  PIN_Clear( PIN_PWMO5 );
//			  Timer4Handle.Instance->CCR4 = OutputL;
//		  }else
//		  {
//			  PIN_Set( PIN_PWMO5 );
//			  Timer4Handle.Instance->CCR4 = OutputL+1000;
//		  };
//
//		  if( OutputR > 0 )
//		  {
//			  PIN_Clear( PIN_PWMO3 );
//			  Timer4Handle.Instance->CCR2 = OutputR;
//
//		  }else
//		  {
//			  PIN_Set( PIN_PWMO3 );
//			  Timer4Handle.Instance->CCR2 = OutputR+1000;
//
//		  };
//	  }else{
//		  PIN_Clear( PIN_PWMO5 );
//		  PIN_Clear( PIN_PWMO3 );
//		  Timer4Handle.Instance->CCR2 = 0;
//		  Timer4Handle.Instance->CCR4 = 0;
//	  }

	  PIN_Clear( PIN_PWMI7 );
	};
};

void mag_update_task(void const * argument)
{
	const TickType_t xFrequency = 100;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
	  xSemaphoreTake( MAG_DataReady_sem, portMAX_DELAY );
	  vTaskDelayUntil( &xLastWakeTime, xFrequency );

	  mag.read( magData );
	};
};

void ACC_getADC(void)
{
    acc.read(accADC);
}

#define BARO_TAB_SIZE_MAX   48

void Baro_Common(void)
{
    static int32_t baroHistTab[BARO_TAB_SIZE_MAX];
    static int baroHistIdx;
    int indexplus1;

    indexplus1 = (baroHistIdx + 1);
    if (indexplus1 == BARO_TAB_SIZE_MAX)
        indexplus1 = 0;
    baroHistTab[baroHistIdx] = baroPressure;
    baroPressureSum += baroHistTab[baroHistIdx];
    baroPressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;
}

void baro_update_task(void const * argument)
{
	const TickType_t xFrequency = 20;
	TickType_t xLastWakeTime = xTaskGetTickCount();

	uint8_t state = 0;

	for(;;)
	{
	  vTaskDelayUntil( &xLastWakeTime, xFrequency );

	  switch( state ){
	  	  case 0:
	  		  baro.start_ut();
	  		  state = 1;
	  		  break;
	  	  case 1:
	  		 baro.get_ut();
	  		 baro.start_up();
	  		 state = 2;
	  		 break;
	  	  case 2:
	  		 baro.get_up();
	  		 baro.start_ut();
	  		 baro.calculate(&baroPressure, &baroTemperature);
	  		 state = 1;
	  		 break;
	  	};
	};
};

typedef struct stdev_t {
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

static void devClear(stdev_t *dev)
{
    dev->m_n = 0;
}

static void devPush(stdev_t *dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    } else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

void Gyro_getADC(void)
{
    // range: +/- 8192; +/- 2000 deg/sec
    gyro.read(gyroADC);
}

#ifdef MAG
static uint8_t magInit = 0;

void Mag_init(void)
{
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    LED1_ON;
    mag.init(mcfg.mag_align);
    LED1_OFF;
    magInit = 1;
}

int Mag_getADC(void)
{
    static uint32_t t, tCal = 0;
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    uint32_t axis;

    if ((int32_t)(currentTime - t) < 0)
        return 0;                 //each read is spaced by 100ms
    t = currentTime + 100000;

    // Read mag sensor
    mag.read(magADC);

    if (f.CALIBRATE_MAG) {
        tCal = t;
        for (axis = 0; axis < 3; axis++) {
            mcfg.magZero[axis] = 0;
            magZeroTempMin[axis] = magADC[axis];
            magZeroTempMax[axis] = magADC[axis];
        }
        f.CALIBRATE_MAG = 0;
    }

    if (magInit) {              // we apply offset only once mag calibration is done
        magADC[X] -= mcfg.magZero[X];
        magADC[Y] -= mcfg.magZero[Y];
        magADC[Z] -= mcfg.magZero[Z];
    }

    if (tCal != 0) {
        if ((t - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            LED0_TOGGLE;
            for (axis = 0; axis < 3; axis++) {
                if (magADC[axis] < magZeroTempMin[axis])
                    magZeroTempMin[axis] = magADC[axis];
                if (magADC[axis] > magZeroTempMax[axis])
                    magZeroTempMax[axis] = magADC[axis];
            }
        } else {
            tCal = 0;
            for (axis = 0; axis < 3; axis++)
                mcfg.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2; // Calculate offsets
            writeEEPROM(1, true);
        }
    }

    return 1;
}
#endif

#ifdef SONAR

void Sonar_init(void)
{
    hcsr04_init(sonar_rc78);
    sensorsSet(SENSOR_SONAR);
    sonarAlt = 0;
}

void Sonar_update(void)
{
    hcsr04_get_distance(&sonarAlt);
}

#endif
