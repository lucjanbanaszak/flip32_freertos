/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "platform_cfg.h"
#include "task.h"
#include "drv_i2c.h"
#include "printf.h"
#include "math.h"

#include "drv_mpu.h"
#include "drv_mag.h"

// This is generally where all Invensense devices are at, for default (AD0 down) I2C address
#define MPU_ADDRESS                         (0b11010000)

#define MPU_RA_WHO_AM_I                     (0x75)
#define MPU_RA_GYRO_XOUT_H                  (0x43)
#define MPU_RA_ACCEL_XOUT_H                 (0x3B)

// For debugging/identification purposes
#define MPU_RA_XA_OFFS_H                    (0x06)    //[15:0] XA_OFFS
#define MPU_RA_PRODUCT_ID                   (0x0C)    // Product ID Register

#define MPUx0x0_WHO_AM_I_CONST              (0x68)

enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};

enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,
    INV_FSR_1000DPS,
    INV_FSR_2000DPS,
    NUM_GYRO_FSR
};

enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL,
    NUM_CLK
};

enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

typedef void (*baroOpFuncPtr)(void);                       // baro start operation
typedef void (*baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature);             // baro calculation (filled params are pressure and temperature)
typedef void (*serialReceiveCallbackPtr)(uint16_t data);   // used by serial drivers to return frames to app
typedef uint16_t (*rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data
typedef void (*pidControllerFuncPtr)(void);                // pid controller function prototype

typedef HAL_StatusTypeDef (*mpuReadRegPtr)(uint8_t reg, uint8_t *data, int length);
typedef HAL_StatusTypeDef (*mpuWriteRegPtr)(uint8_t reg, uint8_t data);
typedef void (*mpuInitPtr)(sensor_t *acc, sensor_t *gyro);

typedef struct mpu_access_t {
    mpuReadRegPtr read;
    mpuWriteRegPtr write;
    mpuInitPtr init;

    uint8_t acc_xout;
    uint8_t gyro_xout;
} mpu_access_t;

// Hardware access functions
static HAL_StatusTypeDef mpuReadRegisterI2C(uint8_t reg, uint8_t *data, int length);
static HAL_StatusTypeDef mpuWriteRegisterI2C(uint8_t reg, uint8_t data);
static void mpu6050Init(sensor_t *acc, sensor_t *gyro);

// Hardware access funcptrs

// General forward declarations
#ifdef PROD_DEBUG
static void mpu6050SelfTest(void);
#endif

static void mpuAccRead(int16_t *accData);
static void mpuGyroRead(int16_t *gyroData);

// Needed for MPU6050 half-scale acc bug
uint16_t acc_1G;
// Hardware access function
static mpu_access_t mpu;
// Lowpass
static uint8_t mpuLowPassFilter = INV_FILTER_42HZ;

HAL_StatusTypeDef mpuDetect(sensor_t *acc, sensor_t *gyro )
{
	// Set acc_1G. Modified once by mpu6050CheckRevision for old (hopefully nonexistent outside of clones) parts
	acc_1G = 512 * 8;

	printf("\n\r> MPU detecting: ");

	// Try I2C access first
	mpu.read = mpuReadRegisterI2C;
	mpu.write = mpuWriteRegisterI2C;
	// Default gyro/acc read offsets (overwritten only by legacy MPU3xxx hardware)
	mpu.gyro_xout = MPU_RA_GYRO_XOUT_H;
	mpu.acc_xout = MPU_RA_ACCEL_XOUT_H;

	uint8_t sig = 0;
	mpu.read(MPU_RA_WHO_AM_I, &sig, 1);

	if( (sig&0x7E) != MPUx0x0_WHO_AM_I_CONST ){ printf("ERROR"); return HAL_ERROR; };

	printf("MPU6050 ");

    uint8_t tmp[6] = {0,0,0,0,0,0 };

	mpu.read(MPU_RA_XA_OFFS_H, tmp, 6);
	uint8_t rev = ((tmp[5] & 0x01) << 2) | ((tmp[3] & 0x01) << 1) | (tmp[1] & 0x01);

	if(rev){
		// Congrats, these parts are better
		if( rev == 1 ) acc_1G /= 2;
		else if(rev != 2) return HAL_ERROR;

	}else{
		mpu.read(MPU_RA_PRODUCT_ID, &rev, 1);
		rev &= 0x0F;
		if( !rev ){ printf("ERROR"); return HAL_ERROR; }
		else if( rev == 4 ) acc_1G /= 2;
	};

	printf("rev.%d", rev );

	mpu.init = mpu6050Init;

	// 16.4 dps/lsb scalefactor for all Invensense devices
	gyro->scale = (4.0f / 16.4f) * (M_PI / 180.0f) * 0.000001f;

	// default lpf is 42Hz, 255 is special case of nolpf
	mpuLowPassFilter = INV_FILTER_42HZ;

	// initialize the device
	mpu.init(acc, gyro);

	return HAL_OK;
}

// MPU6xxx registers
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_SIGNAL_PATH_RST  0x68
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_FIFO_COUNT_H     0x72
#define MPU_RA_FIFO_R_W         0x74

// MPU6050 bits
#define MPU6050_INV_CLK_GYROZ   0x03
#define MPU6050_BIT_FIFO_RST    0x04
#define MPU6050_BIT_DMP_RST     0x08
#define MPU6050_BIT_FIFO_EN     0x40

#define INV_X_GYRO              0x40
#define INV_Y_GYRO              0x20
#define INV_Z_GYRO              0x10
#define INV_XYZ_GYRO            (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL           0x08

#define MPU6050_MAX_PACKET_LEN  12

SemaphoreHandle_t MPU_DataReady_sem;

static void mpu6050Init(sensor_t *acc, sensor_t *gyro)
{
	printf("\n\r> MPU initialising: ");

	MPU_DataReady_sem = xSemaphoreCreateBinary();
	xSemaphoreTake( MPU_DataReady_sem, 0);

	MPU_INT_GPIO_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();

	HAL_GPIO_Init( PIN_MPU_INT.gpio, (GPIO_InitTypeDef*)&PIN_MPU_INT.cfg );

    // Device reset
    mpu.write(MPU_RA_PWR_MGMT_1, 0x80); // Device reset
    vTaskDelay(100);
    // Gyro config
    mpu.write(MPU_RA_SMPLRT_DIV, 0x00); // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    mpu.write(MPU_RA_PWR_MGMT_1, MPU6050_INV_CLK_GYROZ); // Clock source = 3 (PLL with Z Gyro reference)
    vTaskDelay(10);
    mpu.write(MPU_RA_CONFIG, mpuLowPassFilter); // set DLPF
    mpu.write(MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3); // full-scale 2kdps gyro range
    mpu.write(MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);     // Accel scale 8g (4096 LSB/g)

    // Data ready interrupt configuration
    mpu.write(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 1 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);  // INT_RD_CLEAR_DIS, I2C_BYPASS_EN
    mpu.write(MPU_RA_INT_ENABLE, 0x01); // DATA_RDY_EN interrupt enable

    acc->read = mpuAccRead;
    gyro->read = mpuGyroRead;

    /* Enable and set EXTI line 0 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(MPU_INT_EXTI_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+2, 0);
    HAL_NVIC_EnableIRQ(MPU_INT_EXTI_IRQn);

    printf("OK");
}

void MPU_INT_EXTI_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( MPU_DataReady_sem, &xHigherPriorityTaskWoken );

	if( xHigherPriorityTaskWoken != pdFALSE ) portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
};

void MPU_INT_EXTI_IRQHandler(void)
{
	if( __HAL_GPIO_EXTI_GET_IT( MPU_INT_PIN ) != RESET ){

		__HAL_GPIO_EXTI_CLEAR_IT( MPU_INT_PIN );
		MPU_INT_EXTI_Callback();

	}else if( __HAL_GPIO_EXTI_GET_IT( MAG_RDRY_PIN ) != RESET ){
		__HAL_GPIO_EXTI_CLEAR_IT( MAG_RDRY_PIN );
		MAG_RDRY_EXTI_Callback();

	}else{
		__HAL_GPIO_EXTI_CLEAR_IT( EXTI_PR_PR10 | EXTI_PR_PR11 | EXTI_PR_PR12 |EXTI_PR_PR13 | EXTI_PR_PR14 | EXTI_PR_PR15 );
	};
};

static void mpuAccRead(int16_t *accData)
{
    uint8_t buf[6];

    mpu.read(mpu.acc_xout, buf, 6);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
};

static void mpuGyroRead(int16_t *gyroData)
{
    uint8_t buf[6];

    mpu.read(mpu.gyro_xout, buf, 6);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

static HAL_StatusTypeDef mpuReadRegisterI2C(uint8_t reg, uint8_t *data, int length)
{
	return i2cRead(MPU_ADDRESS, reg, data, length);
};

static HAL_StatusTypeDef mpuWriteRegisterI2C(uint8_t reg, uint8_t data)
{
    return i2cWrite(MPU_ADDRESS, reg, data);
};
