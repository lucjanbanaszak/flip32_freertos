#include "platform_cfg.h"
#include "sensors.h"
#include "math.h"
#include "imu.h"

#include "MahonyAHRS.h"
#include "imu_util.h"

int16_t gyroADC[3], accADC[3], accSmooth[3], magADC[3];

// **************
// gyro+acc IMU
// **************
int16_t gyroData[3] = { 0, 0, 0 };
int16_t angle[2] = { 0, 0 };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
float anglerad[2] = { 0.0f, 0.0f };    // absolute angle inclination in radians

static void getEstimatedAttitude(void);

#define cfg_accxy_deadband				40
#define cfg_acc_lpf_factor				4
#define mcfg_gyro_cmpf_factor			50.0f	// default MWC
#define deltaT							5000 // 5000us
#define acc_lpf_factor					1.0f

void imuInit(void)
{
#ifdef MAG
	// if mag sensor is enabled, use it
	Mag_init();
#endif
}

float angleG[4] = {0,0,0,0};
float rollACC = 0.0f;
float pitchACC = 0.0f;
float rollGYR = 0.0f;
float pitchGYR = 0.0f;

#define filterG		0.99f

float pitch = 0.0f;
float roll = 0.0f;

// these must be defined somewhere else
float samplePeriod = 0.01f;
float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f };;

void computeIMU(void)
{
    Gyro_getADC();
    ACC_getADC();

    float gyro[3], acc[3];

    for( uint32_t axis = 0; axis < 3; axis++ ){
    	gyro[axis] = 2000.0f*(float)gyroADC[axis]*0.0000305f*0.01f;
    	acc[axis]  = 8.0f*(float)accADC[axis]*0.0000305f;
    };

    imuDegToRadV3(gyro);

    MahonyAHRSupdateIMU( gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2] );

    imuQuaternionToYawPitchRoll( quaternion, angleG );
    imuRadToDegV3(angleG);
}

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

t_fp_vector EstG;

// Normalize a vector
void normalizeV(struct fp_vector *src, struct fp_vector *dest)
{
    float length;

    length = sqrtf(src->X * src->X + src->Y * src->Y + src->Z * src->Z);
    if (length != 0) {
        dest->X = src->X / length;
        dest->Y = src->Y / length;
        dest->Z = src->Z / length;
    }
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;

    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta[ROLL]);
    sinx = sinf(delta[ROLL]);
    cosy = cosf(delta[PITCH]);
    siny = sinf(delta[PITCH]);
    cosz = cosf(delta[YAW]);
    sinz = sinf(delta[YAW]);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}

void getEstimatedAttitude(void)
{
    static float accLPF[3];
    float scale, deltaGyroAngle[3];
    scale = deltaT * gyro.scale;

    int32_t accMag = 0;

    // Initialization
    for( uint32_t axis = 0; axis < 3; axis++ ){
        deltaGyroAngle[axis] = gyroADC[axis] * scale;
        accLPF[axis] += accADC[axis] - accLPF[axis]/acc_lpf_factor;
        accSmooth[axis] = accLPF[axis]/acc_lpf_factor;
        accMag += (int32_t)accSmooth[axis] * accSmooth[axis];
    };

    accMag = 100*accMag/ ((int32_t)acc_1G * acc_1G);

    rotateV(&EstG.V, deltaGyroAngle);

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if( ((uint16_t)accMag > 72) && ((uint16_t)accMag < 133) ){
        for( uint32_t axis = 0; axis < 3; axis++ )
            EstG.A[axis] = (EstG.A[axis]*mcfg_gyro_cmpf_factor + accSmooth[axis])/(mcfg_gyro_cmpf_factor+1.0f);
    }

    // Attitude of the estimated vector
    anglerad[ROLL] = atan2f(EstG.V.Y, EstG.V.Z);
    anglerad[PITCH] = atan2f(-EstG.V.X, sqrtf(EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z));
    angle[ROLL] = lrintf(anglerad[ROLL] * (1800.0f / M_PI));
    angle[PITCH] = lrintf(anglerad[PITCH] * (1800.0f / M_PI));
};
