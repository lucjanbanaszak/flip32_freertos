/*
 * drv_msp.c
 *
 *  Created on: 1 kwi 2016
 *      Author: lucja
 */

#include "platform_cfg.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "drv_usart.h"
#include "drv_mpu.h"
#include "drv_msp.h"
#include "imu.h"

void msp_task(void const * argument);

mspPort_t currentPort;
QueueHandle_t xMSPFramesQueue;

HAL_StatusTypeDef mspInit( UART_HandleTypeDef *huart )
{
	currentPort.port = huart;
	xMSPFramesQueue = xQueueCreate( 4, sizeof(QueueHandle_t) );

	osThreadDef( MSPTask, msp_task, osPriorityNormal, 0, 128);
	osThreadCreate(osThread( MSPTask ), NULL);

	return HAL_OK;
};

void MSP_RXCallback( uint8_t c )
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	static mspFrame_t mspFrame = { 0, 0, { 0,0,0,0,0,0,0,0 }};
	static uint8_t msp_rx_state = 0;
	static uint8_t msp_rx_crc = 0;
	static uint8_t msp_rx_payload_n = 0;

	switch( msp_rx_state ){
		case 0:
			msp_rx_crc = 0;
			msp_rx_payload_n = 0;
			if( c == '$' ) msp_rx_state = 1;
			break;
		case 1:
			if( c == 'M' ) msp_rx_state = 2;
			else		   msp_rx_state = 0;
			break;
		case 2:
			if( c == '<' ) msp_rx_state = 3;
			else		   msp_rx_state = 0;
			break;
		case 3:
			PIN_Set( PIN_PWMI5 );
			mspFrame.size = c;
			msp_rx_crc ^= c;
			msp_rx_state = 4;
			break;
		case 4:
			mspFrame.type = c;
			msp_rx_crc ^= c;
			if( mspFrame.size > 0 ) msp_rx_state = 5;
			else					msp_rx_state = 6;
			break;
		case 5:
			mspFrame.payload[msp_rx_payload_n] = c;
			msp_rx_crc ^= c;
			msp_rx_payload_n++;
			if( msp_rx_payload_n == mspFrame.size ) msp_rx_state = 6;
			break;
		case 6:
			if( msp_rx_crc == c ){
				PIN_Clear( PIN_PWMI5 );
				xQueueSendFromISR( xMSPFramesQueue, (void*)&mspFrame, &xHigherPriorityTaskWoken );
			}
			msp_rx_state = 0;
			break;
		default:
			msp_rx_state = 0;
			break;
	};

	/* Switch tasks if necessary. */
	if( xHigherPriorityTaskWoken != pdFALSE ) portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
};

static void serialize8(uint8_t a)
{
	HAL_UART_Transmit( currentPort.port, (uint8_t *)&a, 1, 0xFFFF);
    currentPort.checksum ^= a;
};

static void serialize16(uint16_t a)
{
    serialize8((uint8_t)(a >> 0));
    serialize8((uint8_t)(a >> 8));
};

static void serialize32(uint32_t a)
{
    serialize16((uint16_t)(a >> 0));
    serialize16((uint16_t)(a >> 16));
};

static void serializefloat(float a)
{
	uint32_t ia = *((uint32_t*)&a);
	serialize16((uint16_t)(ia >> 0));
	serialize16((uint16_t)(ia >> 16));
};

static void serializeNames(const char *s)
{
    const char *c;
    for (c = s; *c; c++)
        serialize8(*c);
};

static void headSerialResponse(uint8_t err, uint8_t responseBodySize)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    currentPort.checksum = 0;               // start calculating a new checksum
    serialize8(responseBodySize);
    serialize8(currentPort.cmdMSP);
};

static void headSerialReply(uint8_t responseBodySize)
{
    headSerialResponse(0, responseBodySize);
};

static void tailSerialReply(void)
{
    serialize8(currentPort.checksum);
}

extern float angleG[4];

void msp_task(void const * argument)
{
    static mspFrame_t mspFrame;

    for(;;){

		xQueueReceive( xMSPFramesQueue, &mspFrame, portMAX_DELAY );
		LEDToggle( PIN_LED1 );
		PIN_Set( PIN_PWMI6 );

		currentPort.cmdMSP = mspFrame.type;

		switch( mspFrame.type ){

		case MSP_API_VERSION:
			headSerialReply(
				1 + // protocol version length
				API_VERSION_LENGTH
			);
			serialize8(MSP_PROTOCOL_VERSION);

			serialize8(API_VERSION_MAJOR);
			serialize8(API_VERSION_MINOR);
			break;

		case MSP_FC_VARIANT:
			headSerialReply(FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);

			for( uint8_t i = 0; i < FLIGHT_CONTROLLER_IDENTIFIER_LENGTH; i++) {
				serialize8(i);
			}
			break;

		case MSP_FC_VERSION:
			headSerialReply(FLIGHT_CONTROLLER_VERSION_LENGTH);

			serialize8(1);
			serialize8(1);
			serialize8(1);
			break;

		case MSP_BOARD_INFO:
			headSerialReply(
				BOARD_IDENTIFIER_LENGTH +
				BOARD_HARDWARE_REVISION_LENGTH
			);
			for( uint8_t i = 0; i < BOARD_IDENTIFIER_LENGTH; i++) {
				serialize8(123);
			}
	#ifdef NAZE
			serialize16(hardwareRevision);
	#else
			serialize16(0); // No other build targets currently have hardware revision detection.
	#endif
			break;

		// DEPRECATED - Use MSP_API_VERSION
		case MSP_IDENT:
			headSerialReply(7);
			serialize8(1);
			serialize8(1);
			serialize8(MSP_PROTOCOL_VERSION);
			serialize32(CAP_DYNBALANCE); // "capability"
			break;

		 case MSP_RAW_IMU:
			headSerialReply(16);

			// Hack scale due to choice of units for sensor data in multiwii
			uint8_t scale = (acc_1G > 1024) ? 8 : 1;

			for( uint8_t i = 0; i < 4; i++) serializefloat(angleG[i]);
			//for( uint8_t i = 0; i < 3; i++) serialize16(gyroADC[i]);
			//for( uint8_t i = 0; i < 3; i++) serialize16(angleG[i]);
			break;

		case MSP_ATTITUDE:
			headSerialReply(6);
			serialize16( angle[ROLL] );
			serialize16( angle[PITCH] );
			serialize16(DECIDEGREES_TO_DEGREES( 0 ));
			break;

		case MSP_ALTITUDE:
			headSerialReply(6);
	#if defined(BARO) || defined(SONAR)
			serialize32(altitudeHoldGetEstimatedAltitude());
	#else
			serialize32(0);
	#endif
			serialize16(0);
			break;

		case MSP_SONAR_ALTITUDE:
			headSerialReply(4);
	#if defined(SONAR)
			serialize32(sonarGetLatestAltitude());
	#else
			serialize32(0);
	#endif
			break;

		case MSP_LOOP_TIME:
			headSerialReply(2);
			serialize16( 10 );
			break;

		case MSP_PIDNAMES:
			headSerialReply(4);
			serializeNames("test");
			break;

		case MSP_PID_CONTROLLER:
			headSerialReply(1);
			serialize8(0);
			break;

		case MSP_UID:
			headSerialReply(12);
			serialize32(U_ID_0);
			serialize32(U_ID_1);
			serialize32(U_ID_2);
			break;

		case MSP_FEATURE:
			headSerialReply(4);
			serialize32(0xaa55ff00);
			break;

		default:
			break;
		};

		tailSerialReply();
		PIN_Clear( PIN_PWMI6 );
    };
};
