/*
 * iic.h
 *
 *  Created on: Jul 1, 2025
 *      Author: carlos
 */

#ifndef INC_COMMUNICATION_PROTOCOLS_H_
#define INC_COMMUNICATION_PROTOCOLS_H_

#define HMC5883L_READ_ADDRESS 0x3D
#define HMC5883L_WRITE_ADDRESS 0x3C
#define HMC5883L_REG_WRITE_ADDRESS 0x02
#define HMC5883L_REG_READ_ADDRESS 0x03
#define I2C_TASK_DELAY 200

#define DATA_QUEUE_SIZE 5

#define SENSOR_MAG_READY    (1 << 2)
#define SENSOR_GPS_READY	(1 << 4)
#define SENSOR_ALL_READY	(1 << 8)

#define UART_RX_BUFFER_SIZE 1
#define UART_TASK_DELAY 200
#define UART_RX_DELAY_TIME 10
#define UART_BREAK_LINE '\n'
#define UART_LINE_FINISH '\0'

typedef struct {
	TickType_t timestamp;
	int16_t x, y, z;
} MagnetometerData;

typedef struct {
	TickType_t timestamp;
	int16_t x, y, z, id;
	uint8_t anchors;
} GPSData;

typedef struct {
	TickType_t timestamp;
	int16_t x, y, z, roll, pitch, yaw;
} SensorData;

/*
* Task para enviar o pedido de Write para o IMU
*/
void vI2C_IMU_Send_Request_Task(void *pvParameters);
/*
 * Task para processar os dados recebidos pelo IMU via I2C
 */
void vI2C_IMU_Management_Task(void *pvParameters);
/*
 * Task para lidar com a interrupcao que o IMU gera
 */
void vI2C_IMU_Handle_Interrupt();

void vUART_Management_Task(void *pvParameters);

void vSensor_Event_Handler_Task(void *pvParameters);

#endif /* INC_COMMUNICATION_PROTOCOLS_H_ */
