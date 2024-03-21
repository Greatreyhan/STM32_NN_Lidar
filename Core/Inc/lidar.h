/*
Author	: Alfonsus Giovanni
Date		: 10 November 2023
*/

#ifndef YDLIDAR_H_
#define YDLIDAR_H_

#include "main.h"
#include "stdbool.h"
#include "math.h"

/*YDLIDAR HEADER & ADDRESS*/
#define HEADER 0xA5
#define DEV_ADDR 0x00

/*YDLIDAR COMMMAD REGISTER*/
#define START_IAP 0x0A
#define RUNNING_IAP 0x0B
#define COMPLETE_IAP 0x0C
#define ACK_IAP 0x20
#define RST_SYS 0x67

#define GET_DEV_ADDR 0x60
#define GET_DEV_PARAM 0x61
#define GET_DEV_VER 0x62
#define DEV_START_SCAN 0x63
#define DEV_STOP_SCAN 0x64
#define DEV_SOFT_RESTART 0x67
#define SET_DEV_BAUDRATE 0x68
#define SET_DEV_EDGE_MODE 0x69

/*YDLIDAR STATUS*/
#define LIDAR_OK 0x1
#define LIDAR_ERR 0x2

/*YDLIDAR SCAN MODE*/
#define SCAN_ALL  0x01
#define	RIGHT_ONLY 0x02
#define	LEFT_ONLY 0x03
#define	FRONT_ONLY 0x04
#define LEFT_RIGHT 0x05

typedef struct{
	uint8_t
	status,
	dev_address;

	uint16_t
	K0, B0, K1, B1;

	int8_t
	get_bias;

	float
	distance_point[160],
	R_distance[80], L_distance[80],
	intensity_point[160],
	R_intensity[80], L_intensity[80],
	M_intensity[10], M_distance[10],
	env,
	right_sum, right_val,
	left_sum, left_val,
	front_sum, front_val;

	float
	d_compensateK0, d_compensateB0,
	d_compensateK1, d_compensateB1,
	bias;

	UART_HandleTypeDef* huart;
}Lidar_t;

void lidar_init(Lidar_t* lidar, UART_HandleTypeDef *uart_handler);

uint8_t lidar_get_dev_addr(Lidar_t *lidar);
uint8_t lidar_get_dev_param(Lidar_t *lidar);
uint8_t lidar_dev_scan(Lidar_t *lidar, int8_t scan_mode, int8_t data_qty, int8_t data_shift);
uint8_t lidar_dev_stop_scan(Lidar_t *lidar);

#endif
