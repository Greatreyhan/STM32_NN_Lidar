/*
Author	: Alfonsus Giovanni
Date		: 10 November 2023
*/

#include "lidar.h"

#define UART_TIMEOUT 100

uint8_t rx_buff[20], scan_data[322];

void lidar_init(Lidar_t* lidar, UART_HandleTypeDef *uart_handler){
	lidar->huart = uart_handler;
}

uint8_t lidar_get_dev_addr(Lidar_t *lidar){
	uint8_t tx_data[] = {HEADER, HEADER, HEADER, HEADER, 0x00, GET_DEV_ADDR, 0x00, 0x00, 0x60};
	HAL_UART_Transmit(lidar->huart, tx_data, sizeof(tx_data), UART_TIMEOUT);

	HAL_UART_Receive(lidar->huart, rx_buff, sizeof(rx_buff), UART_TIMEOUT);

	if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[1] == HEADER && rx_buff[3] == HEADER){
		lidar->dev_address = rx_buff[6] || rx_buff[7];

		return lidar->status = LIDAR_OK;
	}

	else return lidar->status = LIDAR_ERR;
}

uint8_t lidar_get_dev_param(Lidar_t *lidar){
	uint8_t tx_data[] = {HEADER, HEADER, HEADER, HEADER, 0x00, GET_DEV_PARAM, 0x00, 0x00, 0x61};
	HAL_UART_Transmit(lidar->huart, tx_data, sizeof(tx_data), UART_TIMEOUT);

	HAL_UART_Receive(lidar->huart, rx_buff, sizeof(rx_buff), UART_TIMEOUT);

	if(rx_buff[0] == HEADER && rx_buff[1] == HEADER && rx_buff[1] == HEADER && rx_buff[3] == HEADER){
		lidar->K0 			= rx_buff[8] 	| rx_buff[9];
		lidar->B0 			= rx_buff[10] | rx_buff[11];
		lidar->K1 			= rx_buff[11] | rx_buff[12];
		lidar->B1 			= rx_buff[12] | rx_buff[13];
		lidar->get_bias = rx_buff[14];

		lidar->d_compensateK0 = (float)lidar->K0/10000.0f;
		lidar->d_compensateB0 = (float)lidar->B0/10000.0f;
		lidar->d_compensateK1 = (float)lidar->K1/10000.0f;
		lidar->d_compensateB1 = (float)lidar->B1/10000.0f;
		lidar->bias						= (float)lidar->get_bias/10;
	}
}

uint8_t lidar_dev_scan(Lidar_t *lidar, int8_t scan_mode, int8_t data_qty, int8_t data_shift){
	uint8_t tx_data[] = {HEADER, HEADER, HEADER, HEADER, 0x00, DEV_START_SCAN, 0x00, 0x00, 0x63};
	HAL_UART_Transmit(lidar->huart, tx_data, sizeof(tx_data), UART_TIMEOUT);

	HAL_UART_Receive(lidar->huart, scan_data, sizeof(scan_data), UART_TIMEOUT);

	if(scan_data[0] == HEADER && scan_data[1] == HEADER && scan_data[1] == HEADER && scan_data[3] == HEADER){
		lidar->env = scan_data[8] | scan_data[9];

		lidar->right_sum = 0;
		lidar->left_sum = 0;
		lidar->front_sum = 0;

		if(scan_mode == SCAN_ALL){
			for(int i=0; i<160; i++){
				lidar->distance_point[i] = (scan_data[11 + i*2] & 0x01) | scan_data[10 + i*2];
				lidar->intensity_point[i] = scan_data[11 + i*2] >> 1;
			}

			for(int i=0; i<79; i++){
				lidar->L_distance[i] = lidar->distance_point[79-i];
				lidar->L_intensity[i] = lidar->intensity_point[79-i];

				lidar->R_distance[i] = lidar->distance_point[80+i];
				lidar->R_intensity[i] = lidar->intensity_point[80+i];

				lidar->right_sum += lidar->R_distance[i];
				lidar->left_sum += lidar->L_distance[i];
			}

			lidar->right_val = lidar->right_sum / 80;
			lidar->left_val = lidar->left_sum / 80;
		}

		else if(scan_mode == RIGHT_ONLY){
			for(int i=80; i<160; i++){
				lidar->distance_point[i] = (scan_data[11 + i*2] & 0x01) | scan_data[10 + i*2];
				lidar->intensity_point[i] = scan_data[11 + i*2] >> 1;
			}

			for(int i=(80+data_shift); i<80+(data_shift+data_qty); i++){
				lidar->R_distance[i-80] = lidar->distance_point[i];
				lidar->R_intensity[i-80] = lidar->intensity_point[i];

				lidar->right_sum += lidar->R_distance[i-80];
			}
			lidar->right_val = lidar->right_sum/data_qty;
		}

		else if(scan_mode == LEFT_ONLY){
			for(int i=0; i<80; i++){
				lidar->distance_point[i] = (scan_data[11 + i*2] & 0x01) | scan_data[10 + i*2];
				lidar->intensity_point[i] = scan_data[11 + i*2] >> 1;
			}

			for(int i=(79-data_shift); i<(79-data_shift)+data_qty; i++){
				lidar->L_distance[i] = lidar->distance_point[79-i];
				lidar->L_intensity[i] = lidar->intensity_point[79-i];

				lidar->left_sum += lidar->L_distance[i];
			}
			lidar->left_val = lidar->left_sum / data_qty;
		}

		else if(scan_mode == FRONT_ONLY){
			for(int i=0; i<10; i++){
				lidar->distance_point[i] = (scan_data[11 + i*2] & 0x01) | scan_data[10 + i*2];
				lidar->intensity_point[i] = scan_data[11 + i*2] >> 1;
			}

			/*FRONT LEFT*/
			for(int i=0; i<data_shift+data_qty; i++){
				lidar->distance_point[i] = (scan_data[11 + i*2] & 0x01) | scan_data[10 + i*2];
				lidar->intensity_point[i] = scan_data[11 + i*2] >> 1;

				lidar->M_distance[i] = lidar->distance_point[i];
				lidar->M_intensity[i] = lidar->intensity_point[i];
				lidar->front_sum += lidar->M_distance[i];
			}
			lidar->front_val = lidar->front_sum/data_qty;
		}
	}
}

uint8_t lidar_dev_stop_scan(Lidar_t *lidar){
	uint8_t tx_data[] = {HEADER, HEADER, HEADER, HEADER, 0x00, DEV_STOP_SCAN, 0x00, 0x00, 0x64};
	HAL_UART_Transmit(lidar->huart, tx_data, sizeof(tx_data), UART_TIMEOUT);
	HAL_UART_Receive(lidar->huart, scan_data, sizeof(scan_data), UART_TIMEOUT);
}
