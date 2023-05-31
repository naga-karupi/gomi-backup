/*
 * Vesc_CANBUS_Control.h
 *
 *  Created on: Oct 17, 2022
 *      Author: ke_to
 */

#ifndef VESC_CANBUS_CONTROL_H_
#define VESC_CANBUS_CONTROL_H_

#include"main.h"

#ifdef HAL_CAN_MODULE_ENABLED

typedef struct {
	CAN_HandleTypeDef *hcan;
	CAN_RxHeaderTypeDef RxHeader;
	CAN_TxHeaderTypeDef TxHeader;
	CAN_FilterTypeDef filter;
	uint8_t rxbuf[8];
	uint32_t Vesc_ID;
} Vesc_CANBUS_Control_Typedef;

void Vesc_CANBUS_Control_Init(Vesc_CANBUS_Control_Typedef *vesc,CAN_HandleTypeDef *hcan,uint32_t id);
void Vesc_CANBUS_Control_Duty(Vesc_CANBUS_Control_Typedef *vesc,float duty);
void Vesc_CANBUS_Cotnrol_RPM(Vesc_CANBUS_Control_Typedef *vesc,float rpm);
void Vesc_CANBUS_Cotnrol_Current(Vesc_CANBUS_Control_Typedef *vesc,float current);

#endif

#ifdef HAL_FDCAN_MODULE_ENABLED

typedef struct {
	FDCAN_HandleTypeDef *hfdcan;
	FDCAN_RxHeaderTypeDef RxHeader;
	FDCAN_TxHeaderTypeDef TxHeader;
	FDCAN_FilterTypeDef filter;
	uint8_t rxbuf[8];
	uint32_t Vesc_ID;
} Vesc_CANBUS_Control_Typedef;


void Vesc_CANBUS_Control_Init(Vesc_CANBUS_Control_Typedef *vesc,FDCAN_HandleTypeDef *hfdcan,uint32_t id);
void Vesc_CANBUS_Control_Duty(Vesc_CANBUS_Control_Typedef *vesc,float duty);
void Vesc_CANBUS_Cotnrol_RPM(Vesc_CANBUS_Control_Typedef *vesc,float rpm);
void Vesc_CANBUS_Cotnrol_Current(Vesc_CANBUS_Control_Typedef *vesc,float current);

#endif

#endif /* VESC_CANBUS_CONTROL_H_ */
