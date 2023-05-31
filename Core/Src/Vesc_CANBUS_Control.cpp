/*
 * Vesc_CANBUS_Control.c
 *
 *  Created on: Oct 17, 2022
 *      Author: ke_to
 */

#include "Vesc_CANBUS_Control.hpp"
#include "buffer.hpp"

#ifdef HAL_CAN_MODULE_ENABLED

static void vesc_canbus_control_senddata(Vesc_CANBUS_Control_Typedef *vesc,
		uint32_t extid, uint8_t *txdata, uint8_t size) {
	uint32_t TxMailbox = 0;
	vesc->TxHeader.ExtId = extid;
	vesc->TxHeader.IDE = CAN_ID_EXT;
	vesc->TxHeader.RTR = CAN_RTR_DATA;
	vesc->TxHeader.DLC = size;
	vesc->TxHeader.TransmitGlobalTime = DISABLE;
	HAL_CAN_ConfigFilter(vesc->hcan, &vesc->filter);

	if (HAL_CAN_GetTxMailboxesFreeLevel(vesc->hcan) > 0){
		HAL_CAN_AddTxMessage(vesc->hcan, &(vesc->TxHeader), txdata, &TxMailbox);

	}
	else {
		HAL_CAN_Stop(vesc->hcan);
		HAL_CAN_Start(vesc->hcan);

		HAL_CAN_AbortTxRequest(vesc->hcan, CAN_TX_MAILBOX0);
		HAL_CAN_AbortTxRequest(vesc->hcan, CAN_TX_MAILBOX1);
		HAL_CAN_AbortTxRequest(vesc->hcan, CAN_TX_MAILBOX2);
		HAL_CAN_AddTxMessage(vesc->hcan, &(vesc->TxHeader), txdata, &TxMailbox);

	}
}

void Vesc_CANBUS_Control_Init(Vesc_CANBUS_Control_Typedef *vesc,
		CAN_HandleTypeDef *hcan, uint32_t id) { //Please Initialize CAN Module before do this
	vesc->Vesc_ID = id;
	static int slave_count = 0;
	vesc->hcan = hcan;
	vesc->filter.FilterIdHigh = 0;
	vesc->filter.FilterIdLow = 0;
	vesc->filter.FilterMaskIdHigh = 0;
	vesc->filter.FilterMaskIdLow = 0;
	vesc->filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	vesc->filter.FilterBank = slave_count * 14;
	vesc->filter.FilterMode = CAN_FILTERMODE_IDMASK;
	vesc->filter.FilterScale = CAN_FILTERSCALE_32BIT;
	vesc->filter.FilterActivation = CAN_FILTER_ENABLE;
	vesc->filter.SlaveStartFilterBank = 14;
}
void Vesc_CANBUS_Control_Duty(Vesc_CANBUS_Control_Typedef *vesc, float duty) {
	uint32_t ExtID = vesc->Vesc_ID | 0x000;
	uint8_t send_data[4];
	int32_t index = 0;
	buffer_append_int32(send_data, (int32_t) (duty * 100000.0), &index);
	vesc_canbus_control_senddata(vesc, ExtID, send_data, 4);
}
void Vesc_CANBUS_Cotnrol_RPM(Vesc_CANBUS_Control_Typedef *vesc, float rpm) {
	uint32_t ExtID = vesc->Vesc_ID | 0x300;
	uint8_t send_data[4];
	int32_t index = 0;
	buffer_append_int32(send_data, (int32_t) (rpm), &index);
	vesc_canbus_control_senddata(vesc, ExtID, send_data, 4);
}
void Vesc_CANBUS_Cotnrol_Current(Vesc_CANBUS_Control_Typedef *vesc,
		float current) {
	uint32_t ExtID = vesc->Vesc_ID | 0x100;
	uint8_t send_data[4];
	int32_t index = 0;
	buffer_append_int32(send_data, (int32_t) (current * 1000.0), &index);
	vesc_canbus_control_senddata(vesc, ExtID, send_data, 4);
}

#endif

#ifdef HAL_FDCAN_MODULE_ENABLED

static void vesc_canbus_control_senddata(Vesc_CANBUS_Control_Typedef *vesc,
		uint32_t extid, uint8_t *txdata, uint8_t size) {
	vesc->TxHeader.Identifier = extid;
	vesc->TxHeader.IdType = FDCAN_EXTENDED_ID;
	vesc->TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	vesc->TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	vesc->TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	vesc->TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	vesc->TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	vesc->TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	vesc->TxHeader.MessageMarker = 0;


	if (HAL_FDCAN_GetTxFifoFreeLevel(vesc->hfdcan) > 0)
		if(HAL_FDCAN_AddMessageToTxFifoQ(vesc->hfdcan, &(vesc->TxHeader), txdata) == HAL_OK) {
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		}else {
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		}
	else {
		HAL_FDCAN_Stop(vesc->hfdcan);
		HAL_FDCAN_Start(vesc->hfdcan);

		HAL_FDCAN_AbortTxRequest(vesc->hfdcan, FDCAN_TX_BUFFER0);
		HAL_FDCAN_AbortTxRequest(vesc->hfdcan, FDCAN_TX_BUFFER1);
		HAL_FDCAN_AbortTxRequest(vesc->hfdcan, FDCAN_TX_BUFFER2);
		HAL_FDCAN_AddMessageToTxFifoQ(vesc->hfdcan, &(vesc->TxHeader), txdata);// need to check

	}
}

void Vesc_CANBUS_Control_Init(Vesc_CANBUS_Control_Typedef *vesc,
		FDCAN_HandleTypeDef *hcan, uint32_t id) { //Please Initialize CAN Module before do this
	vesc->Vesc_ID = id;

	vesc->hfdcan = hcan;
	vesc->filter.IdType = FDCAN_EXTENDED_ID;
	vesc->filter.FilterIndex = 0;
	vesc->filter.FilterType = FDCAN_FILTER_MASK;
	vesc->filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	vesc->filter.FilterID1 = 0x000;
	vesc->filter.FilterID2 = 0x000;

	HAL_FDCAN_ConfigFilter(vesc->hfdcan, &vesc->filter);
	HAL_FDCAN_Start(vesc->hfdcan);
}
void Vesc_CANBUS_Control_Duty(Vesc_CANBUS_Control_Typedef *vesc, float duty) {
	uint32_t ExtID = vesc->Vesc_ID | 0x000;
	uint8_t send_data[4];
	int32_t index = 0;
	buffer_append_int32(send_data, (int32_t) (duty * 100000.0), &index);
	vesc_canbus_control_senddata(vesc, ExtID, send_data, 4);
}
void Vesc_CANBUS_Cotnrol_RPM(Vesc_CANBUS_Control_Typedef *vesc, float rpm) {
	uint32_t ExtID = vesc->Vesc_ID | 0x300;
	uint8_t send_data[4];
	int32_t index = 0;
	buffer_append_int32(send_data, (int32_t) (rpm), &index);
	vesc_canbus_control_senddata(vesc, ExtID, send_data, 4);
}
void Vesc_CANBUS_Cotnrol_Current(Vesc_CANBUS_Control_Typedef *vesc,
		float current) {
	uint32_t ExtID = vesc->Vesc_ID | 0x100;
	uint8_t send_data[4];
	int32_t index = 0;
	buffer_append_int32(send_data, (int32_t) (current * 1000.0), &index);
	vesc_canbus_control_senddata(vesc, ExtID, send_data, 4);
}

#endif

