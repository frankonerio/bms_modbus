/*
 * modbus.h
 *
 *  Created on: Feb 12, 2024
 *      Author: frank
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "socket.h"
#include "wizchip_conf.h"



#define Slave_ID_broadcast                      0x00 //10
#define Slave_ID                                0x01 //01
#define FC_broadcast                            0x00
#define FC_read_coils                           0x01
#define FC_read_discrete_inputs                 0x02
#define FC_read_holding_registers               0x03
#define FC_read_input_registers                 0x04
#define FC_write_single_coil                    0x05
#define FC_write_single_register                0x06
#define FC_write_multiple_coils                 0x0F
#define FC_write_multiple_registers             0x10




#define byte_length_byte                        2

#define transaction_id_high_byte				0
#define transaction_id_low_byte					1
#define protocol_nr_high_byte					2
#define protocol_nr_low_byte					3

#define byte_size_high_byte						4
#define byte_size_low_byte						5

#define id_byte                                 6
#define function_code_byte                      7

#define location_start_high_byte                8
#define location_start_low_byte                 9

#define addr_holding_reg_start                  0
#define addr_holding_reg_end                    63

#define location_end_high_byte                  10
#define location_end_low_byte                   11

#define byte_size_byte                          12

#define no_of_holding_regs                      64

#define TX_buffer_length                        128
#define RX_buffer_length                        128

#define mandatory_bytes_to_read                 13

#define fixed_no_of_bytes_to_read               5


#define ENABLE_MB_FUNCTION00_BROADCAST                ( 0 )
#define ENABLE_MB_FUNCTION01_READ_COILS               ( 0 )
#define ENABLE_MB_FUNCTION02_READ_DISCRETE_INPUTS     ( 0 )
#define ENABLE_MB_FUNCTION03_READ_HOLDING_REGISTERS   ( 1 )
#define ENABLE_MB_FUNCTION04_READ_INPUT_REGISTER      ( 0 )
#define ENABLE_MB_FUNCTION05_WRITE_SINGLE_COILS       ( 0 )
#define ENABLE_MB_FUNCTION06_WRITE_SINGLE_REGISTER    ( 0 )
#define ENABLE_MB_FUNCTION015_MUTIPLES_COILS          ( 0 )
#define ENABLE_MB_FUNCTION16_WRITE_MULTIPLE_REGISTERS ( 1 )


uint16_t  MODBUS_RTU_CRC16(unsigned char *data_input, unsigned char data_length);
void MODBUS_receive_task(uint8_t *RX_buffer, uint8_t *TX_buffer, uint16_t *holding_registers, uint8_t TX_buffer_size, uint8_t RX_buffer_size, uint8_t *FrameNrBytes);
void MODBUS_send_task(unsigned char function_code, unsigned char data_length, unsigned char *values, uint8_t *TX_buffer, uint8_t *RX_buffer, uint8_t TX_buffer_size, uint8_t *FrameNrBytes);

uint16_t  make_word(unsigned char HB, unsigned char LB);
void get_HB_LB(uint16_t  value, unsigned char *HB, unsigned char *LB);

void RTU_RX_Int(UART_HandleTypeDef* huart);

void modbus_receive(uint8_t *rx_buffer, uint8_t *tx_buffer ,int recv_lenght, uint16_t *read_registers );

#endif /* INC_MODBUS_H_ */
