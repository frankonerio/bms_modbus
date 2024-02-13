/*
 * modbus.c
 *
 *  Created on: Feb 12, 2024
 *      Author: frank
 */

#include "modbus.h"

uint8_t cnt_env = 0;
uint8_t func = 0;
uint8_t duty = 0;
uint8_t indice = 0;
uint8_t cntRx;
uint16_t contTx = 0;
uint8_t ERROR_COM = 0;
uint16_t cntERROR = 0;

uint8_t error_flag = 0;
_Bool flg_salva = 0;
_Bool flg_broadcast = 0;

unsigned char low_byte = 0x00;
unsigned char high_byte = 0x00;


void get_HB_LB(uint16_t  value, unsigned char *HB, unsigned char *LB)
{
    *LB = (value & 0x00FF);
    *HB = ((value & 0xFF00) >> 0x08);
}



uint16_t  make_word(unsigned char HB, unsigned char LB)
{
   uint16_t tmp = 0;

    tmp = HB;
    tmp <<= 8;
    tmp |= LB;

    return tmp;
}


uint16_t MODBUS_RTU_CRC16(unsigned char *data_input, unsigned char data_length)
{
	unsigned char n = 0x08;
	unsigned char s = 0x00;
	uint16_t CRC_word = 0xFFFF;

	for (s = 0x00; s < data_length; s++) {
		CRC_word ^= ((uint16_t) data_input[s]);

		n = 8;

		while (n > 0) {
			if ((CRC_word & 0x0001) == 0) {
				CRC_word >>= 1;
			}

			else {
				CRC_word >>= 1;
				CRC_word ^= 0xA001;
			}

			n--;
		}
	}

	return CRC_word;
}
void modbus_receive(uint8_t *rx_buffer, uint8_t *tx_buffer ,int recv_lenght, uint16_t *read_registers ){

	uint16_t temp1 = 0x0000;
	uint16_t temp2 = 0x0000;
	uint16_t temp3 = 0x0000;
	uint16_t temp4 = 0x0000;
	uint16_t temp5 = 0x0000;
	uint16_t temp6 = 0x0000;
	uint16_t temp7 = 0x0000;

	//memset(tx_buffer,0,sizeof(tx_buffer));

	//send(1,rx_buffer,recv_lenght);// the data

	if(rx_buffer[id_byte] ==  Slave_ID){
	tx_buffer[id_byte] = rx_buffer[id_byte];

	switch(rx_buffer[function_code_byte]) {

		case FC_read_holding_registers:
		{
			temp1 = make_word(rx_buffer[location_start_high_byte], rx_buffer[location_start_low_byte]);
			if ((temp1 >= addr_holding_reg_start) && (temp1 <= addr_holding_reg_end))
			{
				temp2 = make_word(rx_buffer[location_end_high_byte], rx_buffer[location_end_low_byte]);
				if ((temp2 <= no_of_holding_regs) && (temp2 + temp1 - 1) <= addr_holding_reg_end)
				{
					tx_buffer[transaction_id_high_byte] = rx_buffer[transaction_id_high_byte];
					tx_buffer[transaction_id_low_byte] = rx_buffer[transaction_id_low_byte];

						tx_buffer[byte_size_low_byte]= (temp2 *2) + 3;
						tx_buffer[8] = (temp2 * 2);
						tx_buffer[function_code_byte] = rx_buffer[function_code_byte];


					if (temp2 > 1)
					{
						int i = 0;
						for (temp3 = (temp1 - addr_holding_reg_start); temp3 < temp2; temp3++)
						{
							get_HB_LB(read_registers[((temp1 - addr_holding_reg_start) + temp3)], &high_byte, &low_byte);
							tx_buffer[9 + i] = high_byte;
							tx_buffer[10 + i] = low_byte;
							i = i + 2;
						}
					}

					else
					{
						get_HB_LB(read_registers[(temp1 - addr_holding_reg_start)], &high_byte, &low_byte);
						tx_buffer[1] = high_byte;
						tx_buffer[2] = low_byte;
					}
					//memset(tx_buffer,0,sizeof(tx_buffer));
					send(1,tx_buffer,((temp2 * 2)+ 9 ));
					//memset(tx_buffer,0,sizeof(tx_buffer));

				}

				else
				{
					error_flag = 1;
				}
			}

			else
			{
				error_flag = 1;
			}
			printf("You entered 1.\n");
			break;
		}


		case FC_write_single_register:
			// Code to execute if choice is 2
			printf("You entered 2.\n");
			break;
		case FC_write_multiple_registers:
			// Code to execute if choice is 3
			printf("You entered 3.\n");
			break;
		default:
			// Code to execute if choice doesn't match any case
			printf("Invalid choice.\n");
			break;
		}
	}


}

void modbus_transmit(){

}
void MODBUS_receive_task(uint8_t *RX_buffer, uint8_t *TX_buffer, uint16_t *holding_registers, uint8_t TX_buffer_size, uint8_t RX_buffer_size, uint8_t *FrameNrBytes)
{

	unsigned char low_byte = 0x00;
	unsigned char high_byte = 0x00;

	uint16_t temp1 = 0x0000;
	uint16_t temp2 = 0x0000;
	uint16_t temp3 = 0x0000;
	uint16_t temp4 = 0x0000;
	uint16_t temp5 = 0x0000;
	uint16_t temp6 = 0x0000;
	uint16_t temp7 = 0x0000;

	unsigned char data_array[128];

	for (int i = 0; i < 128; i++) {
		data_array[i] = 0;
	}

//	temp7 = make_word(RX_buffer[transaction_id_low_byte],RX_buffer[transaction_id_low_byte]);
//	data_array[0] = temp7;

		switch (RX_buffer[function_code_byte])
		{

#if ENABLE_MB_FUNCTION00_BROADCAST >0
		case FC_broadcast:
		{

			temp3 = make_word(RX_buffer[CRC_low_byte], RX_buffer[CRC_high_byte]);
			temp4 = MODBUS_RTU_CRC16(RX_buffer, 6);

			if(temp4 == temp3) {

				flg_broadcast = 1;

			}

		break;
		}


#endif

#if ENABLE_MB_FUNCTION01_READ_COILS >0

		case FC_read_coils:
		{

			break;
		}
#endif

#if ENABLE_MB_FUNCTION02_READ_DISCRETE_INPUTS >0

		case FC_read_discrete_inputs:
		{


			break;
		}
#endif

#if ENABLE_MB_FUNCTION03_READ_HOLDING_REGISTERS > 0

		case FC_read_holding_registers:
		{
			temp1 = make_word(RX_buffer[location_start_high_byte], RX_buffer[location_start_low_byte]);

			if ((temp1 >= addr_holding_reg_start) && (temp1 <= addr_holding_reg_end))
			{
				temp2 = make_word(RX_buffer[location_end_high_byte], RX_buffer[location_end_low_byte]);

				if ((temp2 <= no_of_holding_regs) && (temp2 + temp1 - 1) <= addr_holding_reg_end)
				{
						data_array[0] = temp2;
						data_array[0] <<= 1;

						if (temp2 > 1)
						{

							for (temp3 = (temp1 - addr_holding_reg_start); temp3 < temp2; temp3++)
							{
								get_HB_LB(holding_registers[((temp1 - addr_holding_reg_start) + temp3)], &high_byte, &low_byte);
								data_array[1 + temp3 + temp3] = high_byte;
								data_array[2 + temp3 + temp3] = low_byte;
							}
						}

						else
						{
							get_HB_LB(holding_registers[(temp1 - addr_holding_reg_start)], &high_byte, &low_byte);
							data_array[1] = high_byte;
							data_array[2] = low_byte;
						}

						MODBUS_send_task(FC_read_holding_registers,((temp2 << 1) + 1), data_array, TX_buffer, RX_buffer, TX_buffer_size, FrameNrBytes);

				}

				else
				{
					error_flag = 1;
				}
			}

			else
			{
				error_flag = 1;
			}

			switch (error_flag)
			{
			case 1:
			{
				data_array[0] = 0x02;
				MODBUS_send_task((FC_read_holding_registers | 0x80), 1, data_array, TX_buffer, RX_buffer, TX_buffer_size, FrameNrBytes);
				break;
			}
			default:
			{
				break;
			}
			}

			break;
		}
#endif

#if   ENABLE_MB_FUNCTION04_READ_INPUT_REGISTER > 0
			case FC_read_input_registers:
			{


				break;
			}
#endif

#if  ENABLE_MB_FUNCTION05_WRITE_SINGLE_COILS >0

			case FC_write_single_coil:
			{

				break;
			}
#endif

#if   ENABLE_MB_FUNCTION06_WRITE_SINGLE_REGISTER > 0

		case FC_write_single_register:
		{

			temp1 = make_word(RX_buffer[location_start_high_byte], RX_buffer[location_start_low_byte]);

			if ((temp1 >= addr_holding_reg_start) && (temp1 <= addr_holding_reg_end))
			{
				#if (ENABLE_MB_TCP == 0)&&(ENABLE_MB_RTU == 1)
				temp3 = make_word(RX_buffer[CRC_low_byte], RX_buffer[CRC_high_byte]);
				temp4 = MODBUS_RTU_CRC16(RX_buffer, 6);

				if (temp4 == temp3)
				{
				#endif
					temp2 = make_word(RX_buffer[location_end_high_byte], RX_buffer[location_end_low_byte]);
					holding_registers[temp1 - addr_holding_reg_start] = temp2;

					data_array[0] = RX_buffer[location_start_high_byte];
					data_array[1] = RX_buffer[location_start_low_byte];
					data_array[2] = RX_buffer[location_end_high_byte];
					data_array[3] = RX_buffer[location_end_low_byte];
					MODBUS_send_task(FC_write_single_register, 4, data_array, TX_buffer, RX_buffer, TX_buffer_size, FrameNrBytes);
				#if (ENABLE_MB_TCP == 0)&&(ENABLE_MB_RTU == 1)
				}
				else
				{
					error_flag = 1;
				}
				#endif
			}

			else
			{
				error_flag = 1;
			}

			switch (error_flag)
			{
			case 1:
			{
				data_array[0] = 0x02;
				MODBUS_send_task((FC_write_single_register | 0x80), 1, data_array, TX_buffer, RX_buffer, TX_buffer_size, FrameNrBytes);
				break;
			}
			default:
			{
				break;
			}
			}

			break;
		}
#endif
#if ENABLE_MB_FUNCTION015_MUTIPLES_COILS   >0

			case FC_write_multiple_coils:
			{

				break;
			}

#endif
#if ENABLE_MB_FUNCTION16_WRITE_MULTIPLE_REGISTERS   > 0
		case FC_write_multiple_registers:
		{
			temp1 = make_word(RX_buffer[location_start_high_byte], RX_buffer[location_start_low_byte]);

			if ((temp1 >= addr_holding_reg_start) && (temp1 <= addr_holding_reg_end))
			{
				temp2 = make_word(RX_buffer[location_end_high_byte], RX_buffer[location_end_low_byte]);
				temp3 = (temp2 + temp1 - 1);

				if ((temp2 <= no_of_holding_regs) && (temp3 <= addr_holding_reg_end))
				{
					temp3 = RX_buffer[byte_size_byte];
					temp4 = (temp2 << 1);

					if (temp3 == temp4)
					{
							temp5 = (temp1 - addr_holding_reg_start);
							temp3 = mandatory_bytes_to_read;

							if (temp2 == 1)
							{
								holding_registers[temp5] = make_word(RX_buffer[temp3], RX_buffer[(1 + temp3)]);
							}

							else
							{
								for (temp6 = temp5; temp6 < (temp2 + temp5); temp6++)
								{
									holding_registers[temp6] = make_word(RX_buffer[temp3], RX_buffer[(1 + temp3)]);
									temp3 += 2;
								}
							}

							data_array[0] = RX_buffer[location_start_high_byte];
							data_array[1] = RX_buffer[location_start_low_byte];
							data_array[2] = RX_buffer[location_end_high_byte];
							data_array[3] = RX_buffer[location_end_low_byte];

							MODBUS_send_task(FC_write_multiple_registers, 4, data_array, TX_buffer, RX_buffer, TX_buffer_size, FrameNrBytes);

					}

					else
					{
						error_flag = 1;
					}
				}

				else
				{
					error_flag = 1;
				}
			}

			else
			{
				error_flag = 1;
			}

			switch (error_flag)
			{
			case 1:
			{
				data_array[0] = 0x02;
				MODBUS_send_task((FC_write_multiple_registers | 0x80), 1, data_array, TX_buffer, RX_buffer, TX_buffer_size, FrameNrBytes);
				break;
			}

			default:
			{
				break;
			}
			}

			break;
		}
#endif

		default:
		{
			break;
		}
		}


	if (error_flag == 1)
	{
		cntERROR++;
		error_flag = 0;

	}

	//clean RX_buffer
	memset(RX_buffer,0,RX_buffer_size);
}




void MODBUS_send_task(unsigned char function_code, unsigned char data_length, unsigned char *values, uint8_t *TX_buffer, uint8_t *RX_buffer, uint8_t TX_buffer_size, uint8_t *FrameNrBytes)
{
	unsigned char byte_count = 0x00;

	//clean TX_buffer
	memset(TX_buffer,0,TX_buffer_size);
	TX_buffer[0] = RX_buffer[0];
	TX_buffer[1] = RX_buffer[1];
	TX_buffer[2] = 0;
	TX_buffer[3] = 0;
	TX_buffer[4] = 0;
	TX_buffer[5] = data_length + 2;
	TX_buffer[6] = RX_buffer[6];
	TX_buffer[function_code_byte] = function_code;

	for (byte_count = 0; byte_count < data_length; byte_count++)
	{
		TX_buffer[8 + byte_count] = values[byte_count];
	}
	*FrameNrBytes = 8 + data_length;
	send(1,TX_buffer,FrameNrBytes);
}

__weak void RTU_RX_Int(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the RTU_RX_Int could be implemented in the user file
   */
}
