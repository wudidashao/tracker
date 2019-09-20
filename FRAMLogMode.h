/*
 * config.h
 *
 *  Created on: 2018Äê7ÔÂ16ÈÕ
 *      Author: zlzhang
 */


#ifndef OUTOFBOX_FR5969_NEWD_FRAMLOGMODE_H_
#define OUTOFBOX_FR5969_NEWD_FRAMLOGMODE_H_

#include <stdint.h>

#define FRAM_LOG_MODE      '2'
#define TRANSMIT_FRAM_DATA_MODE '3'
#define send_none_frame    1
extern uint8_t sensordata[];
extern uint16_t voltageData[];
extern uint16_t dataIndex;
extern uint8_t timeStamp[];

void framLog(uint8_t *ptr);
void sendDataFRAM(void);
void sendTimeStampFRAM(void);
void storeTimeStampFRAM(void);
void sendDataFRAM_fifo(void);
void framLog_fifo(uint8_t *ptr);
void send_data_fifo(void);
#endif /* OUTOFBOX_FR5969_NEWD_FRAMLOGMODE_H_ */
