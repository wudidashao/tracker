/*
 * hdc1000.h
 * Header file for HDC1000 for I2C communication functions.
 *  Created on: Jan 15, 2015
 *      Author: a0273932
 *
 * Copyright 2015 Texas Instruments Incorporated. All rights reserved.
 */

#ifndef HDC1000_H_
#define HDC1000_H_



void HDC1000_Init(void);
int32_t HDC1000_TempRead(void);
int32_t getHumidity(void);

#endif /* HDC1000_H_ */
