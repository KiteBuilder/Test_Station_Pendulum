/*
 ******************************************************************************
 * @file           : KeyPad.h
 * @brief          : Header for KeyPad.c file.
 *                   Subroutines for keys support
 ******************************************************************************
 *  Created on: May 24, 2026
 *      Author: KiteBuilder
 */

#ifndef KEY_PAD_H_
#define KEY_PAD_H_

#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <stdbool.h>

#define R_DIV_CONST 15000 //Omh
#define NUM_CONVERSIONS 5 //number of ADC conversions

#define UP_KEY_VAL      0x07
#define DOWN_KEY_VAL    0x00
#define RIGH_KEY_VAL    0x04
#define LEFT_KEY_VAL    0x0A
#define ENTER_KEY_VAL   0x0B
#define NO_KEY_VAL      0x0F

 void KeyPad_Scan(ADC_HandleTypeDef* p_hadc);
 void KeyPad_Handle(ADC_HandleTypeDef* p_hadc);
 uint16_t KeyPad_GetVal();
 bool KeyPad_IsPressed();
 void KeyPad_Release();

#endif /* KEY_PAD_H_ */