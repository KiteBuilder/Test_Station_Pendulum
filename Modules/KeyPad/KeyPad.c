/*
 ******************************************************************************
 * @file           : KeyPad.c
 * @brief          : Subroutines for keys support
 ******************************************************************************
 *  Created on: May 24, 2026
 *      Author: KiteBuilder
 */
#include "KeyPad.h"

uint16_t dma_buff[NUM_CONVERSIONS];
uint16_t key_val = NO_KEY_VAL, prev_key_val = NO_KEY_VAL;
bool f_pressed = false;

/**
 * @brief 
 * @param None
 * @retval None
 */
 void KeyPad_Scan(ADC_HandleTypeDef* p_hadc)
 {
    if (!f_pressed) //no start conversion until previously pressed key wasn't handled
    {
        HAL_ADC_Start_DMA(p_hadc, (uint32_t*)dma_buff, NUM_CONVERSIONS); 
    }
 }
 
/**
 * @brief 
 * @param None
 * @retval None
 */
void KeyPad_Handle(ADC_HandleTypeDef* p_hadc)
{
    HAL_ADC_Stop_DMA(p_hadc);

    key_val = 0;

    for (uint32_t i = 0; i < NUM_CONVERSIONS; i++)
    {
        key_val += dma_buff[i];
    }

    key_val /= NUM_CONVERSIONS;

    key_val >>= 8; //Key value located in the second byte

    if (key_val != NO_KEY_VAL && key_val != prev_key_val)
    {
        if (key_val == UP_KEY_VAL   || 
            key_val == DOWN_KEY_VAL || 
            key_val == LEFT_KEY_VAL || 
            key_val == RIGH_KEY_VAL || 
            key_val == ENTER_KEY_VAL)
        {
            f_pressed = true;
            prev_key_val = key_val; //prevent repititious handling until key pressed
        }
    }
    else
    {
        prev_key_val = key_val;
    }

}

/**
 * @brief 
 * @param None
 * @retval None
 */
uint16_t KeyPad_GetVal()
{
    return key_val;
}

/**
 * @brief 
 * @param None
 * @retval None
 */
bool KeyPad_IsPressed()
{
    return f_pressed;
}

/**
 * @brief 
 * @param None
 * @retval None
 */
void KeyPad_Release()
{
    f_pressed = false;
}