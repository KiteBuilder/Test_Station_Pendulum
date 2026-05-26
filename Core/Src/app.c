#include "stm32f4xx_hal.h"
#include "fatfs.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define ILI9341_INCLUDE_FONT_6x8
#define ILI9341_INCLUDE_FONT_7x10
#define ILI9341_INCLUDE_FONT_11x18
#define ILI9341_INCLUDE_FONT_16x26
#include "ILI9341_Driver.h"
#include "Graph.h"
#include "TouchScreen.h"
#include "DebugProtocol.h"
#include "Times.h"
#include "Key.h"
#include "KeyPad.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

typedef struct
{
    FATFS FatFs;         //Fatfs handle
    FIL fil;             //file handle
    uint16_t findex;     //file index
    char fname[16];      //file name
    FRESULT status;         //OK - true, ERROR - false
    uint32_t numRecords; //records counter
} file_t;

timeUs_t currentTimeUs = 0, previousTimeUs = 0;

static void InitGraphInterface();
static void GraphsAndTextUpdate(timeDelta_t, float*);
static void Print_Screen_2();
static void InitFileSystem(file_t*);
static void FileDataUpdate(file_t*, timeUs_t, float*);
static void FileSync(file_t*);
static void Text_Scr_Key_Handler(key_state_e);
static void KeyPad_Handler();

ILI9341_Port cs  = {TFT_CS_GPIO_Port , TFT_CS_Pin };
ILI9341_Port dc  = {TFT_DC_GPIO_Port , TFT_DC_Pin };
ILI9341_Port rst = {TFT_RST_GPIO_Port, TFT_RST_Pin};
ILI9341_Port led = {TFT_LED_GPIO_Port, TFT_LED_Pin};

Touch_Port touch_cs  = {TOUCH_CS_GPIO_Port , TOUCH_CS_Pin };
Touch_Port touch_int  = {TOUCH_INT_GPIO_Port , TOUCH_INT_Pin };

#ifdef ILI9341_INCLUDE_FONT_6x8
ILI9341_FontDef Font_6x8 = {6, 8, Font6x8, 32, 126};
#endif

#ifdef ILI9341_INCLUDE_FONT_7x10
ILI9341_FontDef Font_7x10 = {7, 10, Font7x10, 32, 126};
#endif

#ifdef ILI9341_INCLUDE_FONT_11x18
ILI9341_FontDef Font_11x18 = {11, 18, Font11x18, 32, 126};
#endif

#ifdef ILI9341_INCLUDE_FONT_16x26
ILI9341_FontDef Font_16x26 = {16, 26, Font16x26, 32, 126};
#endif

rect_t power_wnd, pid_wnd, error_wnd, text_wnd;
point_t power_hdr, pid_hdr, error_hdr;

graph_t iBat_graph, vBat_graph;
graph_t pid_P_graph, pid_I_graph, pid_D_graph;
graph_t error_graph, roll_graph, setpoint_graph;

bool f_touch = false;
uint16_t guard_cnt = 0;
const uint16_t guard_threshold = 500;
uint16_t x, y;

#define MAX_TXT_SCR  3 //amount of text screens
uint8_t txt_scr_id = 0; //text screen id
key_t txt_scr_key;

const uint16_t data_num = 16;
float fltData[RX_MAX_CNT/sizeof(float)] = {0.0};

file_t file;
const uint16_t samples_threshold = 50; //to save the every 50 sample
float fltDataAvg[RX_MAX_CNT/sizeof(float)] = {0.0};
uint16_t samples_cnt = 0;

bool f_syncFile = false;
uint16_t syncFile_cnt = 0;
const uint16_t sync_period = 5000; //sync file period in milliseconds

bool first_filter_load = true;

bool f_scan = false;
uint16_t scan_cnt = 0;
const uint16_t scan_period = 10; //sync period for 1ms HAL_SYSTICK

/**
  * @brief
  * @param None
  * @retval None
  */
void initialization(void)
{
  usTicks = SystemCoreClock / 1000000;

  ILI9341_Set_Interface(&hspi2, true, &cs, &dc, &rst, &led);
  ILI9341_BackLight(true);
  ILI9341_Init();
  ILI9341_SetOrientation(SCREEN_VERTICAL_0GRAD);//SCREEN_HORIZONTAL_180GRAD);
  ILI9341_Clear(Black);

  Touch_Set_Interface(&hspi1, &touch_cs, &touch_int);

  InitGraphInterface();

  Debug_InitProtocol(&huart1, fltData);

  InitFileSystem(&file);

  Key_Init(&txt_scr_key, GPIOA, GPIO_PIN_0, LO_LEVEL, &Text_Scr_Key_Handler);

}

/**
  * @brief
  * @param None
  * @retval None
  */
void exec(void)
{
    if (Debug_IsRxready())
    {
        currentTimeUs = micros();
        timeDelta_t dT = currentTimeUs - previousTimeUs;
        previousTimeUs = currentTimeUs;
        for (uint32_t i = 0; i < data_num; i++)
        {
            fltDataAvg[i] += fltData[i];
        }
        GraphsAndTextUpdate(dT, fltData);
        if (++samples_cnt == samples_threshold)
        {
            samples_cnt = 0;
            for (uint32_t i = 0; i < data_num; i++)
            {
                fltDataAvg[i] /= samples_threshold;
            }
            FileDataUpdate(&file, currentTimeUs, fltDataAvg);
            for (uint32_t i = 0; i < data_num; i++)
            {
                fltDataAvg[i] = 0;
            }
        }
    }

    if (f_touch == true)
    {
        f_touch = false;
        if ((DISPLAY_PIX_WIDTH - y) < text_wnd.bottom)
        {
            if (++txt_scr_id >= MAX_TXT_SCR)
            {
                    txt_scr_id = 0;
            }

            ILI9341_DrawFillRectangle(text_wnd.left, text_wnd.top, text_wnd.right, text_wnd.bottom, Black);
        }
    }

    if (f_syncFile == true)
    {
        f_syncFile = false;
        FileSync(&file);
    }

    if (f_scan == true)
    {
        f_scan = false;

        Key_CheckState(&txt_scr_key);

        if (KeyPad_IsPressed())
        {
            KeyPad_Handler();
        }
    }
}

/**
  * @brief UART RX complete callback
  * @param None
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Debug_RxCpltCallback(huart);
}

/**
  * @brief
  * @retval None
  */
void HAL_SYSTICK_Callback()
{
    if(guard_cnt != 0)
    {
        --guard_cnt;
    }

    if (++syncFile_cnt == sync_period)
    {
        syncFile_cnt = 0;
        f_syncFile = true;
    }

    if (++scan_cnt == scan_period)
    {
        scan_cnt = 0;
        f_scan = true;

        KeyPad_Scan(&hadc1);
    }
}

/**
  * @brief
  * @param None
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == touch_int.pin)
    {
        if (guard_cnt == 0 && f_touch == false)
        {
            if (HAL_GPIO_ReadPin(touch_int.gpio, touch_int.pin) == GPIO_PIN_RESET)
            {
                Touch_Get_Coordinates(&x, &y, true);
                f_touch = true;
            }

            guard_cnt =  guard_threshold;
        }
    }
}

/**
  * @brief 
  * @param None
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    KeyPad_Handle(hadc);
}

/**
  * @brief  To draw two windows for the Graphs
  * @retval None
  */
static void InitGraphInterface()
{
    char str[32];
    const uint16_t wnd_height = 130;
    const uint16_t y_offset= 70;

    power_hdr.x = 5; power_hdr.y = y_offset - Font_7x10.height;
    sprintf(str, "Power");
    ILI9341_WriteString(str, Font_7x10, power_hdr.x, power_hdr.y, Yellow, Red);

    pid_hdr.x = 50; pid_hdr.y = y_offset - Font_7x10.height;
    sprintf(str, "PID");
    ILI9341_WriteString(str, Font_7x10, pid_hdr.x, pid_hdr.y, Blue, Green);

    error_hdr.x = 80; error_hdr.y = y_offset - Font_7x10.height;
    sprintf(str, "Error");
    ILI9341_WriteString(str, Font_7x10, error_hdr.x, error_hdr.y, Green, Blue);

    power_wnd.left   = 1;
    power_wnd.top    = y_offset;
    power_wnd.right  = 318;
    power_wnd.bottom = power_wnd.top + wnd_height;
    Graph_InitDynamic(&power_wnd, &iBat_graph, 0, 3000, Red, Black);
    Graph_InitDynamic(&power_wnd, &vBat_graph, 0, 2000, Yellow, Black);

    pid_wnd.left   = 1;
    pid_wnd.top    = power_wnd.bottom + 5;
    pid_wnd.right  = 318;
    pid_wnd.bottom = pid_wnd.top + wnd_height;
    Graph_InitDynamic(&pid_wnd, &pid_P_graph, -1000, 1000, Red, Black);
    Graph_InitDynamic(&pid_wnd, &pid_I_graph, -1000, 1000, Green, Black);
    Graph_InitDynamic(&pid_wnd, &pid_D_graph, -1000, 1000, Blue, Black);        

    error_wnd.left   = 1;
    error_wnd.top    = pid_wnd.bottom + 5;
    error_wnd.right  = 318;
    error_wnd.bottom = error_wnd.top + wnd_height;
    Graph_InitDynamic(&error_wnd, &error_graph, -900, 900, Red, Black);
    Graph_InitDynamic(&error_wnd, &roll_graph, -900, 900, GreenYellow, Black);
    Graph_InitDynamic(&error_wnd, &setpoint_graph, -900, 900, Magenta, Black);

    text_wnd.left = 0; text_wnd.right = 320; text_wnd.top = 0; text_wnd.bottom = Font_16x26.height * 2;
}


/**
  * @brief  To draw two windows for the Graphs
  * @retval None
  */
static void Print_Screen_0(float *flt_data)
{
    char str[32];
    point_t point;
    uint32_t data;

    //IBat filtered
    point.x = 0; point.y = Font_16x26.height + 5;
    data = (uint32_t)(fltData[0] * 100);
    sprintf(str, "I%2lu.%02lu", data/100, data % 100);
    ILI9341_WriteString(str, Font_16x26, point.x, point.y, Red, Black);

    //vBat filtered
    point.x = 0; point.y = 0;
    data = (uint32_t)(fltData[1] * 10);
    sprintf(str, "V%2lu.%1lu", data/10, data % 10);
    ILI9341_WriteString(str, Font_16x26, point.x, point.y, Green, Black);

    //Capacity mAh
    point.x = Font_16x26.width * 8; point.y = Font_16x26.height + 5;
    data = (uint32_t)(fltData[2] * 10);
    sprintf(str, "E%4lu.%1lumAh", data/10, data % 10);
    ILI9341_WriteString(str, Font_16x26, point.x, point.y, Orange, Black);
}

/**
  * @brief  To draw two windows for the Graphs
  * @retval None
  */
static void Print_Screen_1(float *flt_data)
{
    char str[32];
    point_t point;
    uint32_t data;
    char sign;

    //Left Motor Throttle
    point.x = 0; point.y = 0;
    data = (uint32_t)(fltData[9]);
    sprintf(str, "Lm:%4lu", data);
    ILI9341_WriteString(str, Font_11x18, point.x, point.y, Yellow, Black);

    //Right Motor Throttle
    point.x = Font_11x18.width * 8; point.y = 0;
    data = (uint32_t)(fltData[10]);
    sprintf(str, "Rm:%4lu", data);
    ILI9341_WriteString(str, Font_11x18, point.x, point.y, Yellow, Black);    

    //PID P
    point.x = 0; point.y = Font_11x18.height + 2;
    data = (uint32_t)(fabs(fltData[6]) * 10);
    sign = (fltData[6] < 0.0f) ? '-' : ' ';
    sprintf(str, "P:%c%3lu.%1lu", sign, data/10, data % 10);
    ILI9341_WriteString(str, Font_11x18, point.x, point.y, Red, Black);

    //PID I
    point.x = Font_11x18.width * 9; point.y = Font_11x18.height + 2;
    data = (uint32_t)(fabs(fltData[7]) * 10);
    sign = (fltData[7] < 0.0f) ? '-' : ' ';
    sprintf(str, "I:%c%3lu.%1lu", sign, data/10, data % 10);
    ILI9341_WriteString(str, Font_11x18, point.x, point.y, Green, Black);    

    //PID D
    point.x = (Font_11x18.width * 9) * 2; point.y = Font_11x18.height + 2;
    data = (uint32_t)(fabs(fltData[8]) * 10);
    sign = (fltData[8] < 0.0f) ? '-' : ' ';
    sprintf(str, "D:%c%3lu.%1lu", sign, data/10, data % 10);
    ILI9341_WriteString(str, Font_11x18, point.x, point.y, Blue, Black);

    //PID Error
    point.x = 0; point.y = (Font_11x18.height + 2) * 2;
    data = (uint32_t)(fabs(fltData[5]) * 10);
    sign = (fltData[5] < 0.0f) ? '-' : ' ';
    sprintf(str, "Err:%c%2lu.%1lu", sign, data/10, data % 10);
    ILI9341_WriteString(str, Font_11x18, point.x, point.y, Red, Black);

    //Roll
    point.x = Font_11x18.width * 12; point.y = (Font_11x18.height + 2) * 2;
    data = (uint32_t)(fabs(fltData[4]) * 10);
    sign = (fltData[4] < 0.0f) ? '-' : ' ';
    sprintf(str, "Roll:%c%2lu.%1lu", sign, data/10, data % 10);
    ILI9341_WriteString(str, Font_11x18, point.x, point.y, Green, Black);
}

/**
  * @brief  To draw two windows for the Graphs
  * @retval None
  */
static void Print_Screen_2()
{
    char str[32];
    point_t point;
    uint16_t data;

    point.x = 0; point.y = 0;
    data = KeyPad_GetVal();
    sprintf(str, "Key_val: 0x%4X", data);
    ILI9341_WriteString(str, Font_11x18, point.x, point.y, Yellow, Black);
}

/**
  * @brief  To plot graphs and update text information
  * @retval None
  */
static void GraphsAndTextUpdate(timeDelta_t dT, float *flt_data)
{
    //Top information window update
    switch (txt_scr_id)
    {
        case 0:
            Print_Screen_0(flt_data);
            break;

        case 1:
            Print_Screen_1(flt_data);
            break;

        case 2:
            Print_Screen_2();
    }

    //Graph windows update
    int16_t iBat_int = (int16_t)(fltData[0] * 100);
    int16_t vBat_int = (int16_t)(fltData[1] * 10);
    Graph_DynamicDraw(iBat_int, &iBat_graph, true);
    Graph_DynamicDraw(vBat_int, &vBat_graph, true);

    int16_t pid_P_int = (int16_t)(fltData[6]);
    int16_t pid_I_int = (int16_t)(fltData[7]);
    int16_t pid_D_int = (int16_t)(fltData[8]);        
    Graph_DynamicDraw(pid_P_int, &pid_P_graph, true);
    Graph_DynamicDraw(pid_I_int, &pid_I_graph, true);
    Graph_DynamicDraw(pid_D_int, &pid_D_graph, true);        

    int16_t setpoint_int = (int16_t)(fltData[3] * 10);
    int16_t roll_int = (int16_t)(fltData[4] * 10);
    int16_t error_int = (int16_t)(fltData[5] * 10);
    Graph_DynamicDraw(setpoint_int, &setpoint_graph, true);
    Graph_DynamicDraw(roll_int, &roll_graph, true);        
    Graph_DynamicDraw(error_int, &error_graph, true);
}

/**
  * @brief  Initialize a file system and create a file
  * @retval None
  */
static void InitFileSystem(file_t *file)
{
    DIR dir;
    FILINFO finfo;
    char str[256];

    file->findex = 0;

    file->status = f_mount(&file->FatFs, "", 1);

    while (file->status == FR_OK)
    {
        sprintf(file->fname, "log%d.txt", file->findex);
        file->status = f_findfirst(&dir, &finfo, "", file->fname);
        if (file->status == FR_OK && finfo.fname[0] == 0)
        {
            file->status = f_open(&file->fil, file->fname, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
            break;
        }
        else
        {
            ++file->findex;
        }
    }

    if (file->status == FR_OK)
    {
        sprintf(str, "    N:         T(ms):    U(V):     I(A):     E(Wh):    Lm():    Rm():       P():       I():       D():      Err:     Roll: \r\n");

        uint32_t bytesWrote;
        file->status = f_write(&file->fil, str, strlen(str), (UINT*)&bytesWrote);
        file->numRecords = 0;
    }
}

/**
  * @brief  Update file
  * @retval None
  */
static void FileDataUpdate(file_t *file, timeUs_t time, float *fltData)
{
    char str[32];
    char buf[512];
    char sign;
    uint32_t data;

    if (file->status != FR_OK)
    {
        return;
    }

    sprintf(buf, "%06ld     ", ++file->numRecords);

    sprintf(str, "%010ld     ", (uint32_t)(time/1000));
    strcat(buf, str);

    //vBat
    data = (uint32_t)(fltData[1] * 10);
    sprintf(str, "%2lu.%1lu     ", data/10, data % 10);
    strcat(buf, str);

    //iBat
    data = (uint32_t)(fltData[0] * 100);
    sprintf(str, "%2lu.%02lu     ", data/100, data % 100);
    strcat(buf, str);

    //Capacity mAh
    data = (uint32_t)(fabs(fltData[2]) * 10);
    sprintf(str, "%4lu.%1lu     ", data/10, data % 10);
    strcat(buf, str);

    //Left Motor
    data = (uint32_t)(fltData[9]);
    sprintf(str, "%4lu     ", data);
    strcat(buf, str);

    //Right Motor
    data = (uint32_t)(fltData[10]);
    sprintf(str, "%4lu     ", data);
    strcat(buf, str);

    //PID P
    data = (uint32_t)(fabs(fltData[6]) * 10);
    sign = (fltData[6] < 0.0f) ? '-' : ' ';
    sprintf(str, "%c%3lu.%1lu     ", sign, data/10, data % 10);
    strcat(buf, str);

    //PID I
    data = (uint32_t)(fabs(fltData[7]) * 10);
    sign = (fltData[7] < 0.0f) ? '-' : ' ';
    sprintf(str, "%c%3lu.%1lu     ", sign, data/10, data % 10);
    strcat(buf, str); 

    //PID D
    data = (uint32_t)(fabs(fltData[8]) * 10);
    sign = (fltData[8] < 0.0f) ? '-' : ' ';
    sprintf(str, "%c%3lu.%1lu     ", sign, data/10, data % 10);
    strcat(buf, str);

    //PID Error
    data = (uint32_t)(fabs(fltData[5]) * 10);
    sign = (fltData[5] < 0.0f) ? '-' : ' ';
    sprintf(str, "%c%2lu.%1lu     ", sign, data/10, data % 10);
    strcat(buf, str);

    //Roll
    data = (uint32_t)(fabs(fltData[4]) * 10);
    sign = (fltData[4] < 0.0f) ? '-' : ' ';
    sprintf(str, "%c%2lu.%1lu     ", sign, data/10, data % 10);
    strcat(buf, str);

    sprintf(str, "\r\n");
    strcat(buf, str);

    uint32_t bytesWrote;
    file->status = f_write(&file->fil, buf, strlen(buf), (UINT*)&bytesWrote);
}

/**
  * @brief  Sync file
  * @retval None
  */
static void FileSync(file_t* file)
{
    if (file->status == FR_OK)
    {
        file->status = f_sync(&file->fil);
    }
}

/**
  * @brief  Key handler
  * @retval None
  */
static void Text_Scr_Key_Handler(key_state_e state)
{
    if (state == PRESSED)
    {
        if (++txt_scr_id >= MAX_TXT_SCR)
        {
                txt_scr_id = 0;
        }

        ILI9341_DrawFillRectangle(text_wnd.left, text_wnd.top, text_wnd.right, text_wnd.bottom, Black);
    }
}

/**
  * @brief  KeyPad handler
  * @retval None
  */
static void KeyPad_Handler()
{
    //uint16_t key_val = KeyPad_GetVal();

    Print_Screen_2();

    KeyPad_Release();
}