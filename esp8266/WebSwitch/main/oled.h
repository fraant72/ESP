#ifndef OLED_H_
#define OLED_H_

#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "ssd1306.h"

///////////////////////////////////////
//return codes
#define OLED_128_64_CORRECT_EXIT    0
#define OLED_128_64_GENERIC_ERROR   1
///////////////////////////////////////

///////////////////////////////////////
#define OLED_128_64_ADDRESS               0x78
#define OLED_128_64_ADDRESS_SECONDARY     0x7A
//#define OLED_128_64_READ        0x01
#define OLED_128_64_WRITE                 0x00
#define OLED_128_64_CO                    0x80
#define OLED_128_64_DC                    0x40
///////////////////////////////////////

///////////////////////////////////////
#define COMMAND_DATABYTES_ONLY_FOLLOWING            0x00
#define COMMAND_NEXT_BYTE_IS_COMMAND                0x00
#define COMMAND_NEXT_BYTE_IS_DATA                   0x40
///////////////////////////////////////

#define ARRAY_WIDHT 128
#define ARRAY_HEIGHT 8
#define ARRAY_LENGHT ( 8 * ARRAY_HEIGHT )

#define SSD1306_GDDRAM ( ARRAY_WIDHT * ARRAY_HEIGHT / 8 )


//#define WHITE 0x01
//#define BLACK 0x00
#define FILL 0x01
#define NO_FILL 0x00
#define CLEAR 0x01
#define NO_CLEAR 0x00

// 1. Fundamental Commands.
#define OLED_CONTRAST 0x81
#define OLED_DISPLAY_ALL_ON_RESUME 0xA4
#define OLED_DISPLAY_ALL_ON 0xA5
#define OLED_NORMAL_DISPLAY 0xA6
#define OLED_INVERTED_DISPLAY 0xA7
#define OLED_DISPLAY_OFF 0xAE
#define OLED_DISPLAY_ON 0xAF
// 2. Scrolling
#define OLED_RIGHT_HORIZONTAL_SCROLL 0x26
#define OLED_LEFT_HORIZONTAL_SCROLL 0x27
#define OLED_5_FRAMES 0x00
#define OLED_64_FRAMES 0x01
#define OLED_128_FRAMES 0x02
#define OLED_256_FRAMES 0x03
#define OLED_3_FRAMES 0x04
#define OLED_4_FRAMES 0x05
#define OLED_25_FRAMES 0x06
#define OLED_2_FRAMES 0x07
#define OLED_DUMMY_BYTE 0x00
#define OLED_VERTICAL_RIGHT 0x29
#define OLED_VERTICAL_LEFT 0x2A
#define OLED_DEACTIVATE_SCROLL 0x2E
#define OLED_ACTIVATE_SCROLL 0x2F
#define OLED_VERTICAL_SCROLL 0xA3

// 3. Addressing.
#define OLED_LOW_COLUMN_START 0x00
#define OLED_HIGH_COLUMN_START 0x10
#define OLED_MEMORY_MODE 0x20
#define OLED_HORIZONTAL_MODE 0x00
#define OLED_VERTICAL_MODE 0x01
#define OLED_PAGE_MODE 0x02
#define OLED_COLUMN_ADDRESS 0x21
#define OLED_PAGE_ADDRESS 0x22
#define OLED_PAGE_START 0xB0
// 4. Hardware Configuration.
#define OLED_SET_START_LINE 0x40
#define OLED_SEGMENT_REMAP 0xA0
#define OLED_SET_MULTIPLEX_RATIO 0xA8
#define OLED_COM_SCAN_INCREMENT 0xC0
#define OLED_COM_SCAN_DECREMENT 0xC8
#define OLED_DISPLAY_OFFSET 0xD3
#define OLED_SET_COM_PINS 0xDA
// 5. Timing.
#define OLED_DISPLAY_CLOCK_RATIO 0xD5
#define OLED_CLOCK_RESET_RATIO 0x80
#define OLED_SET_PRECHARGE 0xD9
#define OLED_PRECHARGE_RESET 0x22
#define OLED_PRECHARGE_P1_P2 0xF1
#define OLED_SET_VCOM_H 0xDB
#define OLED_VCC_RESET 0x20
// 6?. Other 
#define OLED_CHARGE_PUMP 0x8D
#define OLED_DISABLE_CHARGE 0x10
#define OLED_ENABLE_CHARGE 0x14

#define bitWise(n) ( 1 << n )
#define SIZE_ONE 0x01
#define SIZE_TWO 0x02
#define SIZE_THREE 0x03
#define DDGRAM_CLEAR 0x01
#define DDGRAM_NO_CLEAR 0x00

static const uint8_t light_10x12[]=
{
0,0,0,1,1,1,1,0,0,0,
0,0,1,0,0,0,0,1,0,0,
0,1,0,0,0,0,0,0,1,0,
1,0,0,0,0,0,0,0,0,1,
1,0,0,0,0,0,0,0,0,1,
1,0,0,0,0,0,0,0,0,1,
0,1,0,0,0,0,0,0,1,0,
0,1,0,0,0,0,0,0,1,0,
0,0,1,1,1,1,1,1,0,0,
0,0,1,0,0,0,0,1,0,0,
0,0,0,1,0,0,1,0,0,0,
0,0,0,0,1,1,0,0,0,0
};

void i2c_master_init();
void ssd1306_init();
void ssd1306_display_clear(uint8_t write);
void oledWriteVideoMem();


void OledDrawPixel(uint8_t x, uint8_t y);
void OledClearPixel(uint8_t x, uint8_t y);
uint8_t OledGetPixel(uint8_t x, uint8_t y);
void OledShiftFrameBuffer( uint8_t height);    
void OledDrawCircle(int16_t x0, int16_t y0, int16_t radius);
void OledDrawHorizontalLine(int16_t x0, int16_t y0, int16_t lenght);
void OledDrawVerticallLine(int16_t x0, int16_t y0, int16_t lenght);
void OledDrawRect(int16_t x, int16_t y, int16_t width, int16_t height);
void OledFillRect(int16_t xMove, int16_t yMove, int16_t width, int16_t height);
void OledFillCircle(int16_t x0, int16_t y0, int16_t radius);


int AddBigi2NumtoOledVideoMem(int startx,int pagen,uint8_t symbol);
int AddBigNumtoOledVideoMem(int startx,int pagen,uint8_t symbol);
int AddChartoOledVideoMem(int startx,int pagen,uint8_t symbol);
int AddMiniChartoOledVideoMem(int startx,int pagen,uint8_t symbol);

void WriteMiniStringToOled(char* stringa, int startx, int pagen, int ClearAll,int WriteOut);
void WriteStringToOled(char* stringa, int startx, int pagen, int ClearAll,int WriteOut);

void loadbmp(uint8_t* address,int sizei, uint8_t reverse);
int loadico(uint8_t* address,uint8_t posx,uint8_t posy,uint8_t  sizex,uint8_t sizey,int totalsize,uint8_t reverse);



void oledImg1();
void oledImg2();
void oledImg3();
void oledImg4();
void oledImg5();
void oledImg6();
void oledImg7();
void oledImg8();



extern const uint8_t Accept_bmp_start[] asm("_binary_Accept_bmp_start");
extern const uint8_t Angry_bmp_start[] asm("_binary_Angry_bmp_start");
extern const uint8_t Backward_bmp_start[] asm("_binary_Backward_bmp_start");
extern const uint8_t Bar0_bmp_start[] asm("_binary_Bar0_bmp_start");
extern const uint8_t Bar1_bmp_start[] asm("_binary_Bar1_bmp_start");
extern const uint8_t Bar2_bmp_start[] asm("_binary_Bar2_bmp_start");
extern const uint8_t Bar3_bmp_start[] asm("_binary_Bar3_bmp_start");
extern const uint8_t Bar4_bmp_start[] asm("_binary_Bar4_bmp_start");
extern const uint8_t Carro1_bmp_start[] asm("_binary_Carro1_bmp_start");
extern const uint8_t Carro2_bmp_start[] asm("_binary_Carro2_bmp_start");
extern const uint8_t Decline_bmp_start[] asm("_binary_Decline_bmp_start");
extern const uint8_t Dial0_bmp_start[] asm("_binary_Dial0_bmp_start");
extern const uint8_t Dial1_bmp_start[] asm("_binary_Dial1_bmp_start");
extern const uint8_t Dial2_bmp_start[] asm("_binary_Dial2_bmp_start");
extern const uint8_t Dial3_bmp_start[] asm("_binary_Dial3_bmp_start");
extern const uint8_t Dial4_bmp_start[] asm("_binary_Dial4_bmp_start");
extern const uint8_t Fire_bmp_start[] asm("_binary_Fire_bmp_start");
extern const uint8_t Forward_bmp_start[] asm("_binary_Forward_bmp_start");
extern const uint8_t Heartlarge_bmp_start[] asm("_binary_Heartlarge_bmp_start");
extern const uint8_t Heartsmall_bmp_start[] asm("_binary_Heartsmall_bmp_start");
extern const uint8_t Lightoff_bmp_start[] asm("_binary_Lightoff_bmp_start");
extern const uint8_t Lighton_bmp_start[] asm("_binary_Lighton_bmp_start");
extern const uint8_t Nogo_bmp_start[] asm("_binary_Nogo_bmp_start");
extern const uint8_t Pirate_bmp_start[] asm("_binary_Pirate_bmp_start");
extern const uint8_t Question_bmp_start[] asm("_binary_Question_bmp_start");
extern const uint8_t Right_bmp_start[] asm("_binary_Right_bmp_start");
extern const uint8_t Stop1_bmp_start[] asm("_binary_Stop1_bmp_start");
extern const uint8_t Thumbsdown_bmp_start[] asm("_binary_Thumbsdown_bmp_start");
extern const uint8_t Thumbsup_bmp_start[] asm("_binary_Thumbs up_bmp_start");
extern const uint8_t Timer0_bmp_start[] asm("_binary_Timer0_bmp_start");
extern const uint8_t Timer1_bmp_start[] asm("_binary_Timer1_bmp_start");
extern const uint8_t Timer2_bmp_start[] asm("_binary_Timer2_bmp_start");
extern const uint8_t Timer3_bmp_start[] asm("_binary_Timer3_bmp_start");
extern const uint8_t Timer4_bmp_start[] asm("_binary_Timer4_bmp_start");
extern const uint8_t Warning_bmp_start[] asm("_binary_Warning_bmp_start");
extern const uint8_t Wink_bmp_start[] asm("_binary_Wink_bmp_start");






extern const uint8_t Accept_bmp_end[] asm("_binary_Accept_bmp_end");
extern const uint8_t Angry_bmp_end[] asm("_binary_Angry_bmp_end");
extern const uint8_t Backward_bmp_end[] asm("_binary_Backward_bmp_end");
extern const uint8_t Bar0_bmp_end[] asm("_binary_Bar0_bmp_end");
extern const uint8_t Bar1_bmp_end[] asm("_binary_Bar1_bmp_end");
extern const uint8_t Bar2_bmp_end[] asm("_binary_Bar2_bmp_end");
extern const uint8_t Bar3_bmp_end[] asm("_binary_Bar3_bmp_end");
extern const uint8_t Bar4_bmp_end[] asm("_binary_Bar4_bmp_end");
extern const uint8_t Carro1_bmp_end[] asm("_binary_Carro1_bmp_end");
extern const uint8_t Carro2_bmp_end[] asm("_binary_Carro2_bmp_end");
extern const uint8_t Decline_bmp_end[] asm("_binary_Decline_bmp_end");
extern const uint8_t Dial0_bmp_end[] asm("_binary_Dial0_bmp_end");
extern const uint8_t Dial1_bmp_end[] asm("_binary_Dial1_bmp_end");
extern const uint8_t Dial2_bmp_end[] asm("_binary_Dial2_bmp_end");
extern const uint8_t Dial3_bmp_end[] asm("_binary_Dial3_bmp_end");
extern const uint8_t Dial4_bmp_end[] asm("_binary_Dial4_bmp_end");
extern const uint8_t Fire_bmp_end[] asm("_binary_Fire_bmp_end");
extern const uint8_t Forward_bmp_end[] asm("_binary_Forward_bmp_end");
extern const uint8_t Heartlarge_bmp_end[] asm("_binary_Heartlarge_bmp_end");
extern const uint8_t Heartsmall_bmp_end[] asm("_binary_Heartsmall_bmp_end");
extern const uint8_t Lightoff_bmp_end[] asm("_binary_Lightoff_bmp_end");
extern const uint8_t Lighton_bmp_end[] asm("_binary_Lighton_bmp_end");
extern const uint8_t Nogo_bmp_end[] asm("_binary_Nogo_bmp_end");
extern const uint8_t Pirate_bmp_end[] asm("_binary_Pirate_bmp_end");
extern const uint8_t Question_bmp_end[] asm("_binary_Question_bmp_end");
extern const uint8_t Right_bmp_end[] asm("_binary_Right_bmp_end");
extern const uint8_t Stop1_bmp_end[] asm("_binary_Stop1_bmp_end");
extern const uint8_t Thumbsdown_bmp_end[] asm("_binary_Thumbsdown_bmp_end");
extern const uint8_t Thumbsup_bmp_end[] asm("_binary_Thumbs up_bmp_end");
extern const uint8_t Timer0_bmp_end[] asm("_binary_Timer0_bmp_end");
extern const uint8_t Timer1_bmp_end[] asm("_binary_Timer1_bmp_end");
extern const uint8_t Timer2_bmp_end[] asm("_binary_Timer2_bmp_end");
extern const uint8_t Timer3_bmp_end[] asm("_binary_Timer3_bmp_end");
extern const uint8_t Timer4_bmp_end[] asm("_binary_Timer4_bmp_end");
extern const uint8_t Warning_bmp_end[] asm("_binary_Warning_bmp_end");
extern const uint8_t Wink_bmp_end[] asm("_binary_Wink_bmp_end");



#endif
