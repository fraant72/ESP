#include <string.h>
#include <time.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "esp_err.h"
#include "ssd1306.h"
#include "oled.h"
#include "oledfont.h"
#include "oledimg.h"

#define VideoMemSize 1024
#define OLED_LCDWIDTH 128
#define OLED_LCDHEIGHT 64
#define OledWriteForceRawCicle 16



i2c_cmd_handle_t cmd;

static uint8_t videomem[1024];
uint8_t GetBit (uint8_t Valore, uint8_t posiz)
{       
    switch (posiz)
    {
         case 0: 
             if ((Valore & 0x01)==0){return 0; }else {return 1;};
                
         case 1: 
             if ((Valore & 0x02)==0){return 0; }else {return 1;};
             
         case 2: 
             if ((Valore & 0x04)==0){return 0; }else {return 1;};

         case 3: 
             if ((Valore & 0x08)==0){return 0; }else {return 1;};
             
         case 4: 
             if ((Valore & 0x10)==0){return 0; }else {return 1;};
   
         case 5: 
             if ((Valore & 0x20)==0){return 0; }else {return 1;};
                
         case 6: 
             if ((Valore & 0x40)==0){return 0; }else {return 1;};
                
         case 7: 
             if ((Valore & 0x80)==0){return 0; }else {return 1;};
    } 
    return 0;
}

void OLEDCommand (uint8_t c)
{
i2c_master_write_byte(cmd,c,true);
}
    
void i2c_init(void)
{
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
}

void i2c_end(void)
{
    esp_err_t espRc;
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
       // ESP_LOGI("fraant", "OLED configured successfully");
    } else {
        ESP_LOGE("fraant", "OLED configuration failed. code: 0x%.2X", espRc);
    }
    i2c_cmd_link_delete(cmd);

}

void OLEDPartInit(void)
{
    OLEDCommand ( OLED_CONTROL_BYTE_CMD_STREAM);//set lower column address.
    OLEDCommand ( OLED_LOW_COLUMN_START );      //set lower column address.
    OLEDCommand ( OLED_HIGH_COLUMN_START );     //set higher column address.
    OLEDCommand ( OLED_PAGE_START );            //set page address.
}

void i2c_master_init()
    {
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 4;
    conf.sda_pullup_en = 0;
    conf.scl_io_num = 5;
    conf.scl_pullup_en = 0;
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    }


void OLEDStopScroll ( void )
{
    OLEDCommand ( OLED_DEACTIVATE_SCROLL );
}

void OLEDInit() 
{
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
// 1. Fundamental Commands.
    OLEDCommand ( OLED_CONTRAST );              // 0x81
    OLEDCommand ( 0x4F );                       // was 0x9F. ******
    OLEDCommand ( OLED_DISPLAY_ALL_ON_RESUME ); // 0xA5
    OLEDCommand ( OLED_NORMAL_DISPLAY );        // 0xA6
    OLEDCommand ( OLED_DISPLAY_ON );            // 0xAF

// 2. Scrolling.

// 3. Addressing.
    OLEDCommand ( OLED_MEMORY_MODE );           // 0x20
    OLEDCommand ( OLED_HORIZONTAL_MODE );       
        
// 4. Hardware Configuration.
    OLEDCommand ( OLED_SET_START_LINE | 0x00 ); // line #0
    OLEDCommand ( OLED_SEGMENT_REMAP | 0x01 );  // --------------------PREVIOUS!!!
    //OLEDCommand ( OLED_SEGMENT_REMAP | 0x00 );      // MODIFY IT TO MAP segment 0 to columns address 0
    OLEDCommand ( OLED_SET_MULTIPLEX_RATIO );   // 0xA8
    OLEDCommand ( 0x3F );
    OLEDCommand ( OLED_COM_SCAN_DECREMENT );    //--------------> it writes from COM[N-1] to COM0
    OLEDCommand ( OLED_DISPLAY_OFFSET );        // 0xD3
    OLEDCommand ( 0x00);                        // no offset
    OLEDCommand ( OLED_SET_COM_PINS );          // 0xDA
    OLEDCommand ( 0x12 );

// 5. Timing.
    OLEDCommand ( OLED_DISPLAY_CLOCK_RATIO );   // 0xD5
    OLEDCommand ( OLED_CLOCK_RESET_RATIO );
    OLEDCommand ( OLED_SET_PRECHARGE );         // 0xd9
    OLEDCommand ( OLED_PRECHARGE_P1_P2 );       // 0xF1
    OLEDCommand ( OLED_SET_VCOM_H );            // 0xDB
    OLEDCommand ( 0x40 );                       //

// 6?. Other 
    OLEDCommand ( OLED_CHARGE_PUMP );           // 0x8D
    OLEDCommand ( OLED_ENABLE_CHARGE );         // 0x14

    OLEDStopScroll ( ); //it deactivates scroll
}



void oledWriteVideoMem()
{
    i2c_init();
    OLEDInit();
    i2c_end();
    i2c_init();
    OLEDPartInit();
    i2c_end();
    i2c_init();
    i2c_master_write_byte(cmd,COMMAND_NEXT_BYTE_IS_DATA, true);
    i2c_master_write(cmd,videomem,1024,true); 
    i2c_end();
}

void ssd1306_display_clear(uint8_t write)
    {
    memset(videomem,0,1024);
    if(write>0){oledWriteVideoMem();}
    }

void OledDrawPixel(uint8_t x, uint8_t y) 
{
  if ((x >= OLED_LCDWIDTH) || (y >= OLED_LCDHEIGHT)){return;}
  videomem[x+ (y/8)*OLED_LCDWIDTH] |= (1 << y%8);
}


void OledClearPixel(uint8_t x, uint8_t y) 
{
  if ((x >= OLED_LCDWIDTH) || (y >= OLED_LCDHEIGHT)){return;}
  videomem[x+ (y/8)*OLED_LCDWIDTH] &= ~(1 << y%8); 
}

uint8_t OledGetPixel(uint8_t x, uint8_t y)
{
  if ((x >= OLED_LCDWIDTH) || (y >=OLED_LCDHEIGHT)) return 0;
  return videomem[x+ (y/8)*OLED_LCDWIDTH] & (1 << y%8) ? 1 : 0;
}

void OledShiftFramevideomem( uint8_t height)
{
  if (height == 0) return;

  uint8_t y, x;
  for (y = 0; y < OLED_LCDHEIGHT; y++)
  {
    for (x = 0; x < OLED_LCDWIDTH; x++)
    {
      if ((OLED_LCDHEIGHT - 1) - y > height)
      {
        OledGetPixel(x, y + height) ? OledDrawPixel(x, y) : OledClearPixel(x, y);
      }
      else
      {
         OledClearPixel(x, y);
      }
    }
  }
}




void OledDrawCircle(int16_t x0, int16_t y0, int16_t radius) 
{
    int16_t x = 0, y = radius;
    int16_t dp = 1 - radius;
    do {
        if (dp < 0)
            {dp = dp + 2 * (++x) + 3;}
        else
            {dp = dp + 2 * (++x) - 2 * (--y) + 5;}

        OledDrawPixel(x0 + x, y0 + y);     //For the 8 octants
        OledDrawPixel(x0 - x, y0 + y);
        OledDrawPixel(x0 + x, y0 - y);
        OledDrawPixel(x0 - x, y0 - y);
        OledDrawPixel(x0 + y, y0 + x);
        OledDrawPixel(x0 - y, y0 + x);
        OledDrawPixel(x0 + y, y0 - x);
        OledDrawPixel(x0 - y, y0 - x);

    } while (x < y);

    OledDrawPixel(x0 + radius, y0);
    OledDrawPixel(x0, y0 + radius);
    OledDrawPixel(x0 - radius, y0);
    OledDrawPixel(x0, y0 - radius);
}

void OledDrawHorizontalLine(int16_t x0, int16_t y0, int16_t lenght) 
{
if(y0>OLED_LCDHEIGHT){return;}
for(int i=0;i<lenght;i++)
    {
     int b=x0+i;
     if(b<OLED_LCDWIDTH){OledDrawPixel(b, y0);}
    }
}

void OledDrawVerticallLine(int16_t x0, int16_t y0, int16_t lenght) 
{
if(x0>OLED_LCDWIDTH){return;}
for(int i=0;i<lenght;i++)
    {
     int b=y0+i;
     if(b<OLED_LCDHEIGHT){OledDrawPixel(x0,b);}
    }
}



void OledDrawRect(int16_t x, int16_t y, int16_t width, int16_t height) 
{
  OledDrawHorizontalLine(x, y, width);
  OledDrawVerticallLine(x, y, height);
  OledDrawVerticallLine(x + width - 1, y, height);
  OledDrawHorizontalLine(x, y + height - 1, width);
}


void OledFillRect(int16_t xMove, int16_t yMove, int16_t width, int16_t height) 
{
  for (int16_t x = xMove; x < xMove + width; x++) 
      {
        OledDrawVerticallLine(x, yMove, height);
      }
}



void OledFillCircle(int16_t x0, int16_t y0, int16_t radius) 
{
    int16_t x = 0, y = radius;
    int16_t dp = 1 - radius;
    do {
        if (dp < 0)
            {dp = dp + 2 * (++x) + 3;}
        else
            {dp = dp + 2 * (++x) - 2 * (--y) + 5;}

    OledDrawHorizontalLine(x0 - x, y0 - y, 2*x);
    OledDrawHorizontalLine(x0 - x, y0 + y, 2*x);
    OledDrawHorizontalLine(x0 - y, y0 - x, 2*y);
    OledDrawHorizontalLine(x0 - y, y0 + x, 2*y);


    } while (x < y);
  OledDrawHorizontalLine(x0 - radius, y0, 2 * radius);

}

int AddBigi2NumtoOledVideoMem(int startx,int pagen,uint8_t symbol)
{
    //la size è 22 x 32
    if (symbol<48){return -1;}
    if (symbol>59){return -2;}
    if(pagen>4){return -3;}
    if(startx>(127-22)){return -4;}
    int posizmem=(symbol-48)*89+1;
    
//  fprintf(stderr,"\n font - posizmem:%d pagen:%d startx:%d symbol %d\n",posizmem,pagen,startx,symbol);    
    for(int i=0;i<22;i++)
        {
         videomem[(pagen*128)+i+startx]=Times_New_Roman22x32[posizmem+(i*4)];
         videomem[((pagen+1)*128)+i+startx]=Times_New_Roman22x32[posizmem+(i*4)+1];
         videomem[((pagen+2)*128)+i+startx]=Times_New_Roman22x32[posizmem+(i*4)+2];
         videomem[((pagen+3)*128)+i+startx]=Times_New_Roman22x32[posizmem+(i*4)+3];
        }
return 0;
}   

int AddBigNumtoOledVideoMem(int startx,int pagen,uint8_t symbol)
{
    //la size è 16 x 24
    if (symbol<48){return -1;}
    if (symbol>58){return -2;}
    if(pagen>5){return -3;}
    if(startx>(127-16)){return -4;}
    int posizmem=(symbol-48)*49+1;
    
    fprintf(stderr,"\n font - posizmem:%d pagen:%d startx:%d symbol %d\n",posizmem,pagen,startx,symbol);    
    for(int i=0;i<16;i++)
        {
         videomem[(pagen*128)+i+startx]=Times_New_Roman16x24[posizmem+(i*3)];
         videomem[((pagen+1)*128)+i+startx]=Times_New_Roman16x24[posizmem+(i*3)+1];
         videomem[((pagen+2)*128)+i+startx]=Times_New_Roman16x24[posizmem+(i*3)+2];
        }
return 0;
}   


int AddChartoOledVideoMem(int startx,int pagen,uint8_t symbol)
{
    //la size è 11X16
    if (symbol<32){return -1;}
    if (symbol>127){return -2;}
    if(pagen>6){return -3;}
    if(startx>(127-11)){return -4;}
    int posizmem=(symbol-32)*23+1;
    
    for(int i=0;i<11;i++)
        {
         videomem[(pagen*128)+i+startx]=Terminal11x16[posizmem+(i*2)];
         videomem[((pagen+1)*128)+i+startx]=Terminal11x16[posizmem+(i*2)+1];
        }
return 0;
}   


int AddMiniChartoOledVideoMem(int startx,int pagen,uint8_t symbol)
{
    //la size è 11X16
    if (symbol<32){return -1;}

    if(pagen>7){return -3;}
    if(startx>(127-6)){return -4;}
    int posizmem=(symbol-32)*7+1;

    for(int i=0;i<6;i++)
        {
         videomem[(pagen*128)+i+startx]=Terminal6x8[posizmem+i];
        }
return 0;
}



void WriteStringToOled(char* stringa, int startx, int pagen, int ClearAll,int WriteOut)
{
     if(ClearAll>0){memset(videomem,0,1024);}
     int charspace=12;
     for(int i=0;i<strlen(stringa);i++)
        {
          AddChartoOledVideoMem(startx+(i*charspace),pagen,stringa[i]);
        }
     if(WriteOut>0){oledWriteVideoMem();}
}




void WriteMiniStringToOled(char* stringa, int startx, int pagen, int ClearAll,int WriteOut)
{
     if(ClearAll>0){memset(videomem,0,1024);}
     int charspace=7;
     for(int i=0;i<strlen(stringa);i++)
        {
          AddMiniChartoOledVideoMem(startx+(i*charspace),pagen,stringa[i]);
        }
     if(WriteOut>0){oledWriteVideoMem();}
}


int loadico(uint8_t* address,uint8_t posx,uint8_t posy,uint8_t  sizex,uint8_t sizey,int totalsize,uint8_t reverse)
{
    if((sizex*sizey)!=totalsize){return -1;}
    if(posx+sizex>OLED_LCDWIDTH){return -2;}
    if(posy+sizey>OLED_LCDHEIGHT){return -3;}
    uint8_t check=1;if(reverse>0){check=0;}
    int conta=0;
    for(int y=0;y<sizey;y++)
        {
         for(int x=0;x<sizex;x++)
            {
            if(address[conta]==check)
                {
                OledDrawPixel(posx+x,posy+y);
                }
            conta++;            
            }
        }
return 0;
}

//carica un file .bmp risoluzione 128X64 monocromatico, direttamente e lo visualizza su oled
void loadbmp(uint8_t* address,int size,uint8_t reverse)
{

    if(size!=(1024+62)){return;}//mi attendo la size giusta
    memset(videomem,0,1024);
    address+=62;//levo l'header della bmp

    uint8_t check=1;if(reverse>0){check=0;} 
    for(int p=0;p<64;p++)//scorro le pagine
        {
         for (int a=0;a<16;a++)//scorro le righe 16x1Byte=128bit
            {
             uint8_t tmp=address[1023-(a+p*16)];
             for(int i=0;i<8;i++)
                {
                 if(GetBit(tmp,(i))==check)
                    {
                     OledDrawPixel((15-a)*8+(7-i),p);
                    }
                }
            }
        }
     
    //stampo la bmp
    oledWriteVideoMem();    
}

void oledImg1()
{
    memcpy(videomem,pollicealto,1024);
    oledWriteVideoMem();
}


void oledImg2()
{
    memcpy(videomem,sonno,1024);
    oledWriteVideoMem();
}


void oledImg3()
{
    memcpy(videomem,aquila,1024);
    oledWriteVideoMem();
}

void oledImg4()
{
    memcpy(videomem,startlogo,1024);
    oledWriteVideoMem();
}


