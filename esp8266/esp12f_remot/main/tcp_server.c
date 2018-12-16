/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */
#include <string.h>
#include <time.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "lwip/apps/sntp.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "spiffs.h"
#include "spiffs_nucleus.h"
#include "esp_partition.h"
#include "esp_err.h"
#include "ssd1306.h"
#include "font.h"


#define SW_VER "<br>v1.2"
#define GPIO_OUTPUT_IO_0    2
#define GPIO_OUTPUT_IO_1    16
#define GPIO_OUTPUT_IO_2    15
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<GPIO_OUTPUT_IO_2))

#define GPIO_INPUT_IO_0     0
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)


#define PORT 80
#define BASICSTR "HTTP/1.1 200 OK\r\nCache-Control: no-cache, private\r\nContent-Length: 94\r\n\r\n"
#define PARAMLEN 32
#define MEXOUTLEN 94//size del messaggio di risposta html
#define GPIOSTARTUTSECTOR   213 //settore nel quale vengono scritti i valori dello startuo delle gpio
#define WIFISSIDSECOTR      214 //settore nel quale viene scritto l'SSID secondario
#define WIFIKEYSECTOR       215 //settore nel quale viene scritta la LEY delll'SSID secondario
#define WIFISSIDCHOICE      216 //settore nel quale viene scritto se utilizzare SSID embedded o su flash
#define GPIOOUTPUTNUM 2//numero delle GPIO configurate come output
#define TAGNUM 13//numero di TAG registrati
#define LENTAG 5//len dei tag
#define KEEPALIVETICK 1000//timer per invio pacchetto UDP keep alive per WiFi circa ogni 50 secondi
#define WIFITICKTRIGGERCHANGE 5000//tempo oltre il quale cambia l'host da primario a secondario

#define I2C_EXAMPLE_MASTER_SCL_IO           5                /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO           4                /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */



int ScanEnd=0;
int WiFiSw=0;
char  WIFI_SSID[32]= "osp";
char  WIFI_PASS[32]= "pippopippo";
uint32_t mainkeepalivecount=0;
char param[PARAMLEN+1]="";//parametri passati con i vari comandi
static EventGroupHandle_t wifi_event_group;
char MexOut[MEXOUTLEN+1]="";
const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;
int ERRORS=0;//controllo gli errori globali per decidere il riavvio
int EXECUTECMD=-1;//comando da eseguire
int RUNNING=0;//comando da eseguire
uint8_t gpiostatus[GPIOOUTPUTNUM]={'0','0'};//memorizzo lo stato delle GPIO
static const char *TAG = "FraAANT";//tag di debug
char TAGS[TAGNUM][6] = {
    "/cmd0",//0  -todo- 
    "/cmd1",//1  spegni gpio1    
    "/cmd2",//2  accendi gpio1
    "/cmd3",//3  spegni gpio2
    "/cmd4",//4  accendi gpio2
    "/cmd5",//5
    "/cmd6",//6
    "/sidk",//7 setto la key per l'SSID secondario
    "/sidn",//8 setto SSIS name secondario
    "/gpsv",//9  setta i valori allo startup delle gpio "/gpsv?10" (prima on seconda off)
    "/info",//10 info gpio status
    "/rst0",//11 reboot
    "/rst1" //12 erase settings and reboot
};//Tag GET http, ovvero dopo lo '/'



#define MAX_APs 20

//nfinite task
void loop_task(void *pvParameter)
{
    while(1) { 
              vTaskDelay(1000 / portTICK_RATE_MS);       
    }
}


static char* getAuthModeName(wifi_auth_mode_t auth_mode) {
       
       char *names[] = {"OPEN", "WEP", "WPA PSK", "WPA2 PSK", "WPA WPA2 PSK", "MAX"};
       return names[auth_mode];
}





static xQueueHandle gpio_evt_queue = NULL;

static void gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
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


void ssd1306_init() {
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
    i2c_master_write_byte(cmd, 0x14, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

    i2c_master_write_byte(cmd, 0XD9, true);
    i2c_master_write_byte(cmd, 0XF1, true);
    i2c_master_write_byte(cmd, 0XDB, true);
    i2c_master_write_byte(cmd, 0X40, true);
    i2c_master_write_byte(cmd, 0XA6, true);
    i2c_master_write_byte(cmd, 0X2E, true);
    i2c_master_write_byte(cmd, 0XA4, true);
    i2c_master_write_byte(cmd, 0XD3, true);
    i2c_master_write_byte(cmd, 0X00, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);

    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        ESP_LOGI("fraant", "OLED configured successfully");
    } else {
        ESP_LOGE("fraant", "OLED configuration failed. code: 0x%.2X", espRc);
    }
    i2c_cmd_link_delete(cmd);
}


void task_ssd1306_display_text(void *arg_text) 
{
    char *text = (char*)arg_text;
    i2c_cmd_handle_t cmd;

    uint8_t cur_page = text[0]-48;text++;
    uint8_t text_len = strlen(text);
    if (text_len>16){text_len=16;}
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, 0x00, true);             // reset column
    i2c_master_write_byte(cmd, 0x10, true);             // reset column
    i2c_master_write_byte(cmd, 0xB0 + cur_page, true);  // reset page

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
 
    for(int i=0;i<text_len;i++)
        {
         i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);
        }
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelete(NULL);
}

void task_ssd1306_display_clear(void *ignore) 
    {
    i2c_cmd_handle_t cmd;

    uint8_t zero[128];memset(zero,0x00,128);
    for (uint8_t i = 0; i < 8; i++) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
        i2c_master_write_byte(cmd, 0xB0 + i, true);

        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
        i2c_master_write(cmd, zero, 128, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }

    vTaskDelete(NULL);
    }



//UdpKeepAlive
static void FrKeepAlive(void* pvParameters)
    {
    mainkeepalivecount++;
    int addr_family;
    int ip_protocol;
    char payload[]="http1.0";
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = inet_addr("8.8.8.8");
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(80);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) 
        {
        ESP_LOGE(TAG, "FR_keep Alive Unable to create socket: errno %d", errno);
        ERRORS=1;
        goto fine1;
        }
    //ESP_LOGI(TAG, "FrKeepalive - Socket OK");
    int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err < 0) 
        {
        ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
        ERRORS=1;
        goto fine1;
        }  
    ESP_LOGI(TAG, "FrKeepAlive Message sent n.:%d",mainkeepalivecount);

fine1:
    vTaskDelay(100 / portTICK_PERIOD_MS);
    shutdown(sock, 0);
    close(sock);
    vTaskDelete(NULL);
    }

//legge nella nand il valore dell'SSID secondare
int readSSIDchoice()
    {
    uint32_t data[1];
    int resu=spi_flash_read(WIFISSIDCHOICE*4*1024, data, 4);
    if(resu<0)
        {
        ESP_LOGI(TAG, "Flash read ssidi selection error:%d",resu);
        return -1;
        }
    ESP_LOGI(TAG, "Flash read ssid switch:%d",data[0]);
    if(data[0]==12345678){return 1;}
    if(data[0]==12345679){return 2;}
    return 0;
    }

//scrive nella nand il valore dell'SSID secondare (max 30 caratteri)
int writeSSIDchoice(uint8_t key)
    {
    ESP_LOGI(TAG, "Flash Write ssid keswitch:%d",key);
    uint32_t data[1];data[0]=1;
    if(key==1){data[0]=12345678;}
    if(key==2){data[0]=12345679;}
    spi_flash_erase_sector(WIFISSIDCHOICE);
    return spi_flash_write(WIFISSIDCHOICE*4*1024, data, 4);
    }


//legge nella nand il valore dell'SSID secondare
int readSSIDkey(char * data)
    {
    int resu=spi_flash_read(WIFIKEYSECTOR*4*1024, data, 32);
    if(resu<0)
        {
        ESP_LOGI(TAG, "Flash read ssidi key error:%d",resu);
        return resu;
        }
    if(data[31]>30){return -2;}
    data[(uint8_t)(data[31])]=0;//taglio la stringa
    ESP_LOGI(TAG, "Flash read ssid key:%s len:%d",data,data[31]);
    return data[31];
    }

//scrive nella nand il valore dell'SSID secondare (max 30 caratteri)
int writeSSIDkey(char * key)
    {
    key[30]=0;//nel dubbio taglio
    ESP_LOGI(TAG, "Flash Write ssid key:%s",key);
    char data[32]="";strcpy(data,key);
    data[31]=strlen(key);//in coda la len effettiva
    spi_flash_erase_sector(WIFIKEYSECTOR);
    return spi_flash_write(WIFIKEYSECTOR*4*1024, data, 32);
    }

//legge nella nand il valore dell'SSID secondare
int readSSIDname(char * data)
    {
    int resu=spi_flash_read(WIFISSIDSECOTR*4*1024, data, 32);
    if(resu<0)
        {
        ESP_LOGI(TAG, "Flash read ssid error: %d",resu);
        return resu;
        }

    if(data[31]>30){return -2;}
    data[(uint8_t)(data[31])]=0;//taglio la stringa
    ESP_LOGI(TAG, "Flash read ssid:%s len:%d",data,data[31]);
    return data[31];
    }

//scrive nella nand il valore dell'SSID secondare (max 30 caratteri)
int writeSSIDname(char * ssid)
    {
    ssid[30]=0;//nel dubbio taglio
    ESP_LOGI(TAG, "Flash Write ssid:%s",ssid);
    char data[32]="";strcpy(data,ssid);
    data[31]=strlen(ssid);//in coda la len effettiva
    spi_flash_erase_sector(WIFISSIDSECOTR);
    return spi_flash_write(WIFISSIDSECOTR*4*1024, data, 32);
    }

//scrive nella nand il valore delle GPIO allo startup
int writeGPioStartupValue(uint8_t * gpio)
    {
    ESP_LOGI(TAG, "Flash Write GpioStValue: %d %d",gpio[0],gpio[1]);
    uint32_t data[GPIOOUTPUTNUM];
    data[0]=gpio[0]; 
    data[1]=gpio[1]; 
    spi_flash_erase_sector(GPIOSTARTUTSECTOR);
    return spi_flash_write(GPIOSTARTUTSECTOR*4*1024, data, GPIOOUTPUTNUM*4);
    }

//scrive nella nand il valore delle GPIO allo startup
int readGPioStartupValue(uint8_t *gpio)
    {
    uint32_t data[GPIOOUTPUTNUM];
    int resu=spi_flash_read(GPIOSTARTUTSECTOR*4*1024, data, GPIOOUTPUTNUM*4);
    if(resu<0){return resu;}
    gpio[0]=(uint8_t)data[0];
    gpio[1]=(uint8_t)data[1];
    if(gpio[0]>49){gpio[0]='0';}
    if(gpio[1]>49){gpio[1]='0';}
    if(gpio[0]<48){gpio[0]='0';}
    if(gpio[1]<48){gpio[1]='0';}
    ESP_LOGI(TAG, "\nFlash Read GpioStValue: %d %d",gpio[0],gpio[1]);
    return 0;
    }

static void get_time()
{
    struct timeval now;
    int sntp_retry_cnt = 0;
    int sntp_retry_time = 0;

    sntp_setoperatingmode(0);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    while (1) 
        {
        for (int32_t i = 0; (i < (SNTP_RECV_TIMEOUT / 100)) && now.tv_sec < 1525952900; i++) 
            {
            vTaskDelay(100 / portTICK_RATE_MS);
            gettimeofday(&now, NULL);
            }

        if (now.tv_sec < 1525952900) 
            {
            sntp_retry_time = SNTP_RECV_TIMEOUT << sntp_retry_cnt;

            if (SNTP_RECV_TIMEOUT << (sntp_retry_cnt + 1) < SNTP_RETRY_TIMEOUT_MAX) 
                {
                 sntp_retry_cnt ++;
                }

            printf("SNTP get time failed, retry after %d ms\n", sntp_retry_time);
            vTaskDelay(sntp_retry_time / portTICK_RATE_MS);
            } 
        else 
            {
             printf("SNTP get time success\n");
             break;
            }
        }
 }




//mia funzione instring
int fr_instring(uint32_t posizione_partenza,char*buffer,uint32_t dim_b,char*tag,uint32_t dim_t)
    {
    int conta=0;
    for(int i=posizione_partenza;i<dim_b;i++)
        {
        if(tag[conta]==buffer[i])
            {
            conta++;
            if(conta==dim_t)
                {
                return (i-dim_t);
                }
            }
        else
            {
            conta=0;
            }

        }

    return -1;
    }


//cerco uno dei tag nella riga di ricerca
int fr_findtag(char*buffer,int lenb)
    {
    const int PosTag=3;
    for(int i=0;i<TAGNUM;i++)
        {
        int res4=fr_instring(PosTag,buffer,lenb,TAGS[i],strlen(TAGS[i]));
        if(res4==PosTag)
            {
            //ora carichiamo i parametri se presenti
            memset(param,0,PARAMLEN);//azzero il buffer
            for(int p=0;p<PARAMLEN;p++)
                {
                if(buffer[PosTag+LENTAG+p+2]>32)
                    {
                    param[p]=buffer[PosTag+LENTAG+p+2];
                    }
                else
                    {
                    break;
                    }
                }
            if(buffer[PosTag+LENTAG+1]!='?'){param[0]=0;}//se non troviamo il '?' dopo il tag ignoriamo il parametro 
            ESP_LOGI(TAG, "\nCMD: %d  param: %s",i,param);
            return i;
            }        
        }

    return -1;
    }



const int WIFI_CONNECTED_BIT = BIT0;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        if (WiFiSw==1){esp_wifi_connect();}
        break;

    case SYSTEM_EVENT_SCAN_DONE:
         ScanEnd=1;
         break;

    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",MAC2STR(event->event_info.sta_connected.mac),event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",MAC2STR(event->event_info.sta_disconnected.mac),event->event_info.sta_disconnected.aid);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}


static void initialise_wifi(void)
    {
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    //ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config;
    strcpy((char*)wifi_config.sta.ssid,WIFI_SSID); 
    strcpy((char*)wifi_config.sta.password,WIFI_PASS); 
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    }

int wait_for_ip()
    {
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;

    ESP_LOGI(TAG, "Waiting for AP connection...");
    int resu=xEventGroupWaitBits(wifi_event_group, bits, false, false, WIFITICKTRIGGERCHANGE);//fraant modded, mettendo false, aspetta un solo bit di conferma, quindi ipv4 o ipv6, con ap che non hanno ipv6 rimaneva il loop, in attesa dell'ip ipv6
    //xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP, %d ,%d",resu,bits);
    return resu;
    }



static void tcp_server_task(void *pvParameters)
    {
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    char repoht[512]="";
    char spaces[MEXOUTLEN+1]="";memset(spaces,32,MEXOUTLEN);spaces[MEXOUTLEN]=0;strcpy(repoht,BASICSTR);strcat(repoht,spaces);//preparo http out
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        goto quitta;
    }
    //        ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto quitta;
    }
    //        ESP_LOGI(TAG, "Socket binded");

    while (1) {
        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
        uint addrLen = sizeof(sourceAddr);
        int sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        ESP_LOGI(TAG, "Socket accepted");

        while (1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occured during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Connection closed
            else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (sourceAddr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (sourceAddr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(sourceAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                //ESP_LOGI(TAG, "%s", rx_buffer);

                //controllo se ho ricevuto il TAG atteso
                int resfi=fr_findtag(rx_buffer,len);

                if(resfi>-1)
                    {
                    ESP_LOGI(TAG, "TAG found: %s:    pos: %d", TAGS[resfi],resfi);
                    EXECUTECMD=resfi;
                    }
                else
                    {
                    ESP_LOGI(TAG, "TAG NOT Found");
                    }


                vTaskDelay(750/portTICK_PERIOD_MS);//tempo per l'esecuzione del comando e riempimento della risposta http
                int aa=strlen(repoht)-MEXOUTLEN;
                for(int i=0;i<(MEXOUTLEN-1);i++){repoht[aa+i]=32;}//riazzero a tuttti spazi
                for(int i=0;i<strlen(MexOut);i++){repoht[aa+i]=MexOut[i];}
                int err = send(sock, repoht, strlen(repoht), 0);
                //ESP_LOGE(TAG, "\nsent byte %d %d repo:%s", err,resa,repobuff);
                if (err < 0) 
                    {
                    ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                    break;
                    }

                vTaskDelay(500/portTICK_PERIOD_MS);//chiudo la socket
                shutdown(sock, 0);
                close(sock);
            }
        }

        if (sock != -1) 
            {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
            vTaskDelay((100/portTICK_PERIOD_MS));
            }
    }

quitta:
    vTaskDelete(NULL);
    ERRORS=1;
    }



//eseguo i comandi una alla volta come TASK
static void exec_task(void *pvParameters)
    {
    struct timeval tv;
    time_t nowtime;
    struct tm *nowtm;
    char tmbuf[32]="";

    gettimeofday(&tv, NULL);
    nowtime = tv.tv_sec;
    nowtm = localtime(&nowtime);
    strftime(tmbuf,sizeof(tmbuf),"%Y/%m/%d-%H:%M:%S",nowtm);

    strcpy(MexOut,tmbuf);

    ESP_LOGI(TAG, "%s Eseguo Comando TASK: %d",tmbuf,RUNNING);


    if(RUNNING==0)
        {
        //todo
        }
    if(RUNNING==1)
        {
        gpio_set_level(GPIO_OUTPUT_IO_0, 0);
        ESP_LOGI(TAG, "Eseguo GPIO %d 0",GPIO_OUTPUT_IO_0);
        gpiostatus[0]='0';
        strcat(MexOut," accesa GPio1");
        }
    if(RUNNING==2)
        {
        gpio_set_level(GPIO_OUTPUT_IO_0, 1);
        ESP_LOGI(TAG, "Eseguo GPIO  %d 1",GPIO_OUTPUT_IO_0);
        gpiostatus[0]='1';
        strcat(MexOut," spenta GPio1");
        }
    if(RUNNING==3)
        {
        gpio_set_level(GPIO_OUTPUT_IO_1, 0);
        ESP_LOGI(TAG, "Eseguo GPIO %d 0",GPIO_OUTPUT_IO_1);
        gpiostatus[1]='0';
        strcat(MexOut," accesa GPio2");
        }
    if(RUNNING==4)
        {
        gpio_set_level(GPIO_OUTPUT_IO_1, 1);
        ESP_LOGI(TAG, "Eseguo GPIO  %d 1",GPIO_OUTPUT_IO_1);
        gpiostatus[1]='1';
        strcat(MexOut," spenta GPio2");
        }

    if(RUNNING==7)//setto key ssid secondario
        {
        ESP_LOGI(TAG, "Eseguo set SSID key %s",param);
        writeSSIDkey(param);
        strcat(MexOut," SSID key stored");
        }
    if(RUNNING==8)//setto SSID secondario
        {
        ESP_LOGI(TAG, "Eseguo set SSID name %s",param);
        writeSSIDname(param);
        strcat(MexOut," SSID stored");
        }

    if(RUNNING==9)//set startup gpioValue
        {
        uint8_t gpioV[2]={'0','0'};
        if (param[0]=='1'){gpioV[0]='1';}
        if (param[1]=='1'){gpioV[1]='1';}

        writeGPioStartupValue(gpioV);

        ESP_LOGI(TAG, "Eseguo GPIO set StartupValue  %d %d ",gpioV[0],gpioV[1]);
        strcat(MexOut,"StartUp Value Acquired");
        }

    if(RUNNING==10)//info
        {
        char keepc[10]="";
        itoa(mainkeepalivecount,keepc,10);
        ESP_LOGI(TAG, "Eseguo info");
        strcat(MexOut,"<br>Status GPIO1=_ GPIO2=_ <br>KeepAlive count: ");
        strcat(MexOut,keepc);
        strcat(MexOut,SW_VER);
        MexOut[36]=gpiostatus[0];
        MexOut[44]=gpiostatus[1];
        }


    if(RUNNING==11)//reset
        {
        ESP_LOGI(TAG, "Eseguo reset!");
        strcpy(MexOut,"addio puerco!");
        vTaskDelay((1500 / portTICK_PERIOD_MS));
        esp_restart();//riavvia tutto se qualcosa non va, qui non dovrebbe mai arrivare
        }

    if(RUNNING==12)//reset and flash erase
        {
        ESP_LOGI(TAG, "Eseguo ERASE flash e reset!");
        spi_flash_erase_sector(GPIOSTARTUTSECTOR);
        spi_flash_erase_sector(WIFISSIDSECOTR);
        spi_flash_erase_sector(WIFIKEYSECTOR);
        spi_flash_erase_sector(WIFISSIDCHOICE);
        strcpy(MexOut,"adios puerco!");
        vTaskDelay((1500 / portTICK_PERIOD_MS));
        esp_restart();//riavvia tutto se qualcosa non va, qui non dovrebbe mai arrivare
        }

    char mex2[96]="";
    strcpy(mex2,MexOut);
    char* aa=mex2;aa+=10;
    aa[0]='5';
    xTaskCreate(task_ssd1306_display_text, "ssd1306_display_text",1024,aa,6,NULL);



    RUNNING=0;
    vTaskDelete(NULL);

    }


static void gpio_task_example(void *arg)
{
    uint32_t io_num;

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) 
            {
            uint8_t st=gpio_get_level(io_num);
            ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", io_num, st);
            if((st==1)&&(io_num==GPIO_INPUT_IO_0))
                {
                if(gpiostatus[0]=='0'){RUNNING=2;}else{RUNNING=1;}//scehdula cambio della GPIO
                xTaskCreate(exec_task, "tcp_exec", 4096, NULL, 5, NULL);
                }
            }
    }
}

void app_main()
    {
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    // configure, initialize and start the wifi driver
    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // configure and run the scan process in blocking mode
    wifi_scan_config_t scan_config = 
        {
         .ssid = 0,
         .bssid = 0,
         .channel = 0,
         .show_hidden = true
        };

    printf("Start scanning...");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
    printf(" completed!\n");

    // get the list of APs found in the last scan
    uint16_t ap_num = MAX_APs;
    wifi_ap_record_t ap_records[MAX_APs];
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));


    // print the list 
    printf("Found %d access points:\n", ap_num);
    printf("\n");
    printf("               SSID              | Channel | RSSI |   Auth Mode \n");
    printf("----------------------------------------------------------------\n");
    for(int i = 0; i < ap_num; i++)
           printf("%32s | %7d | %4d | %12s\n", (char *)ap_records[i].ssid, ap_records[i].primary, ap_records[i].rssi, getAuthModeName(ap_records[i].authmode));
    printf("----------------------------------------------------------------\n");

    while (ScanEnd==0)
        {
         vTaskDelay((50 / portTICK_PERIOD_MS));
        }
    esp_wifi_deinit();
    esp_wifi_stop();
    WiFiSw=1; 

    uint8_t gpioV[2];//valore di startup delle gpio
    uint32_t conta=0;
    char buff[32]="";
    snprintf(buff,18,"6n.%d AP found!",ap_num);

    i2c_master_init();
    ssd1306_init();  
    xTaskCreate(task_ssd1306_display_clear, "ssd1306_display_clear", 1024, NULL, 6, NULL);
    vTaskDelay((450 / portTICK_PERIOD_MS));
    xTaskCreate(task_ssd1306_display_text, "ssd1306_dt2",  1024,(char *)"0   FraAnt 2018  ", 6, NULL);
    xTaskCreate(task_ssd1306_display_text, "ssd1306_dt1",  1024,buff, 6, NULL);


    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);



    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *) GPIO_INPUT_IO_0);


    //setto gli startup Value 
    if(readGPioStartupValue(gpioV)==0)
        {
        gpio_set_level(GPIO_OUTPUT_IO_1, gpioV[0]-48);
        gpio_set_level(GPIO_OUTPUT_IO_0, gpioV[1]-48);
        gpiostatus[0]=gpioV[0];
        gpiostatus[1]=gpioV[1];
        }

    int swi=readSSIDchoice();
    if(swi==2)//leggo le chiavi alternative
        {
        if(readSSIDname(buff)>0)
            {
            if(readSSIDkey(buff)>0)
                {//solo se sono caricati tutti e due li prendo per buoni
                strcpy(WIFI_PASS,buff);
                readSSIDname(buff);
                strcpy(WIFI_SSID,buff);
                ESP_LOGI(TAG, "Using stored WiFi credentials");        
                }
            }
        }

    char pwdout[18]="";
    snprintf(pwdout,17,"4%s %s",WIFI_PASS,WIFI_SSID);
    xTaskCreate(task_ssd1306_display_text, "ssd1306_display_text",  2048,pwdout, 6, NULL);


    initialise_wifi();

    int reswifi=wait_for_ip();

    if(reswifi==0)
        {
        if(swi==2){writeSSIDchoice(1);}
        if(swi==1){writeSSIDchoice(2);}
        if(swi<1){writeSSIDchoice(1);}//se non esiste uso le interne
        esp_restart();
        }

    get_time();


    uint8_t mac[6];
    esp_wifi_get_mac(ESP_IF_WIFI_AP, mac);
    char macout[20];
    snprintf(macout,19,"7MAC:%02x%02x%02x%02x%02x%02x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    ESP_LOGI(TAG, "\nPrinting Mac onto Oled:%s",macout);        
    xTaskCreate(task_ssd1306_display_text, "ssd1306_display_text",  2048,macout, 6, NULL);

    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);

    struct timeval tv;
    time_t nowtime;
    struct tm *nowtm;
    char tmbuf[32]="";


    //thread principale esecuzione comandi e controllo errori
    while (ERRORS==0)
        {
        vTaskDelay((50 / portTICK_PERIOD_MS));

        conta++;
        if(conta%20==0)
            {
            gettimeofday(&tv, NULL);
            nowtime = tv.tv_sec;
            nowtm = localtime(&nowtime);
            strftime(tmbuf,sizeof(tmbuf),"6%H:%M:%S - %d/%m",nowtm);
            xTaskCreate(task_ssd1306_display_text, "ssd1306_display_text",  2048,tmbuf, 6, NULL);
            }

        if(conta==KEEPALIVETICK)
            {
            conta=0;
            xTaskCreate(FrKeepAlive, "udp_keepalive", 4096, NULL, 5, NULL);
            }

        if(EXECUTECMD>-1)
            {
            ESP_LOGI(TAG, "Eseguo Comando: %d",EXECUTECMD);
            if(RUNNING==0)
                {
                RUNNING=EXECUTECMD;
                xTaskCreate(exec_task, "tcp_exec", 4096, NULL, 5, NULL);
                }
            EXECUTECMD=-1;
            }
        }
    esp_restart();//riavvia tutto se qualcosa non va, qui non dovrebbe mai arrivare
    }
