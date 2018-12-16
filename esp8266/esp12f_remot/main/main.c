#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "lwip/apps/sntp.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "mdns.h"

#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "main.h"
#include "oled.h"

static const char *TAG = "FraAANT";//tag di debug

// HTTP headers and web pages
const static char http_html_hdr[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";
const static char http_png_hdr[] = "HTTP/1.1 200 OK\nContent-type: image/png\n\n";

// embedded binary data
extern const uint8_t on_png_start[]      asm("_binary_on_png_start");
extern const uint8_t on_png_end[]        asm("_binary_on_png_end");
extern const uint8_t off_png_start[]     asm("_binary_off_png_start");
extern const uint8_t off_png_end[]       asm("_binary_off_png_end");
extern const uint8_t index_htm_start[]   asm("_binary_index_htm_start");
extern const uint8_t index_htm_end[]     asm("_binary_index_htm_end");
extern const uint8_t favicon_ico_start[] asm("_binary_favicon_ico_start");
extern const uint8_t favicon_ico_end[]   asm("_binary_favicon_ico_end");


// Event group for inter-task communication
static EventGroupHandle_t event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// actual relay status
bool relay_status1=false;
bool relay_status2=false;
int relay1on=0;int relay2on=0;//conto da quanti secondi consecutivi i relay sono accesi

static xQueueHandle gpio_evt_queue = NULL;

int ERRORS=0;
int mainkeepalivecount=0;
int seconds=0;
char my_ip[17]="";//ip testuale asssegnato
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


static void get_time()
{
    setenv("TZ", "CET-1", 1);//Italy Time Central Europe
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



static void gpio_isr_handler(void *arg)
    {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    }




static void gpio_task_example(void *arg)
{
    uint32_t io_num;

    for (;;) 
        {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) 
            {
            uint8_t st=gpio_get_level(io_num);
            ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", io_num, st);
            if((st==1)&&(io_num==GPIO_INPUT_IO_0))
                {
                 if(relay_status1 == false)
                    {
                     printf("Turning relay1_sw ON\n");
                     gpio_set_level(GPIO_OUTPUT_0, 1);
                     relay_status1 = true;
                    }
                else
                    {
                     printf("Turning relay1_sw OFF\n");
                     gpio_set_level(GPIO_OUTPUT_0, 0);
                     relay_status1 = false;
                    }
                
                }
            }
        }
}



// Wifi event handler
static esp_err_t event_handler(void *ctx, system_event_t *event)
    {
    switch(event->event_id) 
        {

        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;

        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(event_group, WIFI_CONNECTED_BIT);
            break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(event_group, WIFI_CONNECTED_BIT);
            esp_wifi_connect();//fraant PAtch
            break;

        default:
            break;
        }  
    return ESP_OK;
    }


static void http_server_netconn_serve(struct netconn *conn) 
{

    struct netbuf *inbuf;
    char *buf;
    u16_t buflen;
    err_t err;

    err = netconn_recv(conn, &inbuf);

    if (err == ERR_OK) {

        netbuf_data(inbuf, (void**)&buf, &buflen);

        // extract the first line, with the request
        char *first_line = strtok(buf, "\n");
        printf("First_line:%s\n",first_line);

        if(first_line) {

            char statusPage2[255]="";
            //genero il relaod page automatico a tempo con l'ip assegnao all'interfaccia
            snprintf(statusPage2,255,"<meta http-equiv=\"refresh\" content=\"0;URL=http://%s\">",my_ip);//stringa per ricaricare la pagna base
            // default page
            if(strstr(first_line, "GET / ")) 
                {
                 char statusPage[255]="";
                 //invio la parte terminale e dinamica della WebPage
                 snprintf(statusPage,255,"<tr><td>min:%d sec: %d</td><td>min:%d sec: %d</td></tr></tbody></table><br>KeepAlive sent:%d</body></html>",relay1on/60,relay1on%60,relay2on/60,relay2on%60,mainkeepalivecount);
                 printf("Sending index.htm page...\n");
                 netconn_write(conn, http_html_hdr, sizeof(http_html_hdr) - 1, NETCONN_NOCOPY);
                 netconn_write(conn, index_htm_start, index_htm_end - index_htm_start, NETCONN_NOCOPY);
                 netconn_write(conn, statusPage, strlen(statusPage), NETCONN_NOCOPY);
                }

            // Sw1 page
            else if(strstr(first_line, "GET /sw1.htm ")) 
                {

                if(relay_status1 == false) 
                    {			
                     printf("Turning relay1 ON\n");
                     gpio_set_level(GPIO_OUTPUT_0, 1);
                     relay_status1 = true;
                    }
                else
                    {
                     printf("Turning relay1 OFF\n");
                     gpio_set_level(GPIO_OUTPUT_0, 0);
                     relay_status1 = false;
                    }
                netconn_write(conn, http_html_hdr, sizeof(http_html_hdr) - 1, NETCONN_NOCOPY);
                //gli faccio ricaricale la pagina principale per fare scomparire il link nell'address bar
                netconn_write(conn, statusPage2, strlen(statusPage2), NETCONN_NOCOPY);
                }			

            // sw2 page
            else if(strstr(first_line, "GET /sw2.htm ")) 
                {

                if(relay_status2 == true) 
                    {			
                    printf("Turning relay2 OFF\n");
                    gpio_set_level(GPIO_OUTPUT_1, 0);
                    relay_status2 = false;
                    }
                else
                    {
                    printf("Turning relay2 OFF\n");
                    gpio_set_level(GPIO_OUTPUT_1, 1);
                    relay_status2 = true;
                    }

                printf("Sending OFF page...\n");
                netconn_write(conn, http_html_hdr, sizeof(http_html_hdr) - 1, NETCONN_NOCOPY);
                //gli faccio ricaricale la pagina principale per fare scomparire il link nell'address bar
                netconn_write(conn, statusPage2, strlen(statusPage2), NETCONN_NOCOPY);
                }

            // sw1 image me la chiede e invio l'immaggine in base allo stato del relay
            else if(strstr(first_line, "GET /sw1.png ")) 
                {
                printf("Sending sw1 status:%d...\n",relay_status1);
                netconn_write(conn, http_png_hdr, sizeof(http_png_hdr) - 1, NETCONN_NOCOPY);
                if(relay_status1==true)
                    {
                     netconn_write(conn, on_png_start, on_png_end - on_png_start, NETCONN_NOCOPY);
                    }
                else
                    {
                     netconn_write(conn, off_png_start, off_png_end - off_png_start, NETCONN_NOCOPY);
                    }
                }

            // sw2 image me la chiede e invio l'immaggine in base allo stato del relay
            else if(strstr(first_line, "GET /sw2.png ")) 
                {
                printf("Sending sw1 status:%d...\n",relay_status2);
                netconn_write(conn, http_png_hdr, sizeof(http_png_hdr) - 1, NETCONN_NOCOPY);
                if(relay_status2==true)
                    {
                     netconn_write(conn, on_png_start, on_png_end - on_png_start, NETCONN_NOCOPY);
                    }
                else
                    {
                     netconn_write(conn, off_png_start, off_png_end - off_png_start, NETCONN_NOCOPY);
                    }
                }

            // Index.htm PAGE
            else if(strstr(first_line, "GET /index.htm ")) 
                {
                printf("Sending index.htm page...\n");
                netconn_write(conn, http_html_hdr, sizeof(http_html_hdr) - 1, NETCONN_NOCOPY);
                netconn_write(conn, index_htm_start, index_htm_end - index_htm_start, NETCONN_NOCOPY);
                }

            // Index.htm PAGE
            else if(strstr(first_line, "GET /favicon.ico ")) 
                {
                printf("Sending favicon...\n");
                netconn_write(conn, http_png_hdr, sizeof(http_png_hdr) - 1, NETCONN_NOCOPY);
                netconn_write(conn, favicon_ico_start, favicon_ico_end - favicon_ico_start, NETCONN_NOCOPY);
                }

            else printf("Unkown request: %s\n", first_line);
        }
        else printf("Unkown request\n");
    }

    // close the connection and free the buffer
    netconn_close(conn);
    netbuf_delete(inbuf);
}

static void http_server(void *pvParameters) {

    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    printf("HTTP Server listening...\n");
    do {
        err = netconn_accept(conn, &newconn);
        printf("New client connected\n");
        if (err == ERR_OK) 
            {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
            }
        vTaskDelay(1); //allows task to be pre-empted
        } while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
    printf("\n");
}


// setup and start the wifi connection
void wifi_setup() {

    event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "osp",
            .password = "pippopippo",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}


// configure the output PIN
void gpio_setup() {

    // configure the relay pin as GPIO, output
    //gpio_pad_select_gpio(CONFIG_RELAY_PIN);

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

    // set initial status = OFF
    gpio_set_level(GPIO_OUTPUT_0, 0);
    gpio_set_level(GPIO_OUTPUT_1, 0);

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



}


// Main application
void app_main()
    {
    struct timeval tv;
    time_t nowtime;
    struct tm *nowtm;
    char tmbuf[32]="";

    // disable the default wifi logging
    esp_log_level_set("wifi", ESP_LOG_NONE);

    nvs_flash_init();
    i2c_master_init();

    ssd1306_display_clear(1);

    loadbmp((uint8_t*)Timer0_bmp_start,(int)(Timer0_bmp_end-Timer0_bmp_start),0);vTaskDelay((300 /portTICK_PERIOD_MS));
    loadbmp((uint8_t*)Timer1_bmp_start,(int)(Timer1_bmp_end-Timer1_bmp_start),0);vTaskDelay((300 /portTICK_PERIOD_MS));
    loadbmp((uint8_t*)Timer2_bmp_start,(int)(Timer2_bmp_end-Timer2_bmp_start),0);vTaskDelay((300 /portTICK_PERIOD_MS));
    loadbmp((uint8_t*)Timer3_bmp_start,(int)(Timer3_bmp_end-Timer3_bmp_start),0);vTaskDelay((300 /portTICK_PERIOD_MS));
    loadbmp((uint8_t*)Timer4_bmp_start,(int)(Timer4_bmp_end-Timer4_bmp_start),0);vTaskDelay((300 /portTICK_PERIOD_MS));
    loadbmp((uint8_t*)Warning_bmp_start,(int)(Warning_bmp_end-Warning_bmp_start),0);vTaskDelay((200 /portTICK_PERIOD_MS));

    WriteStringToOled("FraAnt 18",10,1,1,0);
    WriteMiniStringToOled("The Best Family.....",1,7,0,0);
    WriteStringToOled("ESP8266EX",14,3,0,1);

    wifi_setup();
    gpio_setup();

    // wait for connection
    printf("Waiting for connection to the wifi network...\n ");
    xEventGroupWaitBits(event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    printf("Connected\n\n");

    // print the local IP address
    tcpip_adapter_ip_info_t ip_info;
    ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
    snprintf(my_ip,17,"%s",ip4addr_ntoa(&ip_info.ip));
    printf("IP Address:  %s\n", my_ip);
    printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
    printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));	

    //time update
    get_time();

    // start the HTTP Server task
    xTaskCreate(&http_server, "http_server", 2048, NULL, 3, NULL);
   
    uint8_t shiftx=6; 
    uint32_t lastsec=0; 
    while(1)
        {
         vTaskDelay((980 / portTICK_PERIOD_MS));
         seconds++;
         if(seconds>TTL_PACKET_TIME)
            {
              xTaskCreate(FrKeepAlive, "udp_keepalive", 1024, NULL, 5, NULL);
              seconds=0;
            }
         if(ERRORS>0){esp_restart();}//mio watchdog.. riavvio
         gettimeofday(&tv, NULL);
         nowtime = tv.tv_sec;
         if (tv.tv_sec!=lastsec)
            {
             nowtm = localtime(&nowtime);
             strftime(tmbuf,sizeof(tmbuf),"%H:%M:%S",nowtm);
#ifdef USEOLED
             ssd1306_display_clear(0);
             AddBigi2NumtoOledVideoMem(1+shiftx,1,tmbuf[0]);
             AddBigi2NumtoOledVideoMem(25+shiftx,1,tmbuf[1]);
             AddBigi2NumtoOledVideoMem(50+shiftx,1,58);
             AddBigi2NumtoOledVideoMem(65+shiftx,1,tmbuf[3]);
             AddBigi2NumtoOledVideoMem(90+shiftx,1,tmbuf[4]);
             OledDrawRect(0,0,128,64);
             OledDrawHorizontalLine(0,43,128);
             strftime(tmbuf,sizeof(tmbuf),"%d/%m/%Y %S",nowtm);
             WriteMiniStringToOled(tmbuf,33,6,0,1);
#endif

             //stampa icone relative ai led
             if(relay_status1==false)
                {
                 relay1on++;//contiamo i minuti di relay acceso
                 loadico((uint8_t*)light_10x12,3,47,10,12,120,1);
                }
             else
                {
                 relay1on=0;//contiamo i minuti di relay acceso
                 loadico((uint8_t*)light_10x12,3,47,10,12,120,0);
                }
             if(relay_status2==false)
                {
                 relay2on++;
                 loadico((uint8_t*)light_10x12,17,47,10,12,120,1);
                }
             else
                {
                 relay2on=0;//contiamo i minuti di relay acceso
                 loadico((uint8_t*)light_10x12,17,47,10,12,120,0);
                }
                
             lastsec=tv.tv_sec;
             //printf("\n time:%s",tmbuf);
            }
        }
    }
