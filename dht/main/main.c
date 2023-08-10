#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "connect_wifi.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_attr.h"
#include "esp_spi_flash.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>

#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/event_groups.h"
#include "E:/Esp/Espressif/frameworks/esp-idf-v4.4.4/examples/common_components/protocol_examples_common/include/protocol_examples_common.h"
#include "esp_http_client.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "dht.h"

#include <math.h>
#include "esp_adc_cal.h"
#include "unistd.h"
#include <time.h>


#define PERIOD_GET_DATA_FROM_SENSOR                 (TickType_t)(5000 / portTICK_RATE_MS)

#define WEB_SERVER "api.thingspeak.com"
#define WEB_PORT "80"

// #define DHT_PIN GPIO_NUM_14
// #define WEB_PATH "/"
static const char *TAG = "example";
char REQUEST[512];
char recv_buf[512];
char SUBREQUEST[150];
float temperature;
	float humidity;

#define SPEAKER_PIN GPIO_NUM_2


#define NGUONG_NHIET_DO               30.0f
#define NGUONG_DO_AM                  60.0f

#ifndef LCD_I2C_H_
#define LCD_I2c_H_

void lcd_init(void); // initialize lcd

void lcd_send_cmd(char cmd); // send command to the lcd

void lcd_send_data(char data); // send data to the lcd

void lcd_send_string(char *str); // send string to the lcd

void lcd_put_cur(int row, int col); // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear(void);

#endif

// #include "lcd_i2c.h"

#include "esp_log.h"
// #include "driver/i2c.h"
#include "unistd.h"

#define LCD_ADDR 0x27 // LCD address
#define I2C_NUM I2C_NUM_0
static const char *TAG1 = "LCD";

esp_err_t err;

void lcd_send_cmd(char cmd)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C;
    data_t[1] = data_u | 0x08;
    data_t[2] = data_l | 0x0C;
    data_t[3] = data_l | 0x08;
    err = i2c_master_write_to_device(I2C_NUM, LCD_ADDR, data_t, 4, 1000);
    if (err != 0)
        ESP_LOGI(TAG1, "Error in sending command");
}

void lcd_send_data(char data)
{
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D;
    data_t[1] = data_u | 0x09;
    data_t[2] = data_l | 0x0D;
    data_t[3] = data_l | 0x09;
    err = i2c_master_write_to_device(I2C_NUM, LCD_ADDR, data_t, 4, 1000);
    if (err != 0)
        ESP_LOGI(TAG1, "Error in sending data");
}

void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    usleep(5000);
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
    case 0:
        col |= 0x80;
        break;
    case 1:
        col |= 0xC0;
        break;
    }

    lcd_send_cmd(col);
}

void lcd_init(void)
{
    usleep(50000);
    lcd_send_cmd(0x30);
    usleep(5000);
    lcd_send_cmd(0x30);
    usleep(200);
    lcd_send_cmd(0x30);
    usleep(10000);
    lcd_send_cmd(0x20);
    usleep(10000);

    lcd_send_cmd(0x28);
    usleep(1000);
    lcd_send_cmd(0x08);
    usleep(1000);
    lcd_send_cmd(0x01);
    usleep(1000);
    usleep(1000);
    lcd_send_cmd(0x06);
    usleep(1000);
    lcd_send_cmd(0x0C);
    usleep(1000);
}

void lcd_send_string(char *str)
{
    while (*str)
        lcd_send_data(*str++);
}

// float V0 = 4095, R1 = 5000, R0 = 10000, B = 3950, T0 = 298.15;
// float V1, v1, R2, r, T;

// static const char *TAG = "i2c-simple-example";
// char buffer[15];

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void init_speaker() {
    gpio_pad_select_gpio(SPEAKER_PIN);
    gpio_set_direction(SPEAKER_PIN, GPIO_MODE_OUTPUT);
}

void alert_warning() {
    for (int i = 0; i < 5; i++) { // Phát âm thanh cảnh báo 3 lần
        gpio_set_level(SPEAKER_PIN, 1);  // Bật loa
        vTaskDelay(pdMS_TO_TICKS(100));  // Thời gian bật loa (100ms)
        gpio_set_level(SPEAKER_PIN, 0);  // Tắt loa
        vTaskDelay(pdMS_TO_TICKS(100));  // Thời gian tắt loa (100ms)
    }
}

static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    //char recv_buf[64];
    init_speaker();

    while(1) {
        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
       
        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

        // Read data from DHT11 sensor
        DHT11_init(GPIO_NUM_5);
        float temperature, humidity;
        int dht_err = dht11_read_data(GPIO_NUM_5, &temperature, &humidity);
         // Kiểm tra nhiệt độ hoặc độ ẩm có vượt ngưỡng không
            if (temperature > NGUONG_NHIET_DO || humidity < NGUONG_DO_AM) {
                // Cảnh báo qua loa
                alert_warning();
            }
      
        
        sprintf(SUBREQUEST, "api_key=EVALE3CKLASUP2NG&field1=%.2f&field2=%.2f",temperature, humidity);
		printf("Nhiet do= %.2f,Do am= %.2f, ", temperature, humidity);
        sprintf(REQUEST, "POST /update HTTP/1.1\nHost: api.thingspeak.com\nConection: close\nContent-Type: application/x-www-form-urlencoded\nContent-Length:%d\n\n%s\n",strlen(SUBREQUEST), SUBREQUEST);
        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        for(int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
    }
}


void display_data_on_lcd(void)
{
    char buffer0[15];
    char buffer1[15];

    while (1)
    {
        // Read data from DHT11 sensor
        DHT11_init(GPIO_NUM_5);
        int dht_err = dht11_read_data(GPIO_NUM_5, &temperature, &humidity);

        // Clear LCD and update data
        lcd_clear();

        // Display temperature
        sprintf(buffer0, "Nhietdo = %.2f", temperature);
        lcd_put_cur(0, 0);
        lcd_send_string(buffer0);

        // Display humidity
        sprintf(buffer1, "Doam = %.2f", humidity);
        lcd_put_cur(1, 0);
        lcd_send_string(buffer1);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}




void app_main(void)
{
	
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	} 
	ESP_ERROR_CHECK(ret);


   
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	connect_wifi();
	if (wifi_connect_status)
	{
		 init_speaker();
		xTaskCreate(&http_get_task, "http_get_task", 8192, NULL, 7, NULL);
	}
     ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG1, "I2C initialized successfully");
    lcd_init();
    lcd_clear();
   
    xTaskCreate(&display_data_on_lcd, "display_data_on_lcd", 4096, NULL, 5, NULL);


}

