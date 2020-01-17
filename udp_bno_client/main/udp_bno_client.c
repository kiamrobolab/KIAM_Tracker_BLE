/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "bno055.h"


/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT

#ifdef CONFIG_EXAMPLE_I2C_PORT_0
#define I2C_PORT I2C_NUMBER_0
#elif CONFIG_EXAMPLE_I2C_PORT_1
#define I2C_PORT I2C_NUMBER_1
#endif

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static const char *TAG = "example";
static char payload[1024];

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
#ifdef CONFIG_EXAMPLE_IPV4
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        break;
#else
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
            xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");

            char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
            ESP_LOGI(TAG, "IPv6: %s", ip6);
#endif
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
#ifdef CONFIG_EXAMPLE_IPV4
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
#else
        xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
#endif
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
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;

    ESP_LOGI(TAG, "Waiting for AP connection...");
    xEventGroupWaitBits(wifi_event_group, bits, false, false, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");
}

static void udp_client_task(void *pvParameters)
{
    //char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    esp_err_t err;
    uint32_t time_mks, time_mks_after;
    float time_check_sum = 0;
    int n_sent = 0;
    int n_check_sum = 0;
    bno055_quaternion_t quat;
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 destAddr;
        inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        time_mks = esp_log_timestamp();
        xLastWakeTime = xTaskGetTickCount();
        while (1) {
        	vTaskDelayUntil(&xLastWakeTime, xFrequency);

   			err = bno055_get_quaternion(I2C_PORT, &quat);
   			if( err != ESP_OK ) {
   				printf("bno055_get_quaternion() returned error: %02x \n", err);
   				exit(2);
   			}

   			time_mks_after = esp_log_timestamp();
   			time_check_sum = time_check_sum + ((float)time_mks_after - (float)time_mks)/1000;

   			sprintf(payload, "%.5f\
   					#linacc,%.3f,%.3f,%.3f\
   					#rotvec,%.3f,%.3f,%.3f,%.3f\
   					#rotmat,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\
   					#gyr,%.3f,%.3f,%.3f\
   					#acc,%.3f,%.3f,%.3f\
   					#grav,%.3f,%.3f,%.3f\
                    #mag,%.3f,%.3f,%.3f",
   					(float)time_mks_after/1000,
   					0., 0., 0.,
					quat.w, quat.x, quat.y, quat.z,
					0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
					0., 0., 0.,
					0., 0., 0.,
					0., 0., 0.,
					0., 0., 0.);
   			time_mks = time_mks_after;

   			ESP_LOGI(TAG, "%s", payload);

            err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "%u %u", time_mks_after, esp_log_timestamp());
            n_sent++;

            if (time_check_sum > 1)
            {
            	sprintf(payload, "#%d %f Checksum#%d", n_sent, time_check_sum, n_check_sum);
            	ESP_LOGI(TAG, "Check sum sent: %s", payload);
            	err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&destAddr, sizeof(destAddr));
            	if (err < 0)
            	{
            	    ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
            	    break;
            	}
            	time_check_sum = 0;
            	n_sent = 0;
            	n_check_sum++;
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void app_main()
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    wait_for_ip();

    bno055_config_t bno_conf;

    //    esp_log_level_set("*", ESP_LOG_INFO);

    esp_err_t err;
    err = bno055_set_default_conf( &bno_conf);
    bno_conf.i2c_address = BNO055_ADDRESS_B;
    bno_conf.sda_io_num = 21;
    bno_conf.scl_io_num = 22;
    err = bno055_open(I2C_PORT, &bno_conf);
    printf("bno055_open() returned 0x%02X \n", err);

    if( err != ESP_OK ) {
        printf("Program terminated!\n");
        err = bno055_close(I2C_PORT);
        printf("bno055_close() returned 0x%02X \n", err);
        exit(1);
    }

    vTaskDelay(1000 / portTICK_RATE_MS);

    err = bno055_set_opmode(I2C_PORT, OPERATION_MODE_NDOF);
    printf("bno055_set_opmode(OPERATION_MODE_NDOF) returned 0x%02x \n", err);
    vTaskDelay(1000 / portTICK_RATE_MS);

    TaskHandle_t xHandle = NULL;
    // create task on the APP CPU (CPU_1)
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, &xHandle);
  // err = xTaskCreatePinnedToCore(quat_task,   // task function
  //  		                      "quat_task",    // task name for debugging
  //          					   2048,          // stack size bytes
  //  		        			   &i2c_num,      // pointer to params to pass
  //  							   10,            // task priority. configMAX_PRIORITIES == 25, so 24 is the highest. 0 is idle
  //  							   &xHandle,      // returned task handle
  //  							   1);            // CPU to use 1 means APP_CPU
}
