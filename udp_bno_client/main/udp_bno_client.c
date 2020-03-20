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

#define SEND_PORT CONFIG_SEND_PORT
#define RECEIVE_PORT CONFIG_RECEIVE_PORT
#ifdef CONFIG_BROADCAST_PORT
#define BROADCAST_PORT CONFIG_BROADCAST_PORT
#else
#define BROADCAST_PORT 5550
#endif

#ifdef CONFIG_EXAMPLE_I2C_PORT_0
#define I2C_PORT I2C_NUMBER_0
#elif CONFIG_EXAMPLE_I2C_PORT_1
#define I2C_PORT I2C_NUMBER_1
#endif

#define USE_EXTERNAL_CRYSTAL CONFIG_USE_CRYSTAL

enum modes{
    FUSION_MODE,
    RAW_MODE
};
static const char* FUSION_MES = "fusion";
static const char* RAW_MES = "raw";

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static const char *TAG = "example";
static char buffer[1024];
static char payload[1024];
static char addr_str[128];

static bool isSynchronized = false;
static bool isBroadcasted = false;

static bool mode = FUSION_MODE;

static uint32_t offset, delay;

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

static void send_bno_fusion(int sock, struct sockaddr_in* address)
{
    uint32_t time_mks, time_mks_after;
    float time_checksum = 0;
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
    TickType_t xLastWakeTime;
    esp_err_t err;
    int n_mes = 0;
    int n_sent = 0;
    int n_checksum = 0;
    uint8_t calib;
    bno055_quaternion_t quat = {0, 0, 0, 0};
    bno055_vec3_t lin_accel = {0, 0, 0};
    bno055_vec3_t grav = {0, 0, 0};
    bno055_vec3_t gyr = {0, 0, 0};
    bno055_vec3_t acc = {0, 0, 0};
    bno055_vec3_t mag = {0, 0, 0};

#ifdef CONFIG_EXAMPLE_IPV4
    socklen_t socklen = sizeof(struct sockaddr_in);
#else //IPV6
    socklen_t socklen = sizeof(struct sockaddr_in6);
#endif
    
    time_mks = esp_log_timestamp();
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (mode == FUSION_MODE) {
       	    err = bno055_get_fusion_data(I2C_PORT, &quat, &lin_accel, &grav);
        } else {
            // TODO: change function with read raw data function
            err = bno055_get_fusion_data(I2C_PORT, &quat, &lin_accel, &grav);
        }
        if( err != ESP_OK ) {
   		    printf("bno055_get_fusion() returned error: %02x \n", err);
   		    exit(2);
   		}
   		err = bno055_get_calib_status_byte(I2C_PORT, &calib);
   		if( err != ESP_OK ) {
   		    printf("bno055_get_calib_status_byte() returned error: %02x \n", err);
   		    exit(2);
   		}

	    time_mks_after = esp_timer_get_time();

	    sprintf(payload, "%d"
		    	"#time,%.5f"
		    	"#calib_status,%d"
				"#linacc,%5f,%5f,%5f"
				"#rotvec,%.5f,%.5f,%.5f,%.5f"
				"#grav,%.5f,%.5f,%.5f"
				"#gyr,%.5f,%.5f,%.5f"
				"#acc,%.5f,%.5f,%.5f"
                "#mag,%.5f,%.5f,%.5f",
				n_mes,
				(float)time_mks_after/1000,
				calib,
				lin_accel.x, lin_accel.y, lin_accel.z,
				quat.w, quat.x, quat.y, quat.z,
				grav.x, grav.y, grav.z,
				gyr.x, gyr.y, gyr.z,
				acc.x, acc.y, acc.z,
				mag.x, mag.y, mag.z);

        err = sendto(sock, payload, strlen(payload), 0, address, &socklen);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
            break;
        }

        n_mes++;
        n_sent++;

        time_checksum = time_checksum + ((float)time_mks_after - (float)time_mks)/1000000;
        time_mks = time_mks_after;
        if (time_checksum > 1)
        {
       	    sprintf(payload, "#%d %f Checksum#%d", n_sent, time_checksum, n_checksum);
       	    ESP_LOGI(TAG, "Check sum sent: %s", payload);
      	    err = sendto(sock, payload, strlen(payload), 0, address, &socklen);
      	    if (err < 0)
       	    {
       	        ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
       	        break;
       	    }
       	    time_checksum = 0;
       	    n_sent = 0;
       	    n_checksum++;
        }
    }
}

static void udp_client_task(void *pvParameters)
{
    int addr_family;
    int ip_protocol;

    while (1) {

    	if (isSynchronized) {

#ifdef CONFIG_EXAMPLE_IPV4
            struct sockaddr_in destAddr;
            destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
            destAddr.sin_family = AF_INET;
            destAddr.sin_port = htons(SEND_PORT);
            addr_family = AF_INET;
            ip_protocol = IPPROTO_IP;
            inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
            struct sockaddr_in6 destAddr;
            inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
            destAddr.sin6_family = AF_INET6;
            destAddr.sin6_port = htons(SEND_PORT);
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

            send_bno_fusion(sock, &destAddr);

            if (sock != -1) {
                ESP_LOGE(TAG, "Shutting down socket and restarting...");
                shutdown(sock, 0);
                close(sock);
            }
    	}
    }
    vTaskDelete(NULL);
}

static void timesync_handler(int sock)
{
    struct sockaddr_in6 buf_sourceAddr; // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(buf_sourceAddr);

    char time_str[256];

    const char *tokens;
    uint32_t t1_sent, t1_received, t2_sent, t2_received, t3_sent, t3_received;

    while (1) {

        ESP_LOGI(TAG, "Waiting for timestamp");
        int len = recvfrom(sock, time_str, sizeof(time_str) - 1, 0, (struct sockaddr *)&buf_sourceAddr, &socklen);
        t1_received = esp_timer_get_time();
        // Error occured during receiving
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            isSynchronized = false;
            isBroadcasted = false;
            break;
        }
        // Data received
        else {
            // Get the sender's ip address as string
            if (buf_sourceAddr.sin6_family == PF_INET) {
                inet_ntoa_r(((struct sockaddr_in *)&buf_sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
            } else if (buf_sourceAddr.sin6_family == PF_INET6) {
                inet6_ntoa_r(buf_sourceAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
            }

            if (strcmp(addr_str, HOST_IP_ADDR) == 0) {

                time_str[len] = 0; // Null-terminate whatever we received and treat like a string...

                strcpy(buffer, time_str);
                tokens = strtok(buffer, ",");
                t1_sent = atoi(tokens);

                vTaskDelay(10 / portTICK_PERIOD_MS);

                t2_sent = esp_timer_get_time();
                sprintf(time_str, "%s,%d,%d", time_str, t1_received, t2_sent);

                int err = sendto(sock, time_str, strlen(time_str), 0, (struct sockaddr *)&buf_sourceAddr, &socklen);
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                    break;
                }

                len = recvfrom(sock, time_str, sizeof(time_str) - 1, 0, (struct sockaddr *)&buf_sourceAddr, &socklen);
                t3_received = esp_timer_get_time();

                if (len < 0) {
                    ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                    isBroadcasted = false;
                  	break;
                }
                else {
                    time_str[len] = 0; // Null-terminate whatever we received and treat like a string...
                  	//ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);

                   	strcpy(buffer, time_str);
                   	tokens = strtok(buffer, ",");
                   	t2_received = atoi(tokens + 4);
                   	t3_sent = atoi(tokens + 5);

                   	offset = (t2_received - t2_sent - t3_received + t3_sent) / 2;
                   	isSynchronized = true;
                   	vTaskDelay(10000 / portTICK_PERIOD_MS);
                }
            }
        }
    }
}

static void broadcast_handler(int sock)
{
    const char port_str[5];
    itoa(SEND_PORT, port_str, 10);
    char mes_buffer[1024];
    const char *tokens;

    struct sockaddr_in6 buf_sourceAddr; // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(buf_sourceAddr);

    while (1) {

        ESP_LOGI(TAG, "Waiting for broadcast");
        int len = recvfrom(sock, mes_buffer, sizeof(mes_buffer) - 1, 0, (struct sockaddr *)&buf_sourceAddr, &socklen);
        // Error occured during receiving
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        }
        // Data received
        else {
            // Get the sender's ip address as string
            if (buf_sourceAddr.sin6_family == PF_INET) {
                inet_ntoa_r(((struct sockaddr_in *)&buf_sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
            } else if (buf_sourceAddr.sin6_family == PF_INET6) {
                inet6_ntoa_r(buf_sourceAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
            }
            if (strcmp(addr_str, HOST_IP_ADDR) == 0) {

                mes_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...

                strcpy(buffer, mes_buffer);
                tokens = strtok(buffer, "#");
                if (strcmp((tokens + 1), FUSION_MES) == 0) {
                    mode = FUSION_MODE;
                } else if (strcmp((tokens + 1), RAW_MES) == 0){
                    mode = RAW_MODE;
                } else {
                    mode = FUSION_MODE;
                    ESP_LOGI(TAG, "Unknown mode, use fusion mode as default");
                }

                sprintf(mes_buffer, "%s#%s", mes_buffer, port_str);

                int err = sendto(sock, mes_buffer, strlen(mes_buffer), 0, (struct sockaddr *)&buf_sourceAddr, sizeof(buf_sourceAddr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                    break;
                }
                isBroadcasted = true;
                break;
            }
        }
    }
}

static void udp_server_task(void *pvParameters)
{
    int addr_family;
    int ip_protocol;

    struct timeval timeout;
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;

    while (1) {

        if (isBroadcasted){

#ifdef CONFIG_EXAMPLE_IPV4
            struct sockaddr_in sourceAddr;
            sourceAddr.sin_addr.s_addr = htonl(INADDR_ANY);
            sourceAddr.sin_family = AF_INET;
            sourceAddr.sin_port = htons(RECEIVE_PORT);
            addr_family = AF_INET;
            ip_protocol = IPPROTO_IP;
            inet_ntoa_r(sourceAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
            struct sockaddr_in6 sourceAddr;
            bzero(&sourceAddr.sin6_addr.un, sizeof(sourceAddr.sin6_addr.un));
            sourceAddr.sin6_family = AF_INET6;
            sourceAddr.sin6_port = htons(RECEIVE_PORT);
            addr_family = AF_INET6;
            ip_protocol = IPPROTO_IPV6;
            inet6_ntoa_r(sourceAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

            int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
            if (sock < 0) {
                ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Socket created");

            int err = bind(sock, (struct sockaddr *)&sourceAddr, sizeof(sourceAddr));
            if (err < 0) {
                ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            }
            ESP_LOGI(TAG, "Socket binded");

            if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
                ESP_LOGE(TAG, "... failed to set socket receiving timeout");
                close(sock);
                continue;
            }

            timesync_handler(sock);

            if (sock != -1) {
                ESP_LOGE(TAG, "Shutting down socket and restarting...");
                shutdown(sock, 0);
                close(sock);
            }
        }
        else {

#ifdef CONFIG_EXAMPLE_IPV4
            struct sockaddr_in sourceAddr;
            sourceAddr.sin_addr.s_addr = htonl(INADDR_ANY);
            sourceAddr.sin_family = AF_INET;
            sourceAddr.sin_port = htons(BROADCAST_PORT);
            addr_family = AF_INET;
            ip_protocol = IPPROTO_IP;
            inet_ntoa_r(sourceAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
            struct sockaddr_in6 sourceAddr;
            bzero(&sourceAddr.sin6_addr.un, sizeof(sourceAddr.sin6_addr.un));
            sourceAddr.sin6_family = AF_INET6;
            sourceAddr.sin6_port = htons(BROADCAST_PORT);
            addr_family = AF_INET6;
            ip_protocol = IPPROTO_IPV6;
            inet6_ntoa_r(sourceAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

            int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
            if (sock < 0) {
                ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Socket created");

            int err = bind(sock, (struct sockaddr *)&sourceAddr, sizeof(sourceAddr));
            if (err < 0) {
                ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            }
            ESP_LOGI(TAG, "Socket binded");

            broadcast_handler(sock);

            if (sock != -1) {
                ESP_LOGE(TAG, "Shutting down socket and restarting...");
                shutdown(sock, 0);
                close(sock);
            }
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

    err = bno055_set_ext_crystal_use(I2C_PORT, USE_EXTERNAL_CRYSTAL);
    if( err != ESP_OK ) {
        printf("Couldn't set external crystal use\n");
        err = bno055_close(I2C_PORT);
        printf("bno055_close() returned 0x%02X \n", err);
        exit(1);
    }
    vTaskDelay(1000 / portTICK_RATE_MS);

    err = bno055_set_opmode(I2C_PORT, OPERATION_MODE_NDOF);
    printf("bno055_set_opmode(OPERATION_MODE_NDOF) returned 0x%02x \n", err);
    vTaskDelay(1000 / portTICK_RATE_MS);

    TaskHandle_t xHandle = NULL;
    // TODO: create task on the WIFI CPU (CPU_1)
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, &xHandle);
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, &xHandle);

}
