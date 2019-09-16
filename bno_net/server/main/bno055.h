
#include <string.h> // memset()
#include "esp_log.h"
#include "driver/i2c.h"

#include "bno055.h"

#define I2C_MASTER_TX_BUF_DISABLE 0  // I2C master doesn't need buffer 
#define I2C_MASTER_RX_BUF_DISABLE 0  // I2C master doesn't need buffer 

#define WRITE_BIT I2C_MASTER_WRITE  // I2C master write 
#define READ_BIT I2C_MASTER_READ    // I2C master read 
#define ACK_CHECK_EN 0x1            // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0           // I2C master will not check ack from slave 
#define ACK_VAL 0x0                 // I2C ack value 
#define NACK_VAL 0x1                // I2C nack value 

typedef uint8_t bno055_addr_t;
typedef uint8_t bno055_reg_t;

   
static const char *TAG = "bno055.c";
   
    
typedef struct
{
    bno055_addr_t  i2c_address; // BNO055_ADDRESS_A or BNO055_ADDRESS_B
    bool  bno_is_open;
} bno055_device_t;


static bno055_device_t x_bno_dev[I2C_NUM_MAX];

static uint8_t x_buffer[200];

// Internal functions

// _______________________________________________________________________
// | start | write chip_addr + wr_bit, chk_ack | write reg_addr, chk_ack |
// --------|-----------------------------------|-------------------------|
// ________________________________________________________________________
// | start | write chip_addr + rd_bit, chk_ack | read 1 byte, nack | stop |
// --------|-----------------------------------|-------------------|------|
esp_err_t bno055_read_register(i2c_port_t i2c_num, bno055_reg_t reg, uint8_t *p_reg_val);
  
// ______________________________________________________________________________________________________
// | start | write chip_addr + wr_bit, chk_ack | write reg_addr, chk_ack | write 1 byte, chk_ack | stop |
// --------|-----------------------------------|-------------------------|-----------------------|------|

 esp_err_t bno055_write_register(i2c_port_t i2c_num, bno055_reg_t reg, uint8_t reg_val);
 
// _______________________________________________________________________
// | start | write chip_addr + wr_bit, chk_ack | write reg_addr, chk_ack |
// --------|-----------------------------------|-------------------------|
// ______________________________________________________________________________________________
// | start | write chip_addr + rd_bit, chk_ack | read n-1 bytes, ack | read 1 byte, nack | stop |
// --------|-----------------------------------|---------------------|-------------------|------|
 
esp_err_t bno055_read_data(i2c_port_t i2c_num, bno055_reg_t start_reg, uint8_t *buffer, uint8_t n_bytes);

// Public functions

esp_err_t bno055_set_default_conf(bno_i2c_config_t * p_bno_conf);

esp_err_t bno055_open(i2c_port_t i2c_num, bno_i2c_config_t * p_bno_conf );

esp_err_t bno055_close (i2c_port_t i2c_num );

esp_err_t bno055_get_chip_info(i2c_port_t i2c_num, bno055_chip_info_t* chip_inf);

void bno055_displ_chip_info(bno055_chip_info_t chip_inf);

esp_err_t bno055_set_mode(i2c_port_t i2c_num, bno055_opmode_t mode );

esp_err_t bno055_get_mode(i2c_port_t i2c_num, bno055_opmode_t * mode );

// Note: should be in config mode to work!
esp_err_t bno055_set_ext_crystal_use(i2c_port_t i2c_num, bool use_ext );

esp_err_t bno055_get_temperature(i2c_port_t i2c_num, uint8_t* p_temperature);
    
esp_err_t bno055_get_quaternion(i2c_port_t i2c_num, bno055_quaternion_t* quat);
    

/*

    // Set the output units 
    //
    uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
    write8(BNO055_UNIT_SEL_ADDR, unitsel);
    

    // Configure axis mapping (see section 3.4) 
    
    write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
    delay(10);
    write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
    delay(10);

    write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
    delay(10);
    
    // Set the requested operating mode (see section 3.3) 
    setMode(mode);
    delay(20);

    // Choose page 0 
    err = bno055_write_register(i2c_num, BNO055_PAGE_ID_ADDR, 0);
    if(err != ESP_OK) goto errExit;
    vTaskDelay(30 / portTICK_RATE_MS);
    

        uint8_t prev_mode;
    esp_err_t err = bno055_read_register(i2c_num, BNO055_CHIP_ID_ADDR, & prev_mode);
    if( err != ESP_OK ) return err;
    
    // Set config mode
    err = bno055_set_mode(i2c_num, OPERATION_MODE_CONFIG);
    if( err != ESP_OK ) return err;
    vTaskDelay(25 / portTICK_RATE_MS);

    
    
    
    */
  
