
#include <string.h> // memset()
#include "esp_log.h"
#include "driver/i2c.h"

#include "driver/gpio.h"

#define I2C_MASTER_TX_BUF_DISABLE 0  // I2C master doesn't need buffer 
#define I2C_MASTER_RX_BUF_DISABLE 0  // I2C master doesn't need buffer 

#define BNO055_OPR_MODE_ADDR				(0X3D)

#define OPERATION_MODE_CONFIG (0X00)
#define OPERATION_MODE_NDOF 0x0C

#define BNO055_CHIP_ID_ADDR                 (0x00)

#define BNO055_QUATERNION_DATA_W_LSB_ADDR (0X20)

#define BNO055_TEMP_ADDR (0X34)

#define BNO055_PWR_MODE_ADDR				(0X3E)

#define BNO055_ID 0

#define BNO055_SYS_TRIGGER_ADDR (0X3F)

#define BNO055_NUM_OFFSET_REGISTERS 1

#define POWER_MODE_NORMAL (0X00)

#define WRITE_BIT I2C_MASTER_WRITE  // I2C master write 
#define READ_BIT I2C_MASTER_READ    // I2C master read 
#define ACK_CHECK_EN 0x1            // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0           // I2C master will not check ack from slave 
#define ACK_VAL 0x0                 // I2C ack value 
#define NACK_VAL 0x1                // I2C nack value 

#define BNO055_ADDRESS_A 0
#define BNO055_ADDRESS_B 1

typedef uint8_t bno055_addr_t;
typedef uint8_t bno055_reg_t;
typedef uint8_t bno055_opmode_t;

static const char *TAG = "bno055.c";
   
    
typedef struct{
    bno055_addr_t  i2c_address; // BNO055_ADDRESS_A or BNO055_ADDRESS_B
    bool  bno_is_open;
} bno055_device_t;

/*!
* @brief struct for Quaternion data read from registers
*/
typedef struct{
	uint16_t w;/**< Quaternion w data */
	uint16_t x;/**< Quaternion x data */
	uint16_t y;/**< Quaternion y data */
	uint16_t z;/**< Quaternion z data */
} bno055_quaternion_t;

typedef struct{
	bno055_addr_t i2c_address;           // BNO055_ADDRESS_A or BNO055_ADDRESS_B
    uint8_t sda_io_num;        // GPIO number for I2C sda signal 25
    bool sda_pullup_en;  // Internal GPIO pull mode for I2C sda signal
    uint8_t scl_io_num;        // GPIO number for I2C scl signal 26
    bool scl_pullup_en;  // Internal GPIO pull mode for I2C scl signal
    uint32_t clk_speed;     // I2C clock frequency for master mode, (no higher than 1MHz for now)
    uint32_t timeout;
} bno055_i2c_config_t;

typedef struct{
	uint8_t chip_id;
	uint8_t accel_id;
	uint8_t mag_id;
	uint8_t gyro_id;
	uint8_t sw_rev;
	uint8_t bl_rev;
} bno055_chip_info_t;

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

esp_err_t bno055_set_default_conf(bno055_i2c_config_t * p_bno_conf);

esp_err_t bno055_open(i2c_port_t i2c_num, bno055_i2c_config_t * p_bno_conf );

esp_err_t bno055_close (i2c_port_t i2c_num );

esp_err_t bno055_get_chip_info(i2c_port_t i2c_num, bno055_chip_info_t* chip_inf);

void bno055_displ_chip_info(bno055_chip_info_t chip_inf);

esp_err_t bno055_set_mode(i2c_port_t i2c_num, bno055_opmode_t mode );

esp_err_t bno055_get_mode(i2c_port_t i2c_num, bno055_opmode_t * mode );

// Note: should be in config mode to work!
esp_err_t bno055_set_ext_crystal_use(i2c_port_t i2c_num, bool use_ext );

esp_err_t bno055_get_temperature(i2c_port_t i2c_num, uint8_t* p_temperature);
    
esp_err_t bno055_get_quaternion(i2c_port_t i2c_num, bno055_quaternion_t* quat);
    

  
