#include <string.h> // memset()
#include "esp_log.h"
#include "driver/i2c.h"

#include "bno055.h"
   

// Internal functions

// _______________________________________________________________________
// | start | write chip_addr + wr_bit, chk_ack | write reg_addr, chk_ack |
// --------|-----------------------------------|-------------------------|
// ________________________________________________________________________
// | start | write chip_addr + rd_bit, chk_ack | read 1 byte, nack | stop |
// --------|-----------------------------------|-------------------|------|
esp_err_t bno055_read_register(i2c_port_t i2c_num, bno055_reg_t reg, uint8_t *p_reg_val)
{
    
    if( !x_bno_dev[i2c_num].bno_is_open) {
        ESP_LOGW(TAG, "bno055_read_register(): device is not open");
        return 1; //TODO: make error list
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // making the command - begin 
    i2c_master_start(cmd);  // start condition
    // device address with write bit
    i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | WRITE_BIT, ACK_CHECK_EN); 
    // send the register address
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);   
    i2c_master_start(cmd);  // start condition again
    // device address with read bit
    i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | READ_BIT, ACK_CHECK_EN);   
    // read byte, issue NACK
    i2c_master_read_byte(cmd, p_reg_val, NACK_VAL); 
    i2c_master_stop(cmd);  // stop condition
    // making the command - end 
    
    // Now execute the command
    esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    switch (err) {
        case ESP_OK: 
            break;
        case  ESP_ERR_TIMEOUT:
            ESP_LOGW(TAG, "bno055_read_register(): i2c timeout");
            break;
        default: 
            ESP_LOGW(TAG, "bno055_read_register(): failed");
    }
    
    return err;   
}
  
// ______________________________________________________________________________________________________
// | start | write chip_addr + wr_bit, chk_ack | write reg_addr, chk_ack | write 1 byte, chk_ack | stop |
// --------|-----------------------------------|-------------------------|-----------------------|------|

 esp_err_t bno055_write_register(i2c_port_t i2c_num, bno055_reg_t reg, uint8_t reg_val){
    
    if( !x_bno_dev[i2c_num].bno_is_open) {
        ESP_LOGW(TAG, "bno055_write_register(): device is not open");
        return 1; //TODO: make error list
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // making the command - begin 
    i2c_master_start(cmd);  // start condition
    // device address with write bit
    i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | WRITE_BIT, ACK_CHECK_EN); 
    // send the register address
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);   
    // write byte, issue NACK
    i2c_master_write_byte(cmd, reg_val, ACK_CHECK_EN); 
    i2c_master_stop(cmd);  // stop condition
    // making the command - end 
    
    // Now execute the command
    esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
    i2c_cmd_link_delete(cmd);
    
    switch (err) {
        case ESP_OK: 
            break;
        case  ESP_ERR_TIMEOUT:
            ESP_LOGW(TAG, "bno055_write_register(): i2c timeout");
            break;
        default: 
            ESP_LOGW(TAG, "bno055_write_register(): failed");
    }
    
    return err;   
}
 

 
// _______________________________________________________________________
// | start | write chip_addr + wr_bit, chk_ack | write reg_addr, chk_ack |
// --------|-----------------------------------|-------------------------|
// ______________________________________________________________________________________________
// | start | write chip_addr + rd_bit, chk_ack | read n-1 bytes, ack | read 1 byte, nack | stop |
// --------|-----------------------------------|---------------------|-------------------|------|
 
esp_err_t bno055_read_data(i2c_port_t i2c_num, bno055_reg_t start_reg, uint8_t *buffer, uint8_t n_bytes){

    if( !x_bno_dev[i2c_num].bno_is_open) {
        ESP_LOGW(TAG, "bno055_read_data(): device is not open");
        return 1; //TODO: make error list
    }

    if( n_bytes < 2 || n_bytes > BNO055_NUM_OFFSET_REGISTERS ) {
        ESP_LOGW(TAG, "bno055_read_data(): invalid number of bytes: %d", n_bytes);
        return 1; //TODO: make error list
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // making the command - begin
    i2c_master_start(cmd);  // start condition
    // device address with write bit
    i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | WRITE_BIT, ACK_CHECK_EN);
    // send the register address
    i2c_master_write_byte(cmd, start_reg, ACK_CHECK_EN);
    i2c_master_start(cmd);  // start condition again
    // device address with read bit
    i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | READ_BIT, ACK_CHECK_EN);
    // read n_bytes-1, issue ACK
    i2c_master_read(cmd, buffer, n_bytes - 1, ACK_VAL);
    // read the last byte, issue NACK
    i2c_master_read_byte(cmd, buffer+n_bytes-1, NACK_VAL);
    i2c_master_stop(cmd);  // stop condition
    // making the command - end

    // Now execute the command
    esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);

    switch (err) {
        case ESP_OK:
            break;
        case  ESP_ERR_TIMEOUT:
            ESP_LOGW(TAG, "bno055_read_data(): i2c timeout");
            break;
        default:
            ESP_LOGW(TAG, "bno055_read_data(): failed");
    }

    return err;
}


// Public functions

esp_err_t bno055_set_default_conf(bno_i2c_config_t * p_bno_conf){

    p_bno_conf->i2c_address = BNO055_ADDRESS_A;          // BNO055_ADDRESS_A or BNO055_ADDRESS_B 
    p_bno_conf->sda_io_num = 18;        // GPIO number for I2C sda signal 25
    p_bno_conf->sda_pullup_en = GPIO_PULLUP_DISABLE;  // Internal GPIO pull mode for I2C sda signal
    p_bno_conf->scl_io_num = 19;        // GPIO number for I2C scl signal 26 
    p_bno_conf->scl_pullup_en = GPIO_PULLUP_DISABLE;  // Internal GPIO pull mode for I2C scl signal
    p_bno_conf->clk_speed = 400000;     // I2C clock frequency for master mode, (no higher than 1MHz for now) 
    p_bno_conf->timeout = 16300;            // in 80 MHz ticks, should be < 0x3FFF 
    
    

    return ESP_OK;   
}



esp_err_t bno055_open(i2c_port_t i2c_num, bno_i2c_config_t * p_bno_conf )
{
    if(i2c_num >= I2C_NUM_MAX) return ESP_ERR_INVALID_ARG;
    
    // Check if already in use
    if(x_bno_dev[i2c_num].bno_is_open) {
        ESP_LOGW(TAG, "bno055_open(): device is already open");
        return 2; //TODO: make error list
    }
    
    x_bno_dev[i2c_num].bno_is_open = 0;

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = p_bno_conf->sda_io_num;        
    conf.sda_pullup_en = p_bno_conf->sda_pullup_en;  
    conf.scl_io_num = p_bno_conf->scl_io_num;        
    conf.scl_pullup_en = p_bno_conf->scl_pullup_en;  
    conf.master.clk_speed = p_bno_conf->clk_speed;
    
    esp_err_t err;
    
    err = i2c_param_config(i2c_num, &conf);
    printf("i2c_param_config() returned %02x \n", err);
    if( err != ESP_OK ) return err;
        
    err = i2c_driver_install(i2c_num, I2C_MODE_MASTER,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
    printf("i2c_driver_install() returned %02x \n", err);
    if( err != ESP_OK ) return err;
    
    err = i2c_set_timeout(i2c_num, p_bno_conf->timeout);
    printf("i2c_set_timeout() returned %02x \n", err);
    if( err != ESP_OK ) return err;
    
    x_bno_dev[i2c_num].i2c_address = p_bno_conf->i2c_address;
    
    // Read BNO055 Chip ID to make sure we have a connection
    x_bno_dev[i2c_num].bno_is_open = 1; // bno055_read_register() checks this flag
    uint8_t reg_val;
    err = bno055_read_register(i2c_num, BNO055_CHIP_ID_ADDR, & reg_val);
    
    if( err == ESP_OK ) {
        
        printf("BNO055 ID returned 0x%02X \n", reg_val);
        if( reg_val == BNO055_ID ) {
            printf("BNO055 detected \n");
        } 
        else {
            ESP_LOGW(TAG, "bno055_open() error: BNO055 NOT detected");
            goto errExit;
        }
    }
    
    
     // Switch to config mode
    err = bno055_write_register(i2c_num, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    if(err != ESP_OK) goto errExit;
    vTaskDelay(30 / portTICK_RATE_MS);

    // Reset 
    err=bno055_write_register(i2c_num, BNO055_SYS_TRIGGER_ADDR, 0x20);
    if(err != ESP_OK) goto errExit;
    vTaskDelay(500 / portTICK_RATE_MS);
    
    // Set power mode to normal 
    err=bno055_write_register(i2c_num, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    if(err != ESP_OK) goto errExit;
    vTaskDelay(20 / portTICK_RATE_MS);

    return ESP_OK;   
    
errExit:
    bno055_close (i2c_num);
    return err;
}



esp_err_t bno055_close (i2c_port_t i2c_num )
{
    x_bno_dev[i2c_num].bno_is_open = 0;
    return i2c_driver_delete(i2c_num);
  
}

esp_err_t bno055_get_chip_info(i2c_port_t i2c_num, bno055_chip_info_t* chip_inf){
    
    memset(chip_inf, 0, sizeof(bno055_chip_info_t));
    
    esp_err_t err = bno055_read_data(i2c_num, BNO055_CHIP_ID_ADDR, x_buffer, 7);
    if( err != ESP_OK ) return err;
    
    chip_inf->chip_id = x_buffer[0];
    chip_inf->accel_id = x_buffer[1];
    chip_inf->mag_id = x_buffer[2];
    chip_inf->gyro_id = x_buffer[3];
    chip_inf->sw_rev = x_buffer[4] + ((uint16_t)x_buffer[5]<<8);
    chip_inf->bl_rev = x_buffer[6];
    
    return ESP_OK;
}   

void bno055_displ_chip_info(bno055_chip_info_t chip_inf){

    printf("BNO055 Chip ID (0xA0): 0x%02X \n", chip_inf.chip_id );
    printf("Accelerometer Chip ID (0xFB): 0x%02X \n", chip_inf.accel_id );
    printf("Magnetometer Chip ID (0x32): 0x%02X \n", chip_inf.mag_id );
    printf("Gyroscope Chip ID (0x0F): 0x%02X \n", chip_inf.gyro_id );
    printf("Software Revision: %d \n", chip_inf.sw_rev );
    printf("Bootloader Revision: %d \n", chip_inf.bl_rev );

}   

esp_err_t bno055_set_mode(i2c_port_t i2c_num, bno055_opmode_t mode ){
    
    esp_err_t err=bno055_write_register(i2c_num, BNO055_OPR_MODE_ADDR, mode);
    vTaskDelay(30 / portTICK_RATE_MS);
    return err;
}


esp_err_t bno055_get_mode(i2c_port_t i2c_num, bno055_opmode_t * mode ){
    
    uint8_t ui_mode;
    esp_err_t err = bno055_read_register(i2c_num, BNO055_OPR_MODE_ADDR, &ui_mode);
    ui_mode = ui_mode & 0x0F; // upper 4 bits are reserved, lower 4 represent the mode
    * mode = ui_mode;
    return err;
}

// Note: should be in config mode to work!
esp_err_t bno055_set_ext_crystal_use(i2c_port_t i2c_num, bool use_ext ){
    
    bno055_opmode_t mode;
    esp_err_t err = bno055_get_mode(i2c_num, & mode );
    if( err != ESP_OK ) return err;
    
    if( mode  != OPERATION_MODE_CONFIG ) {
        ESP_LOGW(TAG, "bno055_set_ext_crystal_use(): device should be in the config mode. Current mode: %d", mode);
        return 3; //TODO: make error list
    }
   
    uint8_t reg_val;
    if(use_ext) reg_val = 0x80;
    else reg_val = 0;

    // Set ext crystal on/off 
    err=bno055_write_register(i2c_num, BNO055_SYS_TRIGGER_ADDR, reg_val);
    vTaskDelay(10 / portTICK_RATE_MS);
    return err;
}


esp_err_t bno055_get_temperature(i2c_port_t i2c_num, uint8_t* p_temperature){
    
  esp_err_t err = bno055_read_register(i2c_num, BNO055_TEMP_ADDR, p_temperature);
  return err;
}
    
esp_err_t bno055_get_quaternion(i2c_port_t i2c_num, bno055_quaternion_t* quat) {

    esp_err_t err = bno055_read_data(i2c_num, BNO055_QUATERNION_DATA_W_LSB_ADDR, x_buffer, 8);
    if( err != ESP_OK ) return err;

    int16_t x, y, z, w;

    w = (((uint16_t)x_buffer[1]) << 8) | ((uint16_t)x_buffer[0]);
    x = (((uint16_t)x_buffer[3]) << 8) | ((uint16_t)x_buffer[2]);
    y = (((uint16_t)x_buffer[5]) << 8) | ((uint16_t)x_buffer[4]);
    z = (((uint16_t)x_buffer[7]) << 8) | ((uint16_t)x_buffer[6]);

    /* Assign to Quaternion */
    /* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
        3.6.5.5 Orientation (Quaternion)  */
    const double scale = (1.0 / (1<<14));
    quat->w = scale * w;
    quat->x = scale * x;
    quat->y = scale * y;
    quat->z = scale * z;
    
    return ESP_OK;
}
    

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
  
