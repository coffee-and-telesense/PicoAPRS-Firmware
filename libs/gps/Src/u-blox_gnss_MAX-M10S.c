/*
File u-blox_gnss_MAX-M10S.c
*/

#include "u-blox_gnss_MAX-M10S.h"

// Private function headers
static UBLOX_Status_t ublox_send_command(ubx_packet_t *outgoing_ubx, bool expect_ack_only);
static void calc_check_sum(ubx_packet_t *outgoing_ubx);
static uint16_t set_transaction_size(uint16_t len);
static UBLOX_Status_t ublox_set_i2c_output(comm_type_t comm_settings);

// Private initialization flag might need to fix this
static bool ublox_initialized = false;
extern UART_HandleTypeDef huart2;

#ifdef DEBUG 
    const uint8_t ublox_error_init[] = "u-blox GNSS Module not initialized. Call ublox_init() first.\r\n";
    const uint8_t ublox_error_memset[] = "Memory allocation failed. Check if memory is available.\r\n";
    const uint8_t ublox_error_i2c[] = "I2C peripheral not initialized. Check I2C configuration.\r\n";
    const uint8_t ublox_packet_error[] = "u-blox packet error. Check packet validity.\r\n";
    const uint8_t ublox_transmits_msg[] = "Requesting data from u-blox.\r\n";
    const uint8_t ublox_transaction_size_overflow[] = "Transaction size overflow. Check packet length.\r\n";
#endif

UBLOX_Status_t ublox_init(void){
    // Initialize the I2C peripheral
    MX_I2C1_Init();
    // Zero packet structure
    memset(&ubx_packet, 0, sizeof(ubx_packet_t));
    ublox_initialized = true;

    UBLOX_Status_t status = ublox_set_i2c_output(UBX);
    if(status != UBLOX_OK)
        return status; 
        
    return UBLOX_OK;
}


// Based on sparkfun's getNAVSTATUS(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); func
UBLOX_Status_t ublox_get_nav_status(void){
    // Check if the module has been initialized
    bool ack_only = false;
    if(!ublox_initialized){
        #ifdef DEBUG
            HAL_UART_Transmit(&huart2, ublox_error_init, sizeof(ublox_error_init), 10);
        #endif
        return UBLOX_NOT_INITIALIZED;
    }
    if(ubx_packet.valid.bits.packet){
        #ifdef DEBUG
            HAL_UART_Transmit(&huart2, ublox_packet_error, sizeof(ublox_packet_error), 10);
        #endif
        return UBLOX_PACKET_NEEDS_PROCESSING;
    }
    memset(&ubx_packet, 0, sizeof(ubx_packet_t)); // Clear the packet before sending a new request
    ubx_packet.cls = UBX_CLASS_NAV;
    ubx_packet.id = UBX_NAV_STATUS;

    UBLOX_Status_t status = ublox_send_command(&ubx_packet, ack_only);
    return status;
}


// Private function
static UBLOX_Status_t ublox_send_command(ubx_packet_t *outgoing_ubx, bool expect_ack_only){
    //TODO: Implement this function
    calc_check_sum(outgoing_ubx);

    #ifdef DEBUG
        HAL_UART_Transmit(&huart2, ublox_transmits_msg, sizeof(ublox_transmits_msg), 10);
    #endif   
    //uint16_t bytes_left_to_send = outgoing_ubx->len;
    //uint16_t start_spot = 0; 
    
    //to set transaction size
    uint16_t size = set_transaction_size(outgoing_ubx->len);
    if (size == 0){ // Should never be zero 
        return UBLOX_ERROR;
    }
    uint8_t buff[size];
    buff[0] = UBX_SYNC_CHAR_1;
    buff[1] = UBX_SYNC_CHAR_2;
    buff[2] = outgoing_ubx->cls;
    buff[3] = outgoing_ubx->id;
    buff[4] = outgoing_ubx->len & 0xFF;     //LSB
    buff[5] = outgoing_ubx->len >> 8;       // MSB
    uint16_t i = 0; 
    for(; i < outgoing_ubx->len; i++){
        // TODO: Add error check here so payload and i don't go out of bounds
        buff[i + 6] = outgoing_ubx->payload[i];
    }
    buff[i + 6] = outgoing_ubx->checksumA;
    buff[i + 7] = outgoing_ubx->checksumB;

    // Send the data on bus
    if(HAL_I2C_Master_Transmit(&hi2c1, UBLOX_I2C_ADDR, buff, size, I2C_TIMEOUT) != HAL_OK){
        return UBLOX_ERROR;
    }
    
    if(expect_ack_only) {
        uint8_t rx_buff[10];
        HAL_Delay(1000);
        if(HAL_I2C_Master_Receive(&hi2c1, UBLOX_I2C_ADDR, rx_buff, 10, I2C_TIMEOUT) != HAL_OK){
            return UBLOX_ERROR; 
        }
        //Parse incoming datas ack 
    }

    
    return UBLOX_OK;
}
/* TODO Remove me later
// Private function
static UBLOX_Status_t ublox_send_command(ubx_packet_t *outgoing_ubx, bool expect_ack_only){
    //TODO: Implement this function
    calc_check_sum(outgoing_ubx);

    #ifdef DEBUG
        HAL_UART_Transmit(&huart2, ublox_transmits_msg, sizeof(ublox_transmits_msg), 10);
    #endif   
    //uint16_t bytes_left_to_send = outgoing_ubx->len;
    //uint16_t start_spot = 0; 
    
    //to set transaction size
    uint16_t size = set_transaction_size(outgoing_ubx->len);
    if (size == 0){ // Should never be zero 
        return UBLOX_ERROR;
    }
    uint8_t buff[size];
    buff[0] = UBX_SYNC_CHAR_1;
    buff[1] = UBX_SYNC_CHAR_2;
    buff[2] = outgoing_ubx->cls;
    buff[3] = outgoing_ubx->id;
    buff[4] = outgoing_ubx->len & 0xFF;
    buff[5] = outgoing_ubx->len >> 8;
    uint16_t i = 0; 
    for(; i < outgoing_ubx->len; i++){
        // TODO: Add error check here so payload and i don't go out of bounds
        buff[i + 6] = outgoing_ubx->payload[i];
    }
    buff[i + 6] = outgoing_ubx->checksumA;
    buff[i + 7] = outgoing_ubx->checksumB;

    // Send the data on bus
    if(HAL_I2C_Master_Transmit(&hi2c1, UBLOX_I2C_ADDR, buff, size, I2C_TIMEOUT) != HAL_OK){
        return UBLOX_ERROR;
    }
    // Now read the response
    uint8_t rx_size = UBX_HEADER_LENGTH + UBX_NAV_STATUS_LEN + UBX_CHECKSUM_LENGTH; 
    uint8_t rx_buff[UBX_MAX_PACKET_LENGTH];

    // Read response
    if(HAL_I2C_Master_Receive(&hi2c1, UBLOX_I2C_ADDR, rx_buff, rx_size, I2C_TIMEOUT) != HAL_OK){
        return UBLOX_ERROR;
    }

    //TODO: need to parse data from buffer

    
    return UBLOX_OK;
}
*/

// Fletch Checksum Algorithm
static void calc_check_sum(ubx_packet_t *outgoing_ubx) {
    // Reset checksums
    outgoing_ubx->checksumA = 0;
    outgoing_ubx->checksumB = 0;
    
    // Add class
    outgoing_ubx->checksumA += outgoing_ubx->cls;
    outgoing_ubx->checksumB += outgoing_ubx->checksumA;
    
    // Add ID
    outgoing_ubx->checksumA += outgoing_ubx->id;
    outgoing_ubx->checksumB += outgoing_ubx->checksumA;
    
    // Add length (2 bytes, little endian)
    outgoing_ubx->checksumA += (outgoing_ubx->len & 0xFF);
    outgoing_ubx->checksumB += outgoing_ubx->checksumA;
    
    outgoing_ubx->checksumA += (outgoing_ubx->len >> 8);
    outgoing_ubx->checksumB += outgoing_ubx->checksumA;
    
    // Add payload bytes if there are any
    if(outgoing_ubx->payload != NULL && outgoing_ubx->len > 0) {
        for(uint16_t i = 0; i < outgoing_ubx->len; i++) {
            outgoing_ubx->checksumA += outgoing_ubx->payload[i];
            outgoing_ubx->checksumB += outgoing_ubx->checksumA;
        }
    }
    
}

static uint16_t set_transaction_size(uint16_t len){
    // Number of bytes written is payload length + 8 bytes for header and checksum
    uint16_t size = len + 8;
    if(size > UBX_MAX_PACKET_LENGTH){
        #ifdef DEBUG
            HAL_UART_Transmit(&huart2, ublox_transaction_size_overflow, sizeof(ublox_transaction_size_overflow), 10);
        #endif
        return 0; // Error
    } 
    return size;   
}


// Config setting
static UBLOX_Status_t ublox_set_i2c_output(comm_type_t comm_settings) {
    if(!ublox_initialized) {
        #ifdef DEBUG
            HAL_UART_Transmit(&huart2, ublox_error_init, sizeof(ublox_error_init), 10);
        #endif
        return UBLOX_NOT_INITIALIZED;
    }

    //Clear packet structure
    memset(&ubx_packet, 0, sizeof(ubx_packet_t));

    // Setup packet for CFG-VALSET message
    ubx_packet.cls = UBX_CLASS_CFG; 
    ubx_packet.id = UBX_CFG_VALSET;
    
    // Payload for CFG-VALSET
    uint8_t payload[] = {
        0x00,        // version 
        0x01,        // layer = RAM only
        0x00, 0x00,  // reserved0
        // CFG-I2COUTPROT-UBX key-value pair
        0x01, 0x00, 0x72, 0x10,  // Key ID (0x10720001)
    };

    ubx_packet.len = sizeof(payload);
    ubx_packet.payload = payload;

    // Send command and wait for ACK
    return ublox_send_command(&ubx_packet, true);
}





