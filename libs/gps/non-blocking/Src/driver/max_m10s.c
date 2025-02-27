#include "max_m10s.h"

static gps_status_e handle_init_state(max_m10s_t *dev);
static gps_status_e handle_init_bus_operations(max_m10s_t *dev);
static bool start_i2c_transmission(max_m10s_t *dev, uint16_t size);
static bool start_i2c_receive(max_m10s_t *dev, uint16_t size);
static bool start_delay_timer(max_m10s_t *dev, uint32_t delay_ms);


// Private callback implementations
static void max_m10s_tx_complete_cb(void* context) {
    max_m10s_t* dev = (max_m10s_t*)context;
    if (dev) {
        dev->flags.bits.write_done = 1;
        dev->flags.bits.bus_busy = 0;
    }
}

static void max_m10s_rx_complete_cb(void* context) {
    max_m10s_t* dev = (max_m10s_t*)context;
    if (dev) {
        dev->flags.bits.read_done = 1;
        dev->flags.bits.bus_busy = 0;
    }
}

static void max_m10s_error_cb(void* context, HAL_StatusTypeDef error) {
    (void)error;
    max_m10s_t* dev = (max_m10s_t*)context;
    if (dev) {
        dev->flags.bits.bus_error = 1;
        dev->flags.bits.bus_busy = 0;

        // Set specific error flags based on operation
        if (dev->bus_state == GPS_BUS_OP_READ) {
            dev->flags.bits.read_error = 1;
        } else if (dev->bus_state == GPS_BUS_OP_WRITE) {
            dev->flags.bits.write_error = 1;
        }
    }
}

static void max_m10s_delay_complete_cb(void* context) {
    max_m10s_t* dev = (max_m10s_t*)context;
    if (dev) {
        // We're using write_done as our delay complete flag
        dev->flags.bits.delay_done = 1;
        dev->flags.bits.bus_busy = 0;
    }
}

gps_status_e max_m10s_init(max_m10s_t *dev, hal_interface_t *hal) {
    if (!dev || !hal || !hal->hi2c) {
        #ifdef DEBUG
            debug_print("Invalid parameters\n");
        #endif
        return UBLOX_ERROR;
    }

    if(dev->initialized) {
        #ifdef DEBUG
            debug_print("Device already initialized\n");
        #endif
        // Already initialized
        return UBLOX_OK;
    }

    // Initialize device structure
    dev->initialized = false;
    dev->state = GPS_INITING;
    dev->op_mode = GPS_NON_BLOCKING;  // Default to non-blocking mode
    dev->bus_state = GPS_BUS_OP_NONE;
    dev->flags.all = 0;  // Clear all flags or event bits

    // Store the HAL interface
    dev->interface = hal;

    // Initialize the HAL interface
    HAL_StatusTypeDef hal_status = hal_interface_init(hal);
    if (hal_status != HAL_OK) {
        #ifdef DEBUG
            debug_print("HAL interface initialization failed\n");
        #endif
        return UBLOX_ERROR;
    }

    // Register our callbacks with the HAL interface
    hal->callbacks.context = dev;
    hal->callbacks.tx_complete = max_m10s_tx_complete_cb;
    hal->callbacks.rx_complete = max_m10s_rx_complete_cb;
    hal->callbacks.error = max_m10s_error_cb;
    hal->callbacks.delay_complete = max_m10s_delay_complete_cb;


    // Initialize UBX protocol layer
    dev->protocol = UBX_PROTOCOL;
    if (dev->protocol != UBX_PROTOCOL) {
        #ifdef DEBUG
            debug_print("Invalid protocol, NMEA not implmented yet\n");
        #endif
        return UBLOX_ERROR;
    }

    // Mark basic initialization as done
    dev->flags.bits.inited = 1;

    // The rest of the initialization will be handled by the state machine
    // in max_m10s_run()

    return UBLOX_OK;
}


gps_status_e max_m10s_run(max_m10s_t *dev) {
    if (!dev || !dev->interface) {
        return UBLOX_ERROR;
    }
    // Handle the main states
    switch (dev->state) {
        case GPS_INITING:
            return handle_init_state(dev);
        case GPS_READY:
            #ifdef DEBUG
                debug_print("GPS Ready\n");
            #endif
            return UBLOX_OK;
            //@todo: Implment this branch
            //return handle_ready_state(dev);
        case GPS_CONFIGURING:
            #ifdef DEBUG
                debug_print("GPS Configuring\n");
            #endif
            return UBLOX_OK;
            // @todo: Implement this branch
            //return handle_config_state(dev);
        case GPS_GETTING_DATA:
            #ifdef DEBUG
                debug_print("GPS Getting Data\n");
            #endif
            return UBLOX_OK;
            // @todo: Implement this branch
            //return handle_data_state(dev);
        case GPS_RESETTING:
            #ifdef DEBUG
                debug_print("GPS Resetting\n");
            #endif
            return UBLOX_OK;
            // @todo: Implement this branch
            //return handle_reset_state(dev);
        default:
            #ifdef DEBUG
                debug_print("Invalid state: %d\n", dev->state);
            #endif
            return UBLOX_ERROR;

    }

    return UBLOX_ERROR;
}


static gps_status_e handle_init_state(max_m10s_t *dev) {
    //GPS_INITING (we need to send UBX config messages to the module)
    if (!dev->flags.bits.inited) {
        // Not initialized yet
        dev->config_stage = GPS_CONFIG_NONE;
        #ifdef DEBUG
            debug_print("Device not initialized\n");
        #endif
        return UBLOX_NOT_INITED;
    }

    // Handle bus operations for initialization
    gps_status_e status = handle_init_bus_operations(dev);

    // Check for state transition conditions
    if (status == UBLOX_OK && dev->config_stage == GPS_CONFIG_DONE) {
        dev->state = GPS_READY;
    } else if (dev->flags.bits.needs_reset) {
        #ifdef DEBUG
            debug_print("Something wrong with init state handle. Device needs reset\n");
        #endif
        dev->state = GPS_RESETTING;
    }

    return status;
}


static gps_status_e handle_init_bus_operations(max_m10s_t *dev) {
    gps_status_e status = UBLOX_OK;
    uint16_t msg_size;

    switch(dev->bus_state) {
        case GPS_BUS_OP_NONE:
            // Determine which configuration command to send
            if(dev->config_stage == GPS_CONFIG_NONE ||
               dev->config_stage == GPS_CONFIG_UBX_ENABLE){
                msg_size = ubx_prepare_config_cmd(
                    dev->tx_buffer,
                    UBX_CFG_I2C_UBX_ENABLE,
                    1
                );
                dev->config_stage = GPS_CONFIG_UBX_ENABLE;
            }
            else if(dev->config_stage == GPS_CONFIG_NMEA_DISABLE) {
                msg_size = ubx_prepare_config_cmd(
                    dev->tx_buffer,
                    UBX_CFG_I2C_NMEA_DISABLE,
                    0
                );
            }
            else {
                // Invalid configuration stage
                #ifdef DEBUG
                    debug_print("Invalid config stage: %d\n", dev->config_stage);
                #endif
                return UBLOX_ERROR;
            }
            if(start_i2c_transmission(dev, msg_size)) {
                #ifdef DEBUG
                    debug_print("Starting I2C transmission\n");
                #endif
                dev->bus_state = GPS_BUS_OP_WRITE;
            } else {
                #ifdef DEBUG
                    debug_print("Error starting I2C transmission\n");
                #endif
                return UBLOX_ERROR;
            }
            break;

        case GPS_BUS_OP_WRITE:
            // Check if the write operation is done
            if (dev->flags.bits.write_done) {
                dev->bus_state = GPS_BUS_OP_DELAY;
                dev->flags.bits.write_done = 0;
                start_delay_timer(dev, 1000);  // 1 second delay
            }
            else {
                #ifdef DEBUG
                    debug_print("Write operation not done\n");
                #endif
            }
            break;
        case GPS_BUS_OP_DELAY:
            // Check if the delay operation is done
            if (dev->flags.bits.delay_done) {
                dev->flags.bits.delay_done = 0;
                if(start_i2c_receive(dev, UBX_ACK_PACKET_SIZE)){
                    dev->bus_state = GPS_BUS_OP_READ;
                } else {
                    #ifdef DEBUG
                        debug_print("Error starting I2C receive\n");
                    #endif
                    return UBLOX_ERROR;
                }
            }
            break;
        case GPS_BUS_OP_READ:
            // Check if the read operation is done
            if (dev->flags.bits.read_done) {
                // Validate ACK response
                uint8_t expected_cls = UBX_CLASS_CFG;
                uint8_t expected_id = UBX_CFG_VALSET;

                status = ubx_validate_ack(
                    dev->rx_buffer,
                    UBX_ACK_PACKET_SIZE,
                    expected_cls,
                    expected_id
                );

                if (status == UBLOX_OK) {
                    // Move to next config state
                    if (dev->config_stage == GPS_CONFIG_UBX_ENABLE) {
                        dev->config_stage = GPS_CONFIG_NMEA_DISABLE;
                    } else if (dev->config_stage == GPS_CONFIG_NMEA_DISABLE) {
                        dev->config_stage = GPS_CONFIG_DONE;
                        dev->flags.bits.config_valid = 1;
                    }
                    dev->bus_state = GPS_BUS_OP_NONE;

                } else {
                    #ifdef DEBUG
                        debug_print("ACK validation failed\n");
                    #endif
                    dev->flags.bits.needs_reset = 1;
                }
                dev->flags.bits.read_done = 0;
            }
            break;
        default:
            #ifdef DEBUG
                debug_print("Invalid bus state: %d\n", dev->bus_state);
            #endif
            return UBLOX_ERROR;
    }

    return UBLOX_OK;
}


// Add these helper functions to max_m10s.c
bool start_i2c_transmission(max_m10s_t *dev, uint16_t size) {
    hal_interface_t *hal = (hal_interface_t *)dev->interface;

    HAL_StatusTypeDef hal_status = hal->ops.transmit_it(
        hal->hi2c,
        UBLOX_I2C_ADDR,  // Device address for the GPS module
        dev->tx_buffer,
        size
    );

    if (hal_status == HAL_OK) {
        dev->flags.bits.bus_busy = 1;
        return true;
    } else {
        #ifdef DEBUG
            debug_print("Error starting I2C transmission\n");
        #endif
        dev->flags.bits.bus_error = 1;
        return false;
    }
}

bool start_i2c_receive(max_m10s_t *dev, uint16_t size) {
    hal_interface_t *hal = (hal_interface_t *)dev->interface;

    HAL_StatusTypeDef hal_status = hal->ops.receive_it(
        hal->hi2c,
        UBLOX_I2C_ADDR,
        dev->rx_buffer,
        size
    );

    if (hal_status == HAL_OK) {
        dev->flags.bits.bus_busy = 1;
        return true;
    } else {
        #ifdef DEBUG
            debug_print("Error starting I2C receive\n");
        #endif
        dev->flags.bits.bus_error = 1;
        return false;
    }
}

bool start_delay_timer(max_m10s_t *dev, uint32_t delay_ms) {
    hal_interface_t *hal = (hal_interface_t *)dev->interface;

    HAL_StatusTypeDef hal_status = hal->ops.delay_it(
        hal->htim,
        delay_ms
    );

    if (hal_status == HAL_OK) {
        dev->flags.bits.bus_busy = 1;
        return true;
    } else {
        dev->flags.bits.bus_error = 1;
        return false;
    }
}
