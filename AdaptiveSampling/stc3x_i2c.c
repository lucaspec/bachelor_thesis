/*
 * THIS FILE IS AUTOMATICALLY GENERATED
 *
 * I2C-Generator: 0.2.0
 * Yaml Version: 0.1.1
 * Template Version: 0.6.0
 */
/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "stc3x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"
#include <msp430.h>


#define STC3X_I2C_ADDRESS 0x29

int16_t stc3x_set_binary_gas(uint16_t binary_gas) {
    int16_t error;
    uint8_t buffer[5];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3615);

    offset =
        sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset, binary_gas);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(1000);
    return NO_ERROR;
}

int16_t stc3x_set_relative_humidity(uint16_t relative_humidity_ticks) {
    int16_t error;
    uint8_t buffer[5];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3624);

    offset = sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset,
                                                  relative_humidity_ticks);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(1000);
    return NO_ERROR;
}

int16_t stc3x_set_temperature(uint16_t temperature_ticks) {
    int16_t error;
    uint8_t buffer[5];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x361E);

    offset = sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset,
                                                  temperature_ticks);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(1000);
    return NO_ERROR;
}

int16_t stc3x_set_pressure(uint16_t absolute_pressure) {
    int16_t error;
    uint8_t buffer[5];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x362F);

    offset = sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset,
                                                  absolute_pressure);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(1000);
    return NO_ERROR;
}

int16_t stc3x_measure_gas_concentration(uint16_t* gas_ticks,
                                        uint16_t* temperature_ticks) {
    int16_t error;
    uint8_t buffer[6];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3639);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    _delay_cycles(1600000); // 100ms delay ... //sensirion_i2c_hal_sleep_usec(70000);

    error = sensirion_i2c_read_data_inplace(STC3X_I2C_ADDRESS, &buffer[0], 4);
    if (error) {
        return error;
    }
    *gas_ticks = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    *temperature_ticks = sensirion_common_bytes_to_uint16_t(&buffer[2]);
    return NO_ERROR;
}

int16_t stc3x_forced_recalibration(uint16_t reference_concentration) {
    int16_t error;
    uint8_t buffer[5];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3661);

    offset = sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset,
                                                  reference_concentration);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    _delay_cycles(1600000); // 100ms delay ... //sensirion_i2c_hal_sleep_usec(66000);
    return NO_ERROR;
}

int16_t stc3x_enable_automatic_self_calibration(void) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3FEF);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(1000);
    return NO_ERROR;
}

int16_t stc3x_disable_automatic_self_calibration(void) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3F6E);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(1000);
    return NO_ERROR;
}

int16_t stc3x_prepare_read_state(void) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3752);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(1000);
    return NO_ERROR;
}

int16_t stc3x_set_sensor_state(const uint8_t* state, uint8_t state_size) {
    uint8_t buffer[47];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0xE133);

    offset = sensirion_i2c_add_bytes_to_buffer(&buffer[0], offset, state,
                                               state_size);

    return sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
}

int16_t stc3x_get_sensor_state(uint8_t* state, uint8_t state_size) {
    int16_t error;
    uint8_t buffer[45];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0xE133);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(0);

    error = sensirion_i2c_read_data_inplace(STC3X_I2C_ADDRESS, &buffer[0], 30);
    if (error) {
        return error;
    }
    sensirion_common_copy_bytes(&buffer[0], state, state_size);
    return NO_ERROR;
}

int16_t stc3x_apply_state(void) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3650);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(1000);
    return NO_ERROR;
}

int16_t stc3x_self_test(uint16_t* self_test_output) {
    int16_t error;
    uint8_t buffer[3];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x365B);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(22000);

    error = sensirion_i2c_read_data_inplace(STC3X_I2C_ADDRESS, &buffer[0], 2);
    if (error) {
        return error;
    }
    *self_test_output = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    return NO_ERROR;
}

int16_t stc3x_enter_sleep_mode(void) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3677);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(1000);
    return NO_ERROR;
}

int16_t stc3x_prepare_product_identifier(void) {
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x367C);

    return sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
}

int16_t stc3x_read_product_identifier(uint32_t* product_number,
                                      uint8_t* serial_number,
                                      uint8_t serial_number_size) {
    int16_t error;
    uint8_t buffer[18];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0xE102);

    error = sensirion_i2c_write_data(STC3X_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(10000);

    error = sensirion_i2c_read_data_inplace(STC3X_I2C_ADDRESS, &buffer[0], 12);
    if (error) {
        return error;
    }
    *product_number = sensirion_common_bytes_to_uint32_t(&buffer[0]);
    sensirion_common_copy_bytes(&buffer[4], serial_number, serial_number_size);
    return NO_ERROR;
}
