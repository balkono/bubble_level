//MIT License
//
//Copyright (c) 2024 Moritz Emersberger
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#ifndef ICM42688_H
#define ICM42688_H


#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>
#include "stm32l4xx_hal.h"

#include "myHAL.h"

#define ICM42688_ReceiveBuffer(buf, len) I2C_RXBuffer(buf, len)
#define ICM42688_SendBuffer(buf, len) I2C_TXBuffer(buf, len)

typedef union databuffer
{
    uint8_t twob[2];
    uint8_t threeb[3];
}DATABUF;

typedef struct caldata
{
    double gyrox;
    double gyroy;
    double accx;
    double accy;
    double accz;
}CALDATA;

typedef struct convdata
{
    int16_t gyrox;
    int16_t gyroy;
    int16_t accx;
    int16_t accy;
    int16_t accz;
}CONVDATA;

typedef struct rawdata
{
    uint8_t gyrox[2];
    uint8_t gyroy[2];
    uint8_t accx[2];
    uint8_t accy[2];
    uint8_t accz[2];
}RAWDATA;

typedef struct dataset
{
    RAWDATA raw;
    CONVDATA conv;
}DATASET;

typedef struct realaccgyr
{
    float accx, accy, accz, gyrx, gyry;
}REALACCGYR;

#define ICM42688_REG_BANK_SEL 0x76
#define BANK0 0x00
#define BANK1 0x01
#define BANK2 0x02
#define BANK3 0x03
#define BANK4 0x04

#define ICM42688_B0_DEVICE_CONFIG         0x11
#define ICM42688_B0_DRIVE_CONFIG          0x13
#define ICM42688_B0_INT_CONFIG            0x14
#define ICM42688_B0_FIFO_CONFIG           0x16
#define ICM42688_B0_TEMP_DATA1            0x1D
#define ICM42688_B0_TEMP_DATA0            0x1E
#define ICM42688_B0_ACCEL_DATA_X1         0x1F
#define ICM42688_B0_ACCEL_DATA_X0         0x20
#define ICM42688_B0_ACCEL_DATA_Y1         0x21
#define ICM42688_B0_ACCEL_DATA_Y0         0x22
#define ICM42688_B0_ACCEL_DATA_Z1         0x23
#define ICM42688_B0_ACCEL_DATA_Z0         0x24
#define ICM42688_B0_GYRO_DATA_X1          0x25
#define ICM42688_B0_GYRO_DATA_X0          0x26
#define ICM42688_B0_GYRO_DATA_Y1          0x27
#define ICM42688_B0_GYRO_DATA_Y0          0x28
#define ICM42688_B0_GYRO_DATA_Z1          0x29
#define ICM42688_B0_GYRO_DATA_Z0          0x2A
#define ICM42688_B0_TMST_FSYNCH           0x2B
#define ICM42688_B0_TMST_FSYNCL           0x2C
#define ICM42688_B0_INT_STATUS            0x2D
#define ICM42688_B0_FIFO_COUNTH           0x2E
#define ICM42688_B0_FIFO_COUNTL           0x2F
#define ICM42688_B0_FIFO_DATA             0x30
#define ICM42688_B0_APEX_DATA0            0x31
#define ICM42688_B0_APEX_DATA1            0x32
#define ICM42688_B0_APEX_DATA2            0x33
#define ICM42688_B0_APEX_DATA3            0x34
#define ICM42688_B0_APEX_DATA4            0x35
#define ICM42688_B0_APEX_DATA5            0x36
#define ICM42688_B0_INT_STATUS2           0x37
#define ICM42688_B0_INT_STATUS3           0x38
#define ICM42688_B0_SIGNAL_PATH_RESET     0x4B
#define ICM42688_B0_INTF_CONFIG0          0x4C
#define ICM42688_B0_INTF_CONFIG1          0x4D
#define ICM42688_B0_PWR_MGMT0             0x4E
#define ICM42688_B0_GYRO_CONFIG0          0x4F
#define ICM42688_B0_ACCEL_CONFIG0         0x50
#define ICM42688_B0_GYRO_CONFIG1          0x51
#define ICM42688_B0_GYRO_ACCEL_CONFIG0    0x52
#define ICM42688_B0_ACCEL_CONFIG1         0x53
#define ICM42688_B0_TMST_CONFIG           0x54
#define ICM42688_B0_APEX_CONFIG0          0x56
#define ICM42688_B0_SMD_CONFIG            0x57
#define ICM42688_B0_FIFO_CONFIG1          0x5F
#define ICM42688_B0_FIFO_CONFIG2          0x60
#define ICM42688_B0_FIFO_CONFIG3          0x61
#define ICM42688_B0_FSYNC_CONFIG          0x62
#define ICM42688_B0_INT_CONFIG0           0x63
#define ICM42688_B0_INT_CONFIG1           0x64
#define ICM42688_B0_INT_SOURCE0           0x65
#define ICM42688_B0_INT_SOURCE1           0x66
#define ICM42688_B0_INT_SOURCE3           0x68
#define ICM42688_B0_INT_SOURCE4           0x69
#define ICM42688_B0_FIFO_LOST_PKT0        0x6C
#define ICM42688_B0_FIFO_LOST_PKT1        0x6D
#define ICM42688_B0_SELF_TEST_CONFIG      0x70
#define ICM42688_B0_WHO_AM_I              0x75

#define ICM42688_B1_SENSOR_CONFIG0        0x03
#define ICM42688_B1_GYRO_CONFIG_STATIC2   0x0B
#define ICM42688_B1_GYRO_CONFIG_STATIC3   0x0C
#define ICM42688_B1_GYRO_CONFIG_STATIC4   0x0D
#define ICM42688_B1_GYRO_CONFIG_STATIC5   0x0E
#define ICM42688_B1_GYRO_CONFIG_STATIC6   0x0F
#define ICM42688_B1_GYRO_CONFIG_STATIC7   0x10
#define ICM42688_B1_GYRO_CONFIG_STATIC8   0x11
#define ICM42688_B1_GYRO_CONFIG_STATIC9   0x12
#define ICM42688_B1_GYRO_CONFIG_STATIC10  0x13
#define ICM42688_B1_XG_ST_DATA            0x5F
#define ICM42688_B1_YG_ST_DATA            0x60
#define ICM42688_B1_ZG_ST_DATA            0x61
#define ICM42688_B1_TMSTVAL0              0x62
#define ICM42688_B1_TMSTVAL1              0x63
#define ICM42688_B1_TMSTVAL2              0x64
#define ICM42688_B1_INTF_CONFIG4          0x7A
#define ICM42688_B1_INTF_CONFIG5          0x7B
#define ICM42688_B1_INTF_CONFIG6          0x7C

#define ICM42688_B2_ACCEL_CONFIG_STATIC2  0x03
#define ICM42688_B2_ACCEL_CONFIG_STATIC3  0x04
#define ICM42688_B2_ACCEL_CONFIG_STATIC4  0x05
#define ICM42688_B2_XA_ST_DATA            0x3B
#define ICM42688_B2_YA_ST_DATA            0x3C
#define ICM42688_B2_ZA_ST_DATA            0x3D

#define ICM42688_B4_APEX_CONFIG1          0x40
#define ICM42688_B4_APEX_CONFIG2          0x41
#define ICM42688_B4_APEX_CONFIG3          0x42
#define ICM42688_B4_APEX_CONFIG4          0x43
#define ICM42688_B4_APEX_CONFIG5          0x44
#define ICM42688_B4_APEX_CONFIG6          0x45
#define ICM42688_B4_APEX_CONFIG7          0x46
#define ICM42688_B4_APEX_CONFIG8          0x47
#define ICM42688_B4_APEX_CONFIG9          0x48
#define ICM42688_B4_ACCEL_WOM_X_THR       0x4A
#define ICM42688_B4_ACCEL_WOM_Y_THR       0x4B
#define ICM42688_B4_ACCEL_WOM_Z_THR       0x4C
#define ICM42688_B4_INT_SOURCE6           0x4D
#define ICM42688_B4_INT_SOURCE7           0x4E
#define ICM42688_B4_INT_SOURCE8           0x4F
#define ICM42688_B4_INT_SOURCE9           0x50
#define ICM42688_B4_INT_SOURCE10          0x51
#define ICM42688_B4_OFFSET_USER0          0x77
#define ICM42688_B4_OFFSET_USER1          0x78
#define ICM42688_B4_OFFSET_USER2          0x79
#define ICM42688_B4_OFFSET_USER3          0x7A
#define ICM42688_B4_OFFSET_USER4          0x7B
#define ICM42688_B4_OFFSET_USER5          0x7C
#define ICM42688_B4_OFFSET_USER6          0x7D
#define ICM42688_B4_OFFSET_USER7          0x7E
#define ICM42688_B4_OFFSET_USER8          0x7F

void ICM42688_init();
void ICM42688_calibrate(CALDATA*);
void ICM42688_reg_config(void);

void ICM42688_read_WHOAMI(void);

void ICM42688_read_all_sensors(RAWDATA*, CONVDATA*);
void ICM42688_print_gyro_debug(DATASET*, CALDATA*);
void ICM42688_print_acc_debug(DATASET*, CALDATA*);
void ICM42688_print_gyro(CONVDATA*, CALDATA*);
void ICM42688_print_acc(CONVDATA*, CALDATA*);
#endif /* ICM42688_H */
