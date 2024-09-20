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

#include "icm42688.h"

static DATABUF databuffer;

double x_gfin;
double y_gfin;
double z_gfin;
double x_afin;
double y_afin;
double z_afin;

#define DATABUF_2b databuffer.twob
#define DATABUF_3b databuffer.threeb


int16_t twosCompToDec(int16_t* data)
{
    // [0x0000; 0x7FFF] corresponds to [0; 32,767]
    // [0x8000; 0xFFFF] corresponds to [-32,768; -1]
    // int16_t has the range [-32,768; 32,767]

    // if positive
    if ( (*data & 0x8000) == 0 ) {
        return *data;
    //  if negative
    } else {
        // invert all bits, add one, and make negative
        return -(~*data + 1);
    }
}

void ICM42688_write_buffer(uint8_t* buf, uint32_t len)
{
    ICM42688_SendBuffer(buf, len);
}

void ICM42688_read_buffer(uint8_t* buf, uint32_t len)
{
    ICM42688_ReceiveBuffer(buf, len);
}

void ICM42688_write_adr(uint8_t adr)
{
    DATABUF_2b[0] = adr;

    ICM42688_write_buffer(DATABUF_2b, 1);
}

void ICM42688_write_adr_1b(uint8_t adr, uint8_t data)
{
    DATABUF_2b[0] = adr;
    DATABUF_2b[1] = data;
    ICM42688_write_buffer(DATABUF_2b, 2);
}

void ICM42688_write_adr_2b(uint8_t adr, uint8_t data1, uint8_t data2)
{
    DATABUF_3b[0] = adr;
    DATABUF_3b[1] = data1;
    DATABUF_3b[2] = data2;
    ICM42688_write_buffer(DATABUF_3b, 3);
}

void ICM42688_write_read_reg(uint8_t adr, uint8_t data)
{
    ICM42688_write_adr_1b(adr, data);
    ICM42688_write_adr(adr);
    ICM42688_read_buffer(DATABUF_2b, 1);
#if DEBUG
    if(DATABUF_2b[1]==data)
    {
        //printf("\rwr. of 0x%x to 0x%x successfull\n\r", data, adr);
    }
    else
    {
        printf("\rICM42688: wr. of 0x%x to 0x%x not successfull\n readback: 0x%x\n\r", data, adr, DATABUF_2b[0]);
    }
#endif
}

void ICM42688_init()
{
    ICM42688_reg_config();
}

void ICM42688_read_sensor(uint8_t* buf, uint8_t addr )
{
    ICM42688_write_adr(addr);
    ICM42688_read_buffer(&buf[1], 1);
    ICM42688_write_adr((addr+1));
    ICM42688_read_buffer(&buf[0], 1);
}

void ICM42688_read_all_sensors(RAWDATA* rawset, CONVDATA* convset)
{
    ICM42688_read_sensor(rawset->gyrox, ICM42688_B0_GYRO_DATA_X1 );
    ICM42688_read_sensor(rawset->gyroy, ICM42688_B0_GYRO_DATA_Y1 );
    ICM42688_read_sensor(rawset->accx, ICM42688_B0_ACCEL_DATA_X1 );
    ICM42688_read_sensor(rawset->accy, ICM42688_B0_ACCEL_DATA_Y1 );
    ICM42688_read_sensor(rawset->accz, ICM42688_B0_ACCEL_DATA_Z1 );

    convset->gyrox = twosCompToDec((int16_t*)rawset->gyrox);
    convset->gyroy = twosCompToDec((int16_t*)rawset->gyroy);
    convset->accx = twosCompToDec((int16_t*)rawset->accx);
    convset->accy = twosCompToDec((int16_t*)rawset->accy);
    convset->accz = twosCompToDec((int16_t*)rawset->accz);
}

void ICM42688_print_acc_debug(DATASET* set, CALDATA* cal)
{
    //set->accx.conv = twosCompToDec((int16_t*)set->accx.raw);
    //set->accy.conv = twosCompToDec((int16_t*)set->accy.raw);
    //set->accz.conv = twosCompToDec((int16_t*)set->accz.raw);
    x_afin = (float)set->conv.accx/16384 - cal->accx;
    y_afin = (float)set->conv.accy/16384 - cal->accy;
    z_afin = (float)set->conv.accz/16384 - cal->accz;

    //printf("x-conv: %.3f\r\n", x_fin);
    //printf("acc: x in g: %f\tx-conv: %.3f\t 2scompliment: 0x%x%x\t\r\n", x_afin, (double)set->accx.conv/16384, set->accx.raw[1], set->accx.raw[0]);
    printf("acc: x,y, z in g: %f\t %f \t %f\r\n", x_afin, y_afin, z_afin);
        //printf("y-conv: %.3f\t 2scompliment: 0x%x%x\t", y_fin, gyroy.raw[1], gyroy.raw[0]);
    //printf("z-conv: %.3f\t 2scompliment: 0x%x%x\r\n", z_fin, gyroz.raw[1], gyroz.raw[0]);
}

void ICM42688_print_gyro_debug(DATASET* set, CALDATA* cal)
{
    //set->gyrox.conv = twosCompToDec((int16_t*)set->gyrox.raw);
    //set->gyroy.conv = twosCompToDec((int16_t*)set->gyroy.raw);
    x_gfin = (float)set->conv.gyrox/131 - cal->gyrox;
    y_gfin = (float)set->conv.gyroy/131 - cal->gyroy;

    //z_gfin = (double)gyroz.conv/32.8 - z_gyr;
    printf("gyr: x,y: \t%f\t %f \t \r\n", x_gfin, y_gfin);
    //printf("x-conv: %.3f\r\n", x_fin);
    //printf("gyr: x-real: %f\tx-conv: %.3f\t 2scompliment: 0x%x%x\t\r\n", x_gfin, (double)set->gyrox.conv/131, set->gyrox.raw[1], set->gyrox.raw[0]);
    //printf("y-conv: %.3f\t 2scompliment: 0x%x%x\t", y_fin, gyroy.raw[1], gyroy.raw[0]);
    //printf("z-conv: %.3f\t 2scompliment: 0x%x%x\r\n", z_fin, gyroz.raw[1], gyroz.raw[0]);
}

void ICM42688_print_acc(CONVDATA* set, CALDATA* cal)
{
    x_afin = (float)set->accx/16384 - cal->accx;
    y_afin = (float)set->accy/16384 - cal->accy;
    z_afin = (float)set->accz/16384 - cal->accz;
    printf("acc: x,y, z in g: %f\t %f \t %f\r\n", x_afin, y_afin, z_afin);
}

void ICM42688_print_gyro(CONVDATA* set, CALDATA* cal)
{
    x_gfin = (float)set->gyrox/131 - cal->gyrox;
    y_gfin = (float)set->gyroy/131 - cal->gyroy;
    printf("gyr: x,y: \t%f\t %f \t \r\n", x_gfin, y_gfin);
}

void ICM42688_read_WHOAMI(void)
{
    ICM42688_write_adr(ICM42688_B0_WHO_AM_I);
    ICM42688_read_buffer(DATABUF_2b, 1);
    printf("\rICM42688: whoami %x\r\n\n", DATABUF_2b[0]);
}

void ICM42688_reg_config(void)
{
    ICM42688_write_read_reg(ICM42688_REG_BANK_SEL,          BANK0);
#if DEBUG
    ICM42688_read_WHOAMI();
#endif
    ICM42688_write_read_reg(ICM42688_B0_DEVICE_CONFIG,       0x00);//spimode0and3, no sw reset
    ICM42688_write_read_reg(ICM42688_B0_DRIVE_CONFIG,        0x09);//i2c slewrate to reset 20ns-60ns
    ICM42688_write_read_reg(ICM42688_B0_INT_CONFIG,          0x00);//int pins config
    ICM42688_write_read_reg(ICM42688_B0_FIFO_CONFIG,         0x00);//fifo bypass mode default
    ICM42688_write_read_reg(ICM42688_B0_SIGNAL_PATH_RESET,   0x2E);//reset dmp memory, restart odr cnt and signal path controls, latch of timestamp, flush of fifo
    ICM42688_write_read_reg(ICM42688_B0_INTF_CONFIG0,        0x32); //diable spi
    ICM42688_write_read_reg(ICM42688_B0_INTF_CONFIG1,        0x01); //default pll when available, else rc osc
    ICM42688_write_read_reg(ICM42688_B0_PWR_MGMT0,           0x2F); //temp-disabled, gyro, accel, enabled, pause of 200us required
    HAL_Delay(50);
    ICM42688_write_read_reg(ICM42688_B0_SELF_TEST_CONFIG,    0x00);
    ICM42688_write_read_reg(ICM42688_B0_GYRO_CONFIG0,        0x08);//gyro_fs_sel to +-2000dps, 100Hz samplingrate
    ICM42688_write_read_reg(ICM42688_B0_ACCEL_CONFIG0,       0x68);//accel_fs_sel to +-2g, 100Hz samplingrate
    ICM42688_write_read_reg(ICM42688_B0_GYRO_CONFIG1,        0x06);//gyro_ui_filt 2ndorder, reserved for dec2_m2
    ICM42688_write_read_reg(ICM42688_B0_GYRO_ACCEL_CONFIG0,  0x00);//accel_ui_filt_bw odr/2, gyro_ui_filt_bw odr/2
    ICM42688_write_read_reg(ICM42688_B0_ACCEL_CONFIG1,       0x0D);//accel filt 2ndorder, reserved for dec2_m2 filter
    ICM42688_write_read_reg(ICM42688_B0_TMST_CONFIG,         0x03);
    ICM42688_write_read_reg(ICM42688_B0_APEX_CONFIG0,        0x82);
    ICM42688_write_read_reg(ICM42688_B0_SMD_CONFIG,          0x00);
    ICM42688_write_read_reg(ICM42688_B0_FIFO_CONFIG1,        0x00);
    ICM42688_write_read_reg(ICM42688_B0_FIFO_CONFIG2,        0x00);
    ICM42688_write_read_reg(ICM42688_B0_FIFO_CONFIG3,        0x00);
    ICM42688_write_read_reg(ICM42688_B0_FSYNC_CONFIG,        0x00);
    ICM42688_write_read_reg(ICM42688_B0_INT_CONFIG0,         0x00);
    ICM42688_write_read_reg(ICM42688_B0_INT_CONFIG1,         0x00);
    ICM42688_write_read_reg(ICM42688_B0_INT_SOURCE0,         0x00);
    ICM42688_write_read_reg(ICM42688_B0_INT_SOURCE1,         0x00);
    ICM42688_write_read_reg(ICM42688_B0_INT_SOURCE3,         0x00);
    ICM42688_write_read_reg(ICM42688_B0_INT_SOURCE4,         0x00);

    ICM42688_write_read_reg(ICM42688_REG_BANK_SEL,          BANK1);
    ICM42688_write_read_reg(ICM42688_B1_SENSOR_CONFIG0,      0x80);//default value
    ICM42688_write_read_reg(ICM42688_B1_GYRO_CONFIG_STATIC2, 0x03);//anti-aliasing filter, notch filter disabled
    ICM42688_write_read_reg(ICM42688_B1_GYRO_CONFIG_STATIC3, 0x0D);//see 5.2 for details reset
    ICM42688_write_read_reg(ICM42688_B1_GYRO_CONFIG_STATIC4, 0xAA);//see 5.2 for details reset
    ICM42688_write_read_reg(ICM42688_B1_GYRO_CONFIG_STATIC5, 0x80);//see 5.2 for details reset
    //ICM42688_write_read_reg(ICM42688_B1_GYRO_CONFIG_STATIC6, 0x00);//factory trimmed, 5.1
//    ICM42688_write_read_reg(ICM42688_B1_GYRO_CONFIG_STATIC7, 0x00);//factory trimmed, 5.1
//    ICM42688_write_read_reg(ICM42688_B1_GYRO_CONFIG_STATIC8, 0x00);//factory trimmed, 5.1
//    ICM42688_write_read_reg(ICM42688_B1_GYRO_CONFIG_STATIC9, 0x00);//factory trimmed, 5.1
    ICM42688_write_read_reg(ICM42688_B1_GYRO_CONFIG_STATIC10,0x11);//default
    ICM42688_write_read_reg(ICM42688_B1_INTF_CONFIG4,        0x00);//I2C and I3C devices, 3-wire spi
    ICM42688_write_read_reg(ICM42688_B1_INTF_CONFIG5,        0x00);//default
    ICM42688_write_read_reg(ICM42688_B1_INTF_CONFIG6,        0x10);//as mentioned in datasheet
    
    ICM42688_write_read_reg(ICM42688_REG_BANK_SEL,          BANK2);
    ICM42688_write_read_reg(ICM42688_B2_ACCEL_CONFIG_STATIC2,0x01);//disable accel anti-aliasing filter
    ICM42688_write_read_reg(ICM42688_B2_ACCEL_CONFIG_STATIC3,0x40);//default
    ICM42688_write_read_reg(ICM42688_B2_ACCEL_CONFIG_STATIC4,0x62);
    
    ICM42688_write_read_reg(ICM42688_REG_BANK_SEL,          BANK4);
    ICM42688_write_read_reg(ICM42688_B4_APEX_CONFIG1,        0xA2);
    ICM42688_write_read_reg(ICM42688_B4_APEX_CONFIG2,        0x85);
    ICM42688_write_read_reg(ICM42688_B4_APEX_CONFIG3,        0x51);
    ICM42688_write_read_reg(ICM42688_B4_APEX_CONFIG4,        0xA4);
    ICM42688_write_read_reg(ICM42688_B4_APEX_CONFIG5,        0x8C);
    ICM42688_write_read_reg(ICM42688_B4_APEX_CONFIG6,        0x5C);
    ICM42688_write_read_reg(ICM42688_B4_APEX_CONFIG7,        0x45);
    ICM42688_write_read_reg(ICM42688_B4_APEX_CONFIG8,        0x5B);
    ICM42688_write_read_reg(ICM42688_B4_APEX_CONFIG9,        0x00);
    ICM42688_write_read_reg(ICM42688_B4_ACCEL_WOM_X_THR,     0x00);
    ICM42688_write_read_reg(ICM42688_B4_ACCEL_WOM_Y_THR,     0x00);
    ICM42688_write_read_reg(ICM42688_B4_ACCEL_WOM_Z_THR,     0x00);
    ICM42688_write_read_reg(ICM42688_B4_INT_SOURCE6,         0x00);
    ICM42688_write_read_reg(ICM42688_B4_INT_SOURCE7,         0x00);
    ICM42688_write_read_reg(ICM42688_B4_INT_SOURCE8,         0x00);
    ICM42688_write_read_reg(ICM42688_B4_INT_SOURCE9,         0x00);
    ICM42688_write_read_reg(ICM42688_B4_INT_SOURCE10,        0x00);
    ICM42688_write_read_reg(ICM42688_B4_OFFSET_USER0,        0x00);
    ICM42688_write_read_reg(ICM42688_B4_OFFSET_USER1,        0x00);
    ICM42688_write_read_reg(ICM42688_B4_OFFSET_USER2,        0x00);
    ICM42688_write_read_reg(ICM42688_B4_OFFSET_USER3,        0x00);
    ICM42688_write_read_reg(ICM42688_B4_OFFSET_USER4,        0x00);
    ICM42688_write_read_reg(ICM42688_B4_OFFSET_USER5,        0x00);
    ICM42688_write_read_reg(ICM42688_B4_OFFSET_USER6,        0x00);
    ICM42688_write_read_reg(ICM42688_B4_OFFSET_USER7,        0x00);
    ICM42688_write_read_reg(ICM42688_B4_OFFSET_USER8,        0x00);
    ICM42688_write_read_reg(ICM42688_REG_BANK_SEL,          BANK0);
}
