#include "mbed.h"
#include "MPU6050.h"

#define ADDRESS 0xD1

MPU6050::MPU6050(PinName sda, PinName scl):
    _MPU6050(sda, scl)
{
    _MPU6050.frequency(400000);
}

void MPU6050::start(void)
{
    write_reg(ADDRESS,MPU6050_PWR_MGMT_1,0x00);//disable sleep mode

   write_reg(ADDRESS,MPU6050_GYRO_CONFIG,0b00000000);//gyro +-250deg/s 131f
//    write_reg(ADDRESS,MPU6050_GYRO_CONFIG,0b00001000);//gyro +-500deg/s 65.5f
//    write_reg(ADDRESS,MPU6050_GYRO_CONFIG,0b00010000);//gyro +-1000deg/s 32.8f
    // write_reg(ADDRESS,MPU6050_GYRO_CONFIG,0b00011000);//gyro +-2000deg/s 16.4f

    write_reg(ADDRESS,MPU6050_ACCEL_CONFIG,0x08);//accel +-16G 0x08 0b00011000
    write_reg(ADDRESS,MPU6050_SMPLRT_DIV,0x10);//sample rate 470Hz
    //to set more, see MPU6000 Register Map
}

char MPU6050::getID(void)
{
    char devID;
    read_reg(ADDRESS,MPU6050_WHO_AM_I,&devID);
    return devID;
}

bool MPU6050::read(float *gx, float *gy, float *gz,float *ax, float *ay, float *az)
{
    char data[6];
    char data2[6];
    if (read_data(ADDRESS, MPU6050_GYRO_XOUT_H, data, 6)) {
        read_data(ADDRESS, MPU6050_ACCEL_XOUT_H, data2, 6);
       *gx = float(short(data[0] << 8 | data[1]))/131.0f; // for 250
       *gy =  float(short(data[2] << 8 | data[3]))/131.0f;
       *gz =  float(short(data[4] << 8 | data[5]))/131.0f;

//        *gx = float(short(data[0] << 8 | data[1]))/65.5f; // for 500
//        *gy =  float(short(data[2] << 8 | data[3]))/65.5f;
//        *gz =  float(short(data[4] << 8 | data[5]))/65.5f;

//        *gx = float(short(data[0] << 8 | data[1]))/32.8f; // for 1000
//        *gy =  float(short(data[2] << 8 | data[3]))/32.8f;
//        *gz =  float(short(data[4] << 8 | data[5]))/32.8f;

        // *gx = float(short(data[0] << 8 | data[1]))/16.4f;// for 2000
        // *gy =  float(short(data[2] << 8 | data[3]))/16.4f;
        // *gz =  float(short(data[4] << 8 | data[5]))/16.4f;
        
        *ax = float(short(data2[0] << 8 | data2[1]))*0.000122f;
        *ay =  float(short(data2[2] << 8 | data2[3]))*0.000122f;
        *az =  float(short(data2[4] << 8 | data2[5]))*0.000122f;
        return true;
    }
    return false;
}

bool MPU6050::readraw(int *gx, int *gy, int *gz,int *ax, int *ay, int *az)
{
    char data[6];
    char data2[6];
    if (read_data(ADDRESS, MPU6050_GYRO_XOUT_H, data, 6)) {
        read_data(ADDRESS, MPU6050_ACCEL_XOUT_H, data2, 6);
        *gx = int(short(data[0] << 8 | data[1]));
        *gy =  int(short(data[2] << 8 | data[3]));
        *gz =  int(short(data[4] << 8 | data[5]));
        *ax = int(short(data2[0] << 8 | data2[1]));
        *ay =  int(short(data2[2] << 8 | data2[3]));
        *az =  int(short(data2[4] << 8 | data2[5]));
        return true;
    }
    return false;
}

bool MPU6050::write_reg(int addr_i2c,int addr_reg, char v)
{
    char data[2] = {addr_reg, v};
    return MPU6050::_MPU6050.write(addr_i2c, data, 2) == 0;
}

bool MPU6050::read_reg(int addr_i2c,int addr_reg, char *v)
{
    char data = addr_reg;
    bool result = false;
    if ((_MPU6050.write(addr_i2c, &data, 1) == 0) && (_MPU6050.read(addr_i2c, &data, 1) == 0)) {
        *v = data;
        result = true;
    }
    return result;
}


bool MPU6050::read_data(char sad, char sub, char *buf, int length)
{
    if (length > 1) sub |= 0x80;

    return _MPU6050.write(sad, &sub, 1, true) == 0 && _MPU6050.read(sad, buf, length) == 0;
}
