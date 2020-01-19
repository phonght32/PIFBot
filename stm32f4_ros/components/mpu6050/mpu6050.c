#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_i2c.h"

#include "stdlib.h"
#include "string.h"

#include "include/mpu6050.h"
#include "include/MadgwickAHRS.h"

/* SELF TEST REGISTERS */
#define MPU6050_SELF_TEST_X         0x0D
#define MPU6050_SELF_TEST_Y         0x0E
#define MPU6050_SELF_TEST_Z         0x0F
#define MPU6050_SELF_TEST_A         0x10

/* SAMPLE RATE DIVIDER REGISTERS */
#define MPU6050_SMPLRT_DIV          0x19

/* CONFIGURATION REGISTERS */
#define MPU6050_CONFIG              0x1A

/* GYROSCOPE CONFIGURATION REGISTERS */
#define MPU6050_GYRO_CONFIG         0x1B

/* ACCELEROMETER CONFIGURATION REGISTERS */
#define MPU6050_ACCEL_CONFIG        0x1C

/* FIFO ENABLE REGISTERS */
#define MPU6050_FIFO_EN             0x23

/* I2C MASTER CONTROL REGISTERS */
#define MPU6050_I2C_MST_CTRL        0x24

/* I2C SLAVE 0 CONTROL REGISTERS */
#define MPU6050_I2C_SLV0_ADDR       0x25
#define MPU6050_I2C_SLV0_REG        0x26
#define MPU6050_I2C_SLV0_CTRL       0x27

/* I2C SLAVE 1 CONTROL REGISTERS */
#define MPU6050_I2C_SLV1_ADDR       0x28
#define MPU6050_I2C_SLV1_REG        0x29
#define MPU6050_I2C_SLV1_CTRL       0x2A

/* I2C SLAVE 2 CONTROL REGISTERS */
#define MPU6050_I2C_SLV2_ADDR       0x2B
#define MPU6050_I2C_SLV2_REG        0x2C
#define MPU6050_I2C_SLV2_CTRL       0x2D

/* I2C SLAVE 3 CONTROL REGISTERS */
#define MPU6050_I2C_SLV3_ADDR       0x2E
#define MPU6050_I2C_SLV3_REG        0x2F
#define MPU6050_I2C_SLV3_CTRL       0x30

/* I2C SLAVE 4 CONTROL REGISTERS */
#define MPU6050_I2C_SLV4_ADDR       0x31
#define MPU6050_I2C_SLV4_REG        0x32
#define MPU6050_I2C_SLV4_DO         0x33
#define MPU6050_I2C_SLV4_CTRL       0x34
#define MPU6050_I2C_SLV4_DI         0x35

/* I2C MASTER STATUS REGISTERS */
#define MPU6050_I2C_MST_STATUS      0x36

/* I2C INT PIN/BYPASS ENABLE CONFIGURATION REGISTERS */
#define MPU6050_INT_PIN_CFG         0x37

/* INTERRUPT ENABLE REGISTERS */
#define MPU6050_INT_ENABLE          0x38

/* INTERRUPT STATUS REGISTERS */
#define MPU6050_INT_STATUS          0x3A

/* ACCELEROMETER MEASUREMENTS REGISTERS */
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40

/* TEMPERATURE MEASUREMENT REGISTERS */
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42

/* GYROSCOPE MESUREMENTS REGISTERS */
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46  
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48

/* EXTERNAL SENSOR DATA REGISTERS */
#define MPU6050_EXT_SENS_DATA_00    0x49
#define MPU6050_EXT_SENS_DATA_01    0x4A
#define MPU6050_EXT_SENS_DATA_02    0x4B
#define MPU6050_EXT_SENS_DATA_03    0x4C
#define MPU6050_EXT_SENS_DATA_04    0x4D   
#define MPU6050_EXT_SENS_DATA_05    0x4E
#define MPU6050_EXT_SENS_DATA_06    0x4F
#define MPU6050_EXT_SENS_DATA_07    0x50
#define MPU6050_EXT_SENS_DATA_08    0x51
#define MPU6050_EXT_SENS_DATA_09    0x52
#define MPU6050_EXT_SENS_DATA_10    0x53
#define MPU6050_EXT_SENS_DATA_11    0x54
#define MPU6050_EXT_SENS_DATA_12    0x55
#define MPU6050_EXT_SENS_DATA_13    0x56
#define MPU6050_EXT_SENS_DATA_14    0x57
#define MPU6050_EXT_SENS_DATA_15    0x58
#define MPU6050_EXT_SENS_DATA_16    0x59
#define MPU6050_EXT_SENS_DATA_17    0x5A
#define MPU6050_EXT_SENS_DATA_18    0x5B
#define MPU6050_EXT_SENS_DATA_19    0x5C
#define MPU6050_EXT_SENS_DATA_20    0x5D
#define MPU6050_EXT_SENS_DATA_21    0x5E   
#define MPU6050_EXT_SENS_DATA_22    0x5F
#define MPU6050_EXT_SENS_DATA_23    0x60

/* I2C SLAVE 0 DATA OUT REGISTERS */
#define MPU6050_I2C_SLV0_D0         0x63

/* I2C SLAVE 1 DATA OUT REGISTERS */
#define MPU6050_I2C_SLV1_D0         0x64

/* I2C SLAVE 2 DATA OUT REGISTERS */
#define MPU6050_I2C_SLV2_D0         0x65

/* I2C SLAVE 3 DATA OUT REGISTERS */
#define MPU6050_I2C_SLV3_D0         0x66

/* I2C MASTER DELAY CONTROL REGISTERS */
#define MPU6050_I2C_MST_DELAY_CTRL  0x67

/* SIGNAL PATH RESET REGISTERS */
#define MPU6050_SIGNAL_PATH_RESET   0x68

/* USER CONTROL REGISTERS */
#define MPU6050_USER_CTRL           0x6A

/* POWER MANAGEMENT 1 REGISTERS */
 #define MPU6050_PWR_MGMT_1          0x6B   

/* POWER MANAGEMENT 2 REGISTERS */
#define MPU6050_PWR_MGMT_2          0x6C

/* FIRO COUNT REGISTERS REGISTERS */
#define MPU6050_FIFO_COUNTH         0x72
#define MPU6050_FIRO_COUNTL         0x73

/* FIFO READ WRITE REGISTERS */
#define MPU6050_FIRO_R_W            0x74

/* WHO AM I REGISTERS */
#define MPU6050_WHO_AM_I            0x75

#define MPU6050_ADDR                (0x68<<1)

#define TIMEOUT_MS_DEFAULT          100

static float accelScalingFactor, gyroScalingFactor;

static float accel_x_bias = 0.0;
static float accel_y_bias = 0.0;
static float accel_z_bias = 0.0;

static float gyro_x_bias = 0.0;
static float gyro_y_bias = 0.0;
static float gyro_z_bias = 0.0;

volatile float q0, q1, q2, q3;

I2C_HandleTypeDef mpu6050_i2c_handle;

typedef struct mpu6050 {
    mpu6050_clksel_t        clksel;
    mpu6050_dlpf_cfg_t      dlpf_cfg;
    mpu6050_sleep_mode_t    sleep_mode;
    mpu6050_fs_sel_t        fs_sel;
    mpu6050_afs_sel_t       afs_sel;
} mpu6050_t;

static int mpu6050_write_reg(uint8_t reg_addr, uint8_t data)
{
    uint8_t buf_send[2];
    buf_send[0] = reg_addr;
    buf_send[1] = data;
    HAL_I2C_Master_Transmit(&mpu6050_i2c_handle, MPU6050_ADDR, buf_send, 2, TIMEOUT_MS_DEFAULT);

    return 0;
}

static int mpu6050_read_reg(uint8_t start_reg_addr,uint8_t *buf_rec, uint8_t num_bytes)
{
	uint8_t buffer[1];
	buffer[0] = start_reg_addr;
    HAL_I2C_Master_Transmit(&mpu6050_i2c_handle, MPU6050_ADDR, buffer, 1, 10);
    HAL_I2C_Master_Receive(&mpu6050_i2c_handle, MPU6050_ADDR, buf_rec, num_bytes, 100);

    return 0;
}


int mpu6050_i2c_config(I2C_HandleTypeDef *i2c_handle)
{
	memcpy(&mpu6050_i2c_handle, i2c_handle, sizeof(I2C_HandleTypeDef));

	return 0;
}

mpu6050_handle_t mpu6050_init(mpu6050_config_t *config)
{
	mpu6050_handle_t handle;
	handle = calloc(1, sizeof(mpu6050_t));
	if(handle == NULL)
	{
		return -1;
	}

	handle->afs_sel = config->afs_sel;
	handle->clksel = config->clksel;
	handle->dlpf_cfg = config->dlpf_cfg;
	handle->fs_sel = config->fs_sel;
	handle->sleep_mode = config->sleep_mode;
    uint8_t buffer = 0;

    mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    buffer = config->clksel & 0x07;
    buffer |= (config->sleep_mode << 6) & 0x40;
    mpu6050_write_reg(MPU6050_PWR_MGMT_1, buffer);
    HAL_Delay(100);

    buffer = 0;
    buffer = config->dlpf_cfg & 0x07;
    mpu6050_write_reg(MPU6050_CONFIG, buffer);

    buffer = 0;
    buffer = (config->fs_sel << 3) & 0x18;
    mpu6050_write_reg(MPU6050_GYRO_CONFIG, buffer);

    buffer = 0;
    buffer = (config->afs_sel <<3) & 0x18;
    mpu6050_write_reg(MPU6050_ACCEL_CONFIG, buffer);

    mpu6050_write_reg(MPU6050_SMPLRT_DIV, 0x04);

    switch (config->afs_sel)
    {
        case MPU6050_AFS_SEL_2G:
            accelScalingFactor = (2000.0f/32768.0f);
            break;

        case MPU6050_AFS_SEL_4G:
            accelScalingFactor = (4000.0f/32768.0f);
                break;

        case MPU6050_AFS_SEL_8G:
            accelScalingFactor = (8000.0f/32768.0f);
            break;

        case MPU6050_AFS_SEL_16G:
            accelScalingFactor = (16000.0f/32768.0f);
            break;

        default:
            break;
    }

    switch (config->fs_sel)
    {
        case MPU6050_FS_SEL_250:
            gyroScalingFactor = 250.0f/32768.0f;
            break;

        case MPU6050_FS_SEL_500:
                gyroScalingFactor = 500.0f/32768.0f;
                break;

        case MPU6050_FS_SEL_1000:
            gyroScalingFactor = 1000.0f/32768.0f;
            break;

        case MPU6050_FS_SEL_2000:
            gyroScalingFactor = 2000.0f/32768.0f;
            break;

        default:
            break;
    }

    return handle;
}

int mpu6050_get_accel_raw(mpu6050_raw_data_t *raw_data)
{
    uint8_t buffer, accel_raw_data[6];
    mpu6050_read_reg(MPU6050_INT_STATUS, &buffer, 1);
    if (buffer && 0x01) 
    {
        mpu6050_read_reg(MPU6050_ACCEL_XOUT_H, accel_raw_data, 6);
        raw_data->x_axis = (accel_raw_data[0]<<8) + accel_raw_data[1];
        raw_data->y_axis = (accel_raw_data[2]<<8) + accel_raw_data[3];
        raw_data->z_axis = (accel_raw_data[4]<<8) + accel_raw_data[5];
    }

    return 0;
}

int mpu6050_get_gyro_raw(mpu6050_raw_data_t *raw_data)
{
    uint8_t buffer, gyro_raw_data[6];
    mpu6050_read_reg(MPU6050_INT_STATUS, &buffer, 1);
    if (buffer && 0x01)
    {
        mpu6050_read_reg(MPU6050_GYRO_XOUT_H, gyro_raw_data, 6);
        raw_data->x_axis = (gyro_raw_data[0]<<8) + gyro_raw_data[1];
        raw_data->y_axis = (gyro_raw_data[2]<<8) + gyro_raw_data[3];
        raw_data->z_axis = (gyro_raw_data[4]<<8) + gyro_raw_data[5];
    }

    return 0;
}

int mpu6050_get_accel_scale(mpu6050_scaled_data_t *scale_data)
{
	uint8_t buffer, accel_raw_data[6];
	    mpu6050_read_reg(MPU6050_INT_STATUS, &buffer, 1);
	    if (buffer && 0x01)
	    {
	        mpu6050_read_reg(MPU6050_ACCEL_XOUT_H, accel_raw_data, 6);
	        scale_data->x_axis = ((accel_raw_data[0]<<8) + accel_raw_data[1]+0.0f)*accelScalingFactor;
	        scale_data->y_axis = ((accel_raw_data[2]<<8) + accel_raw_data[3]+0.0f)*accelScalingFactor;
	        scale_data->z_axis = ((accel_raw_data[4]<<8) + accel_raw_data[5]+0.0f)*accelScalingFactor;
	    }

	    return 0;
}

int mpu6050_get_gyro_scale(mpu6050_scaled_data_t *scale_data)
{
	uint8_t buffer, gyro_raw_data[6];
	    mpu6050_read_reg(MPU6050_INT_STATUS, &buffer, 1);
	    if (buffer && 0x01)
	    {
	        mpu6050_read_reg(MPU6050_GYRO_XOUT_H, gyro_raw_data, 6);
	        scale_data->x_axis = ((gyro_raw_data[0]<<8) + gyro_raw_data[1]+0.0f)*gyroScalingFactor;
	        scale_data->y_axis = ((gyro_raw_data[2]<<8) + gyro_raw_data[3]+0.0f)*gyroScalingFactor;
	        scale_data->z_axis = ((gyro_raw_data[4]<<8) + gyro_raw_data[5]+0.0f)*gyroScalingFactor;
	    }

	    return 0;
}

int mpu6050_get_accel_cali(mpu6050_cali_data_t *cali_data)
{
	mpu6050_scaled_data_t scale_data;
	mpu6050_get_accel_scale(&scale_data);

	cali_data->x_axis = scale_data.x_axis - accel_x_bias;
	cali_data->y_axis = scale_data.y_axis - accel_y_bias;
	cali_data->z_axis = scale_data.z_axis - accel_z_bias;

	return 0;
}

int mpu6050_get_gyro_cali(mpu6050_cali_data_t *cali_data)
{
	mpu6050_scaled_data_t scale_data;
	mpu6050_get_gyro_scale(&scale_data);

	cali_data->x_axis = scale_data.x_axis - gyro_x_bias;
	cali_data->y_axis = scale_data.y_axis - gyro_y_bias;
	cali_data->z_axis = scale_data.z_axis - gyro_z_bias;

	return 0;
}

int mpu6050_update_quat(void)
{
	mpu6050_cali_data_t accel_cali, gyro_cali;
	mpu6050_get_accel_cali(&accel_cali);
	mpu6050_get_gyro_cali(&gyro_cali);

	MadgwickAHRSupdateIMU(gyro_cali.x_axis, gyro_cali.y_axis, gyro_cali.z_axis, accel_cali.x_axis, accel_cali.y_axis, accel_cali.z_axis);

	return 0;
}

int mpu6050_get_quat(mpu6050_quat_data_t *quat)
{
	quat->q0 = q0;
	quat->q1 = q1;
	quat->q2 = q2;
	quat->q3 = q3;

	return 0;
}


