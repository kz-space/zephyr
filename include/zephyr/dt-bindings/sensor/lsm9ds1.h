/*
 * Copyright (c) 2024 Bootlin
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_ST_LSM9DS1_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_ST_LSM9DS1_H_

/* Accel range */
#define LSM9DS1_DT_FS_2G			0
#define LSM9DS1_DT_FS_16G			1
#define LSM9DS1_DT_FS_4G			2
#define LSM9DS1_DT_FS_8G			3

#define LSM9DS1_DT_FS_245DPS		0
#define LSM9DS1_DT_FS_500DPS		1
#define LSM9DS1_DT_FS_2000DPS		3

#define LSM9DS1_IMU_OFF             0x00
#define LSM9DS1_GY_OFF_XL_10Hz      0x10
#define LSM9DS1_GY_OFF_XL_50Hz      0x20
#define LSM9DS1_GY_OFF_XL_119Hz     0x30
#define LSM9DS1_GY_OFF_XL_238Hz     0x40
#define LSM9DS1_GY_OFF_XL_476Hz     0x50
#define LSM9DS1_GY_OFF_XL_952Hz     0x60
#define LSM9DS1_XL_OFF_GY_14Hz9     0x01
#define LSM9DS1_XL_OFF_GY_59Hz5     0x02
#define LSM9DS1_XL_OFF_GY_119Hz     0x03
#define LSM9DS1_XL_OFF_GY_238Hz     0x04
#define LSM9DS1_XL_OFF_GY_476Hz     0x05
#define LSM9DS1_XL_OFF_GY_952Hz     0x06
#define LSM9DS1_IMU_14Hz9           0x11
#define LSM9DS1_IMU_59Hz5           0x22
#define LSM9DS1_IMU_119Hz           0x33
#define LSM9DS1_IMU_238Hz           0x44
#define LSM9DS1_IMU_476Hz           0x55
#define LSM9DS1_IMU_952Hz           0x66
#define LSM9DS1_XL_OFF_GY_14Hz9_LP  0x81
#define LSM9DS1_XL_OFF_GY_59Hz5_LP  0x82
#define LSM9DS1_XL_OFF_GY_119Hz_LP  0x83
#define LSM9DS1_IMU_14Hz9_LP        0x91
#define LSM9DS1_IMU_59Hz5_LP        0xA2
#define LSM9DS1_IMU_119Hz_LP        0xB3

/* magnetometer */

#define LSM9DS1_DT_FS_4Ga			0
#define LSM9DS1_DT_FS_8Ga			1
#define LSM9DS1_DT_FS_12Ga			2
#define LSM9DS1_DT_FS_16Ga			3

#define LSM9DS1_MAG_POWER_DOWN      0xC0
#define LSM9DS1_MAG_LP_0Hz625       0x00
#define LSM9DS1_MAG_LP_1Hz25        0x01
#define LSM9DS1_MAG_LP_2Hz5         0x02
#define LSM9DS1_MAG_LP_5Hz          0x03
#define LSM9DS1_MAG_LP_10Hz         0x04
#define LSM9DS1_MAG_LP_20Hz         0x05
#define LSM9DS1_MAG_LP_40Hz         0x06
#define LSM9DS1_MAG_LP_80Hz         0x07
#define LSM9DS1_MAG_MP_0Hz625       0x10
#define LSM9DS1_MAG_MP_1Hz25        0x11
#define LSM9DS1_MAG_MP_2Hz5         0x12
#define LSM9DS1_MAG_MP_5Hz          0x13
#define LSM9DS1_MAG_MP_10Hz         0x14
#define LSM9DS1_MAG_MP_20Hz         0x15
#define LSM9DS1_MAG_MP_40Hz         0x16
#define LSM9DS1_MAG_MP_80Hz         0x17
#define LSM9DS1_MAG_HP_0Hz625       0x20
#define LSM9DS1_MAG_HP_1Hz25        0x21
#define LSM9DS1_MAG_HP_2Hz5         0x22
#define LSM9DS1_MAG_HP_5Hz          0x23
#define LSM9DS1_MAG_HP_10Hz         0x24
#define LSM9DS1_MAG_HP_20Hz         0x25
#define LSM9DS1_MAG_HP_40Hz         0x26
#define LSM9DS1_MAG_HP_80Hz         0x27
#define LSM9DS1_MAG_UHP_0Hz625      0x30
#define LSM9DS1_MAG_UHP_1Hz25       0x31
#define LSM9DS1_MAG_UHP_2Hz5        0x32
#define LSM9DS1_MAG_UHP_5Hz         0x33
#define LSM9DS1_MAG_UHP_10Hz        0x34
#define LSM9DS1_MAG_UHP_20Hz        0x35
#define LSM9DS1_MAG_UHP_40Hz        0x36
#define LSM9DS1_MAG_UHP_80Hz        0x37
#define LSM9DS1_MAG_UHP_155Hz       0x38
#define LSM9DS1_MAG_HP_300Hz        0x28
#define LSM9DS1_MAG_MP_560Hz        0x18
#define LSM9DS1_MAG_LP_1000Hz       0x08
#define LSM9DS1_MAG_ONE_SHOT        0x70

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_ST_LSM9DS1_H_ */
