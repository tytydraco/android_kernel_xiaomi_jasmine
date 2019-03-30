/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 * Copyright (C) 2018 XiaoMi, Inc.
 *
 * $Revision: 12034 $
 * $Date: 2017-05-04 17:49:58 +0800 (周四, 04 五月 2017) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include "nt36xxx.h"

#if BOOT_UPDATE_FIRMWARE
#define FW_BIN_SIZE_116KB 118784
#define FW_BIN_SIZE FW_BIN_SIZE_116KB
#define FW_BIN_VER_OFFSET 0x1A000
#define FW_BIN_VER_BAR_OFFSET 0x1A001
#define FLASH_SECTOR_SIZE 4096
#define SIZE_64KB 65536
#define BLOCK_64KB_NUM 4

const struct firmware *fw_entry = NULL;

int32_t update_firmware_request(char *filename)
{
	int32_t ret = 0;

	if (NULL == filename)
		return -EPERM;

	ret = request_firmware(&fw_entry, filename, &ts->client->dev);
	if (ret)
		return ret;

	if (fw_entry->size != FW_BIN_SIZE)
		return -EINVAL;

	if (*(fw_entry->data + FW_BIN_VER_OFFSET) + *(fw_entry->data + FW_BIN_VER_BAR_OFFSET) != 0xFF)
		return -EINVAL;

	return 0;
}

void update_firmware_release(void)
{
	if (fw_entry)
		release_firmware(fw_entry);
	fw_entry = NULL;
}

int32_t Check_FW_Ver(void)
{
	uint8_t buf[16] = {0};
	int32_t ret = 0;

	buf[0] = 0xFF;
	buf[1] = (ts->mmap->EVENT_BUF_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->EVENT_BUF_ADDR >> 8) & 0xFF;
	ret = CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);
	if (ret < 0)
		return ret;

	buf[0] = EVENT_MAP_FWINFO;
	buf[1] = 0x00;
	buf[2] = 0x00;
	ret = CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 3);
	if (ret < 0)
		return ret;

	if ((buf[1] + buf[2]) != 0xFF)
		return 0;

	if (buf[1] > fw_entry->data[FW_BIN_VER_OFFSET])
		return 1;
	else
		return 0;
}
#endif

int32_t Resume_PD(void)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	buf[0] = 0x00;
	buf[1] = 0xAB;
	ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
	if (ret < 0)
		return ret;

	retry = 0;
	for (;;) {
		msleep(1);
		buf[0] = 0x00;
		buf[1] = 0x00;
		ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0)
			return ret;
		if (buf[1] == 0xAA)
			break;
		retry++;
		if (unlikely(retry > 20))
			return -EPERM;
	}
	msleep(10);

	return 0;
}
#if BOOT_UPDATE_FIRMWARE
int32_t Check_CheckSum(void)
{
	uint8_t buf[64] = {0};
	uint32_t XDATA_Addr = ts->mmap->READ_FLASH_CHECKSUM_ADDR;
	int32_t ret = 0;
	int32_t i = 0;
	int32_t k = 0;
	uint16_t WR_Filechksum[BLOCK_64KB_NUM] = {0};
	uint16_t RD_Filechksum[BLOCK_64KB_NUM] = {0};
	size_t fw_bin_size = 0;
	size_t len_in_blk = 0;
	int32_t retry = 0;

	if (Resume_PD())
		return -1;

	fw_bin_size = fw_entry->size;

	for (i = 0; i < BLOCK_64KB_NUM; i++) {
		if (fw_bin_size > (i * SIZE_64KB)) {
			len_in_blk = min(fw_bin_size - i * SIZE_64KB, (size_t)SIZE_64KB);
			WR_Filechksum[i] = i + 0x00 + 0x00 + (((len_in_blk - 1) >> 8) & 0xFF) + ((len_in_blk - 1) & 0xFF);
			for (k = 0; k < len_in_blk; k++)
				WR_Filechksum[i] += fw_entry->data[k + i * SIZE_64KB];
			WR_Filechksum[i] = 65535 - WR_Filechksum[i] + 1;

			buf[0] = 0x00;
			buf[1] = 0x07;
			buf[2] = i;
			buf[3] = 0x00;
			buf[4] = 0x00;
			buf[5] = ((len_in_blk - 1) >> 8) & 0xFF;
			buf[6] = (len_in_blk - 1) & 0xFF;
			ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 7);
			if (ret < 0)
				return ret;

			retry = 0;
			for (;;) {
				msleep(80);
				buf[0] = 0x00;
				buf[1] = 0x00;
				ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 2);
				if (ret < 0)
					return ret;
				if (buf[1] == 0xAA)
					break;
				retry++;
				if (unlikely(retry > 5))
					return -EPERM;
			}

			buf[0] = 0xFF;
			buf[1] = XDATA_Addr >> 16;
			buf[2] = (XDATA_Addr >> 8) & 0xFF;
			ret = CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);
			if (ret < 0)
				return ret;

			buf[0] = (XDATA_Addr) & 0xFF;
			buf[1] = 0x00;
			buf[2] = 0x00;
			ret = CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 3);
			if (ret < 0)
				return ret;

			RD_Filechksum[i] = (uint16_t)((buf[2] << 8) | buf[1]);
			if (WR_Filechksum[i] != RD_Filechksum[i])
				return 0;
		}
	}

	return 1;
}
#endif

int32_t Init_BootLoader(void)
{
	uint8_t buf[64] = {0};
	int32_t ret = 0;
	int32_t retry = 0;

	nvt_sw_reset_idle();

	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = I2C_FW_Address;
	ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 3);
	if (ret < 0)
		return ret;

	retry = 0;
	for (;;) {
		msleep(1);
		buf[0] = 0x00;
		buf[1] = 0x00;
		ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0)
			return ret;
		if (buf[1] == 0xAA)
			break;
		retry++;
		if (unlikely(retry > 20))
			return -EPERM;
	}

	msleep(20);
	return 0;
}
#if BOOT_UPDATE_FIRMWARE
int32_t Erase_Flash(void)
{
	uint8_t buf[64] = {0};
	int32_t ret = 0;
	int32_t count = 0;
	int32_t i = 0;
	int32_t Flash_Address = 0;
	int32_t retry = 0;

	buf[0] = 0x00;
	buf[1] = 0x06;
	ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
	if (ret < 0)
		return ret;

	retry = 0;
	for (;;) {
		mdelay(1);
		buf[0] = 0x00;
		buf[1] = 0x00;
		ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0)
			return ret;
		if (buf[1] == 0xAA)
			break;
		retry++;
		if (unlikely(retry > 20))
			return -EPERM;
	}

	buf[0] = 0x00;
	buf[1] = 0x01;
	buf[2] = 0x00;
	ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 3);
	if (ret < 0)
		return ret;

	retry = 0;
	for (;;) {
		mdelay(1);
		buf[0] = 0x00;
		buf[1] = 0x00;
		ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0)
			return ret;
		if (buf[1] == 0xAA)
			break;
		retry++;
		if (unlikely(retry > 20))
			return -EPERM;
	}

	retry = 0;
	for (;;) {
		mdelay(5);
		buf[0] = 0x00;
		buf[1] = 0x05;
		ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0)
			return ret;

		buf[0] = 0x00;
		buf[1] = 0x00;
		buf[2] = 0x00;
		ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 3);
		if (ret < 0)
			return ret;
		if ((buf[1] == 0xAA) && (buf[2] == 0x00))
			break;
		retry++;
		if (unlikely(retry > 100))
			return -EPERM;
	}

	if (fw_entry->size % FLASH_SECTOR_SIZE)
		count = fw_entry->size / FLASH_SECTOR_SIZE + 1;
	else
		count = fw_entry->size / FLASH_SECTOR_SIZE;

	for (i = 0; i < count; i++) {
		buf[0] = 0x00;
		buf[1] = 0x06;
		ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0)
			return ret;

		retry = 0;
		for (;;) {
			mdelay(1);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0)
				return ret;
			if (buf[1] == 0xAA)
				break;
			retry++;
			if (unlikely(retry > 20))
				return -EPERM;
		}

		Flash_Address = i * FLASH_SECTOR_SIZE;
		buf[0] = 0x00;
		buf[1] = 0x20;
		buf[2] = ((Flash_Address >> 16) & 0xFF);
		buf[3] = ((Flash_Address >> 8) & 0xFF);
		buf[4] = (Flash_Address & 0xFF);
		ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 5);
		if (ret < 0)
			return ret;

		retry = 0;
		for (;;) {
			mdelay(1);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0)
				return ret;
			if (buf[1] == 0xAA)
				break;
			retry++;
			if (unlikely(retry > 20))
				return -EPERM;
		}

		retry = 0;
		for (;;) {
			mdelay(5);
			buf[0] = 0x00;
			buf[1] = 0x05;
			ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0)
				return ret;

			buf[0] = 0x00;
			buf[1] = 0x00;
			buf[2] = 0x00;
			ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 3);
			if (ret < 0)
				return ret;
			if ((buf[1] == 0xAA) && (buf[2] == 0x00))
				break;
			retry++;
			if (unlikely(retry > 100))
				return -EPERM;
		}
	}

	return 0;
}

int32_t Write_Flash(void)
{
	uint8_t buf[64] = {0};
	uint32_t XDATA_Addr = ts->mmap->RW_FLASH_DATA_ADDR;
	uint32_t Flash_Address = 0;
	int32_t i = 0, j = 0, k = 0;
	uint8_t tmpvalue = 0;
	int32_t count = 0;
	int32_t ret = 0;
	int32_t retry = 0;

	buf[0] = 0xFF;
	buf[1] = XDATA_Addr >> 16;
	buf[2] = (XDATA_Addr >> 8) & 0xFF;
	ret = CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);
	if (ret < 0)
		return ret;

	if (fw_entry->size % 256)
		count = fw_entry->size / 256 + 1;
	else
		count = fw_entry->size / 256;

	for (i = 0; i < count; i++) {
		Flash_Address = i * 256;
		buf[0] = 0x00;
		buf[1] = 0x06;
		ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
		if (ret < 0)
			return ret;

		retry = 0;
		for (;;) {
			udelay(100);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0)
				return ret;
			if (buf[1] == 0xAA)
				break;
			retry++;
			if (unlikely(retry > 20))
				return -EPERM;
		}

		for (j = 0; j < min(fw_entry->size - i * 256, (size_t)256); j += 32) {
			buf[0] = (XDATA_Addr + j) & 0xFF;
			for (k = 0; k < 32; k++)
				buf[1 + k] = fw_entry->data[Flash_Address + j + k];
			ret = CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 33);
			if (ret < 0)
				return ret;
		}
		if (fw_entry->size - Flash_Address >= 256)
			tmpvalue = (Flash_Address >> 16) + ((Flash_Address >> 8) & 0xFF) + (Flash_Address & 0xFF) + 0x00 + (255);
		else
			tmpvalue = (Flash_Address >> 16) + ((Flash_Address >> 8) & 0xFF) + (Flash_Address & 0xFF) + 0x00 + (fw_entry->size - Flash_Address - 1);

		for (k = 0; k < min(fw_entry->size - Flash_Address, (size_t)256); k++)
			tmpvalue += fw_entry->data[Flash_Address + k];

		tmpvalue = 255 - tmpvalue + 1;
		buf[0] = 0x00;
		buf[1] = 0x02;
		buf[2] = ((Flash_Address >> 16) & 0xFF);
		buf[3] = ((Flash_Address >> 8) & 0xFF);
		buf[4] = (Flash_Address & 0xFF);
		buf[5] = 0x00;
		buf[6] = min(fw_entry->size - Flash_Address, (size_t)256) - 1;
		buf[7] = tmpvalue;
		ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 8);
		if (ret < 0)
			return ret;

		retry = 0;
		for (;;) {
			mdelay(1);
			buf[0] = 0x00;
			buf[1] = 0x00;
			ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0)
				return ret;
			if (buf[1] == 0xAA || buf[1] == 0xEA)
				break;
			retry++;
			if (unlikely(retry > 20))
				return -EPERM;
		}
		if (buf[1] == 0xEA)
			return -3;

		retry = 0;
		for (;;) {
			mdelay(5);
			buf[0] = 0x00;
			buf[1] = 0x05;
			ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
			if (ret < 0)
				return ret;

			buf[0] = 0x00;
			buf[1] = 0x00;
			buf[2] = 0x00;
			ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 3);
			if (ret < 0)
				return ret;
			if (((buf[1] == 0xAA) && (buf[2] == 0x00)) || (buf[1] == 0xEA))
				break;
			retry++;
			if (unlikely(retry > 100))
				return -EPERM;
		}

		if (buf[1] == 0xEA)
			return -4;

	}

	return 0;
}

int32_t Verify_Flash(void)
{
	uint8_t buf[64] = {0};
	uint32_t XDATA_Addr = ts->mmap->READ_FLASH_CHECKSUM_ADDR;
	int32_t ret = 0;
	int32_t i = 0;
	int32_t k = 0;
	uint16_t WR_Filechksum[BLOCK_64KB_NUM] = {0};
	uint16_t RD_Filechksum[BLOCK_64KB_NUM] = {0};
	size_t fw_bin_size = 0;
	size_t len_in_blk = 0;
	int32_t retry = 0;

	fw_bin_size = fw_entry->size;

	for (i = 0; i < BLOCK_64KB_NUM; i++) {
		if (fw_bin_size > (i * SIZE_64KB)) {

			len_in_blk = min(fw_bin_size - i * SIZE_64KB, (size_t)SIZE_64KB);
			WR_Filechksum[i] = i + 0x00 + 0x00 + (((len_in_blk - 1) >> 8) & 0xFF) + ((len_in_blk - 1) & 0xFF);
			for (k = 0; k < len_in_blk; k++)
				WR_Filechksum[i] += fw_entry->data[k + i * SIZE_64KB];
			WR_Filechksum[i] = 65535 - WR_Filechksum[i] + 1;

			buf[0] = 0x00;
			buf[1] = 0x07;
			buf[2] = i;
			buf[3] = 0x00;
			buf[4] = 0x00;
			buf[5] = ((len_in_blk - 1) >> 8) & 0xFF;
			buf[6] = (len_in_blk - 1) & 0xFF;
			ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 7);
			if (ret < 0)
				return ret;

			retry = 0;
			for (;;) {
				msleep(80);
				buf[0] = 0x00;
				buf[1] = 0x00;
				ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 2);
				if (ret < 0)
					return ret;
				if (buf[1] == 0xAA)
					break;
				retry++;
				if (unlikely(retry > 5))
					return -EPERM;
			}

			buf[0] = 0xFF;
			buf[1] = XDATA_Addr >> 16;
			buf[2] = (XDATA_Addr >> 8) & 0xFF;
			ret = CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);
			if (ret < 0)
				return ret;

			buf[0] = (XDATA_Addr) & 0xFF;
			buf[1] = 0x00;
			buf[2] = 0x00;
			ret = CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 3);
			if (ret < 0)
				return ret;

			RD_Filechksum[i] = (uint16_t)((buf[2] << 8) | buf[1]);
			if (WR_Filechksum[i] != RD_Filechksum[i])
				return -EPERM;
		}
	}

	return 0;
}

int32_t Update_Firmware(void)
{
	int32_t ret = 0;

	ret = Init_BootLoader();
	if (ret)
		return ret;

	ret = Resume_PD();
	if (ret)
		return ret;

	ret = Erase_Flash();
	if (ret)
		return ret;

	ret = Write_Flash();
	if (ret)
		return ret;

	ret = Verify_Flash();
	if (ret)
		return ret;

	nvt_bootloader_reset();
	nvt_check_fw_reset_state(RESET_STATE_INIT);

	nvt_get_fw_info();
	return ret;
}

#define NVT_FLASH_END_FLAG_LEN 3
#define NVT_FLASH_END_FLAG_ADDR 0x1AFFD
int32_t nvt_check_flash_end_flag(void)
{
	uint8_t buf[8] = {0};
	uint8_t nvt_end_flag[NVT_FLASH_END_FLAG_LEN + 1] = {0};
	int32_t ret = 0;

	ret = Init_BootLoader();
	if (ret)
		return ret;

	ret = Resume_PD();
	if (ret)
		return ret;

	buf[0] = 0x00;
	buf[1] = 0x35;
	ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 2);
	if (ret < 0)
		return ret;
	msleep(10);

	buf[0] = 0x00;
	buf[1] = 0x03;
	buf[2] = (NVT_FLASH_END_FLAG_ADDR >> 16) & 0xFF;
	buf[3] = (NVT_FLASH_END_FLAG_ADDR >> 8) & 0xFF;
	buf[4] = NVT_FLASH_END_FLAG_ADDR & 0xFF;
	buf[5] = (NVT_FLASH_END_FLAG_LEN >> 8) & 0xFF;
	buf[6] = NVT_FLASH_END_FLAG_LEN & 0xFF;
	ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, buf, 7);
	if (ret < 0)
		return ret;
	msleep(10);

	buf[0] = 0x00;
	buf[1] = 0x00;
	ret = CTP_I2C_READ(ts->client, I2C_HW_Address, buf, 2);
	if (ret < 0)
		return ret;
	if (buf[1] != 0xAA)
		return -EPERM;

	msleep(10);

	buf[0] = 0xFF;
	buf[1] = (ts->mmap->READ_FLASH_CHECKSUM_ADDR >> 16) & 0xFF;
	buf[2] = (ts->mmap->READ_FLASH_CHECKSUM_ADDR >> 8) & 0xFF;
	ret = CTP_I2C_WRITE(ts->client, I2C_BLDR_Address, buf, 3);
	if (ret < 0)
		return ret;
	msleep(10);

	buf[0] = ts->mmap->READ_FLASH_CHECKSUM_ADDR & 0xFF;
	ret = CTP_I2C_READ(ts->client, I2C_BLDR_Address, buf, 6);
	if (ret < 0)
		return ret;

	strncpy(nvt_end_flag, &buf[3], NVT_FLASH_END_FLAG_LEN);

	if (strncmp(nvt_end_flag, "NVT", 3) == 0)
		return 0;
	else
		return 1;
}

void Boot_Update_Firmware(struct work_struct *work)
{
	int32_t ret = 0;

	char firmware_name[256] = "";
	if (tianma_jdi_flag)
		sprintf(firmware_name, BOOT_UPDATE_FIRMWARE_NAME_JDI);
	else
		sprintf(firmware_name, BOOT_UPDATE_FIRMWARE_NAME_TIANMA);

	ret = update_firmware_request(firmware_name);
	if (ret)
		return;

	mutex_lock(&ts->lock);
	nvt_sw_reset_idle();
	ret = Check_CheckSum();

	if (ret < 0)
		Update_Firmware();
	else if ((ret == 0) && (Check_FW_Ver() == 0))
		Update_Firmware();
	else if (nvt_check_flash_end_flag())
		Update_Firmware();
	else {
		nvt_bootloader_reset();
		ret = nvt_check_fw_reset_state(RESET_STATE_INIT);
		if (ret)
			Update_Firmware();
	}

	mutex_unlock(&ts->lock);
	update_firmware_release();
}
#endif /* BOOT_UPDATE_FIRMWARE */
