/*****************************************************************************
* File          : SD.h
* Created       : 12.01.2013
*
* Title         : ---
* Author        : Thomas Goepfert
* Contact       : info@SolderLab.de
*
* Version       : 1.0
* Last Changed  : 12.01.2013 by goepfert
* Remarks       : ---
*
* Description   : wrapper for sd-reader
*
*****************************************************************************/

#ifndef SD_H_
#define SD_H_

#define DEBUG 1

#define FILE_CREATE 1
#define FILE_RECREATE 2

#include "partition.h"
#include "partition_config.h"
#include "fat.h"
#include "fat_config.h"

static struct partition_struct* partition;
static struct fat_fs_struct* filesystem;
static struct fat_dir_struct* fatdir;
static struct fat_file_struct* fatfile;

// helper functions
static struct partition_struct* openPartition();
static uint8_t find_file_in_dir(struct fat_dir_struct* fatdir, const char* name, struct fat_dir_entry_struct* dir_entry);
static struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* fatdir, const char* name);

// public functions
uint8_t sd_init(void); //TODO: (+ extern SPI)
void sd_close(void);
uint8_t sd_sync(void);
//uint8_t sd_print_disk_info();
uint8_t sd_print_dir();
uint8_t sd_print_file(char* filename);

uint8_t sd_change_dir(char* dirname);
uint8_t sd_create_dir(char* dirname);
uint8_t sd_create_file(char* filename);
uint8_t sd_remove_file(char* filename);

uint8_t sd_open_file(char* filename, uint8_t opt);
void sd_close_file(void);
int16_t sd_write_file(const uint8_t* buffer, uint16_t buffer_len);
int16_t sd_write_file_pos(int32_t offset, const uint8_t* buffer, uint16_t buffer_len);
int16_t sd_read_file(uint8_t* buffer, uint16_t buffer_len);
int16_t sd_read_file_pos(int32_t offset, uint8_t* buffer, uint16_t buffer_len);

uint32_t sd_get_file_size();

#endif /* SD_H_ */
