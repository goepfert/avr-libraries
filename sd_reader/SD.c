/*****************************************************************************
* File          : SD.c
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

#include "SD.h"

#include <string.h>

#include "sd_raw.h"
#include "sd_raw_config.h"
#include "fat.h"

#include "../uart/uart.h"

/********************************************************************************
 * opens partition
 * - check sd_raw_config.h to enable/disable write support
 *********************************************************************************/
struct partition_struct* openPartition() {

	// open first partition
	struct partition_struct* part = partition_open(sd_raw_read, sd_raw_read_interval,
#if SD_RAW_WRITE_SUPPORT
			sd_raw_write, sd_raw_write_interval,
#else
			0, 0,
#endif
			0);

	if(!part) {

		// If the partition did not open, assume the storage device
		// is a "superfloppy", i.e. has no MBR.
		part = partition_open(sd_raw_read, sd_raw_read_interval,
#if SD_RAW_WRITE_SUPPORT
				sd_raw_write, sd_raw_write_interval,
#else
				0, 0,
#endif
				-1);
	}

	return part;

} // openPartition()


/********************************************************************************
 * Find File/Subdirectory in Directory
 * dir_entry: pointer to a buffer into which to write the directory entry
 * 			  information
 * returns 1 if found, 0 if not
 *********************************************************************************/
uint8_t find_file_in_dir(struct fat_dir_struct* fatdir, const char* name, struct fat_dir_entry_struct* dir_entry) {

    while(fat_read_dir(fatdir, dir_entry)) {

        if(strcmp(dir_entry->long_name, name) == 0) {
            fat_reset_dir(fatdir); // resets directory handle
            return 1;
        }
    }

    return 0;
} // find_file_in_dir()


/********************************************************************************
 *
 *********************************************************************************/
struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* fatdir, const char* name) {
    struct fat_dir_entry_struct file_entry;
    if(!find_file_in_dir(fatdir, name, &file_entry))
        return 0;

    return fat_open_file(fs, &file_entry);
}


/********************************************************************************
 * init SD card
 * - spi config, reset sd,
 * - get partition, filesystem and root directory
 * - return 0 if fail, 1 for success
 *********************************************************************************/
uint8_t sd_init() {

    if(!sd_raw_init()) {
        return 0;
    }

    // open partition
    partition = openPartition();
    if(!partition) {
        return 0;
    }

    // open file system
    filesystem = fat_open(partition);
    if(!filesystem) {
        return 0;
    }

    // open root directory
    struct fat_dir_entry_struct directory;
    fat_get_dir_entry_of_path(filesystem, "/", &directory);

    fatdir = fat_open_dir(filesystem, &directory);
    if(!fatdir) {
        return 0;
    }

    return 1;

}// sd_init()


/********************************************************************************
 * closes all relevant handles
 *********************************************************************************/
void sd_close(void){

	if(!sd_sync() || !fatdir || !filesystem || !partition){
		return;
	}

    // close directory
    fat_close_dir(fatdir);

    // close file system
    fat_close(filesystem);

    // close partition
    partition_close(partition);

} // sd_close()


/********************************************************************************
 * When write buffering is enabled, you should call this function before
 * disconnecting the card to ensure all remaining data has been written.
 *********************************************************************************/
uint8_t sd_sync(){

#if SD_RAW_WRITE_BUFFERING
	if(!sd_raw_sync()){
		return 0;
	}
#endif

	return 1;
} // sd_sync()


/********************************************************************************
 * print sd card information using uart
 ********************************************************************************/
/*
uint8_t sd_print_disk_info() {

	if(!filesystem){
    	return 0;
    }

    struct sd_raw_info disk_info;
    if(!sd_raw_get_info(&disk_info)){
        return 0;
    }

    uart_puts_P("manuf:  0x"); uart_putc_hex(disk_info.manufacturer); uart_putc('\n');
    uart_puts_P("oem:    ");   uart_puts((char*) disk_info.oem); uart_putc('\n');
    uart_puts_P("prod:   ");   uart_puts((char*) disk_info.product); uart_putc('\n');
    uart_puts_P("rev:    ");   uart_putc_hex(disk_info.revision); uart_putc('\n');
    uart_puts_P("serial: 0x"); uart_putdw_hex(disk_info.serial); uart_putc('\n');
    uart_puts_P("date:   ");   uart_putw_dec(disk_info.manufacturing_month); uart_putc('/');
                               uart_putw_dec(disk_info.manufacturing_year); uart_putc('\n');
    uart_puts_P("size:   ");   uart_putdw_dec(disk_info.capacity / 1024 / 1024); uart_puts_P("MB\n");
    uart_puts_P("copy:   ");   uart_putw_dec(disk_info.flag_copy); uart_putc('\n');
    uart_puts_P("wr.pr.: ");   uart_putw_dec(disk_info.flag_write_protect_temp); uart_putc('/');
                               uart_putw_dec(disk_info.flag_write_protect); uart_putc('\n');
    uart_puts_P("format: ");   uart_putw_dec(disk_info.format); uart_putc('\n');
    uart_puts_P("free:   ");   uart_putdw_dec(fat_get_fs_free(filesystem)); uart_putc('/');
                               uart_putdw_dec(fat_get_fs_size(filesystem)); uart_putc('\n');

    return 1;
} // print_disk_info()
*/

/********************************************************************************
 * print directory listing
 * ls
 ********************************************************************************/
uint8_t sd_print_dir() {

	if(!fatdir) {
		return 0;
	}

	struct fat_dir_entry_struct dir_entry;

	while(fat_read_dir(fatdir, &dir_entry)) {
		uint8_t spaces = sizeof(dir_entry.long_name) - strlen(dir_entry.long_name) + 4;
		uart_puts(dir_entry.long_name);
		uart_putc(dir_entry.attributes & FAT_ATTRIB_DIR ? '/' : ' ');
		while(spaces--) {
			uart_putc(' ');
		}
		uart_putdw_dec(dir_entry.file_size);
		uart_putc('\n');
	}

	return 1;
}// sd_print_dir()


/********************************************************************************
 * cat file
 ********************************************************************************/
uint8_t sd_print_file(char* filename) {

	if(!sd_open_file(filename, 0)){
		return 0;
	}

	//save old pos
	int32_t file_pos = 0;
	fat_seek_file(fatfile, &file_pos, FAT_SEEK_CUR);

	//rewind
	int32_t file_pos_new = 0;
	fat_seek_file(fatfile, &file_pos_new, FAT_SEEK_SET);

    // print file contents
    uint8_t buffer[8];
    uint32_t offset = 0;
    intptr_t count;
    while((count = fat_read_file(fatfile, buffer, sizeof(buffer))) > 0)
    {
        uart_putdw_hex(offset);
        uart_putc(':');
        for(intptr_t i = 0; i < count; ++i)
        {
            uart_putc(' ');
            uart_putc_hex(buffer[i]);
        }
        uart_putc('\n');
        offset += 8;
    }

    //back to old pos
    fat_seek_file(fatfile, &file_pos, FAT_SEEK_SET);

    return 1;
} // sd_print_file()


/********************************************************************************
 * change into a (sub)directory directory
 * 'cd ..' for one level up
 ********************************************************************************/
uint8_t sd_change_dir(char* dirname){

	struct fat_dir_entry_struct subdir_entry;
	if(find_file_in_dir(fatdir, dirname, &subdir_entry)) {

		struct fat_dir_struct* fatdir_new = fat_open_dir(filesystem, &subdir_entry);
		if(fatdir_new){

			fat_close_dir(fatdir);
			fatdir = fatdir_new;
			return 1;
		}
	}

	return 0;
} // sd_change_dir()


/********************************************************************************
 * returns 0 on failure, 1 on success, 2 if the directory already existed
 ********************************************************************************/
uint8_t sd_create_dir(char* dirname){

    struct fat_dir_entry_struct dir_entry;
    return fat_create_dir(fatdir, dirname, &dir_entry);
} // sd_create_dir()


/********************************************************************************
 * returns 0 on failure, 1 on success, 2 if the file already existed
 ********************************************************************************/
uint8_t sd_create_file(char* filename){

    struct fat_dir_entry_struct file_entry;
    return fat_create_file(fatdir, filename, &file_entry);
} // sd_create_file()


/********************************************************************************
 * removes file or directory
 * does NOT check if directory is empty
 * returns 0 on failure, 1 on success, 2 if file not found
 ********************************************************************************/
uint8_t sd_remove_file(char* filename){

    struct fat_dir_entry_struct file_entry;
    if(find_file_in_dir(fatdir, filename, &file_entry))
    {
        return fat_delete_file(filesystem, &file_entry);
    }

    return 2;
}


/********************************************************************************
 * open file
 ********************************************************************************/
uint8_t sd_open_file(char* filename, uint8_t opt) {

	if(fatfile) {
		//if(strcmp(fatfile->dir_entry.long_name, filename) == 0)
		sd_close_file();
	}

	//search file
	struct fat_dir_entry_struct file_entry;
	if(find_file_in_dir(fatdir, filename, &file_entry)) {
		// file found
		fatfile = fat_open_file(filesystem, &file_entry);
		if(opt & FILE_RECREATE) {
			// truncate
			fat_resize_file(fatfile, 0);
			int32_t file_pos = 0;
			fat_seek_file(fatfile, &file_pos, FAT_SEEK_SET);
		}
	} else {
		// file doesn't exist
		if(opt & (FILE_CREATE | FILE_RECREATE)){
			// create and open file
		    struct fat_dir_entry_struct file_entry;
		    fat_create_file(fatdir, filename, &file_entry);
		    fatfile = fat_open_file(filesystem, &file_entry);
		} else {
			return 0;
		}
	}

	return 1;
} // sd_open_file()


/********************************************************************************
 * close file (!only FAT_FILE_COUNT file can be opened at one time, default = 1)
 ********************************************************************************/
void sd_close_file(void){

	fat_close_file(fatfile);
	fatfile=0;
}


/********************************************************************************
 * write to a opened file at given position
 * returns the number of bytes written:
 * (0 or something less than buffer_len on disk full) or -1 on failure.
 ********************************************************************************/
int16_t sd_write_file_pos(int32_t offset, const uint8_t* buffer, uint16_t buffer_len) {

	if(!fatfile) {
		return 0;
	}

	if(!fat_seek_file(fatfile, &offset, FAT_SEEK_CUR)){
		return 0;
	}

    return fat_write_file(fatfile, buffer, buffer_len);
} // sd_write_file_pos()


/********************************************************************************
 * write to a opened file
 * returns the number of bytes written:
 * (0 or something less than buffer_len on disk full) or -1 on failure.
 ********************************************************************************/
int16_t sd_write_file(const uint8_t* buffer, uint16_t buffer_len) {

	if(!fatfile) {
		return 0;
	}

    return fat_write_file(fatfile, buffer, buffer_len);
} // sd_write_file()


/********************************************************************************
 * read from opened file
 ********************************************************************************/
int16_t sd_read_file(uint8_t* buffer, uint16_t buffer_len) {

    if(!fatfile) {
    	return -1;
    }

    return fat_read_file(fatfile, buffer, buffer_len);
} // sd_read_file()


/********************************************************************************
 * read from opened file
 ********************************************************************************/
int16_t sd_read_file_pos(int32_t offset, uint8_t* buffer, uint16_t buffer_len) {

    if(!fatfile) {
    	return -1;
    }

  	if(!fat_seek_file(fatfile, &offset, FAT_SEEK_SET)){
   		return -1;
   	}

    return fat_read_file(fatfile, buffer, buffer_len);
} // sd_read_file_pos()


uint32_t sd_get_file_size(){

	if(!fatfile){
		return 0;
	}

	return fatfile->dir_entry.file_size;
}
