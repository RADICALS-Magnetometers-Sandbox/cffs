/*
 * command_handler.c
 *
 *  Created on: May 9, 2022
 *      Author: Liam
 */
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "command_handler.h"
#include "debug.h"
#include "arducam.h"
#include "IEB_TESTS.h"
#include "iris_time.h"
#include "time.h"
#include "nandfs.h"
#include "nand_types.h"
#include "iris_system.h"
#include "nand_errno.h"
#include "logger.h"

extern uint8_t VIS_DETECTED;
extern uint8_t NIR_DETECTED;

extern SPI_HandleTypeDef hspi1;
extern RTC_HandleTypeDef hrtc;

extern int format;
extern int width;

extern const struct sensor_reg OV5642_JPEG_Capture_QSXGA[];
extern const struct sensor_reg OV5642_QVGA_Preview[];

extern uint8_t turn_off_logger_flag;
extern uint8_t direct_method_flag;

#ifdef UART_HANDLER
// Only used for UART operations
uint8_t VIS_DETECTED = 0;
uint8_t NIR_DETECTED = 0;
#endif

uint8_t sensor_status = SENSORS_OFF;

housekeeping_packet_t hk;
char buf[128];

FileInfo_t image_file_infos_queue[MAX_IMAGE_FILES] = {0};
uint8_t image_count = 0;

/**
 * @brief Counts number of images stored in the flash file system and transmits it over SPI
 *
 * @param image_count: Pointer to variable containing number of images in NAND fs
 * Untested
 */
void get_image_count(uint8_t *cnt) { *(cnt) = image_count; }

/**
 * @brief Get the image length
 *
 * @param image_length: Pointer to variable containing length of image
 *
 * Currently image length is directly obtained from Arducam registers, although
 * it is desired to extract image lengths from NAND fs
 */
int get_image_length(uint32_t *image_length, uint8_t index) {
    int ret;
    NAND_FILE *file;
    file = NANDfs_open(image_file_infos_queue[index].file_id);

    if (!file) {
        iris_log("open file %d failed: %d\r\n", file, nand_errno);
        return -1;
    }
    *(image_length) = file->node.file_size;

    ret = NANDfs_close(file);
    if (ret < 0) {
        iris_log("not able to close file %d failed: %d\r\n", file, nand_errno);
        return -1;
    }

    return 0;
}


/**
 * @brief Once sensors are on, initialize configurations (i.e. resolution,
 * saturation, etc) for both of the sensors
 *
 * Future developers can add or modify different configuration
 */
void set_configurations(Iris_config *config) {
    // These flags toggle different software methods
    turn_off_logger_flag = config->toggle_iris_logger;
    direct_method_flag = config->toggle_direct_method;

    // Format NAND flash
    uint8_t format_nand_flash = config->format_iris_nand;
    if (format_nand_flash == 1) {
        NANDfs_format();
    }
}

int transfer_image_to_nand(uint8_t sensor, uint8_t *file_timestamp) {
    int ret = 0;
    HAL_Delay(100);

    uint32_t image_size;
    image_size = read_fifo_length(sensor);

    NAND_FILE *file = NANDfs_create();
    if (!file) {
        iris_log("not able to create file %d failed: %d", file, nand_errno);
        return -1;
    }

    int size_remaining;
    uint8_t image[PAGE_DATA_SIZE];
    uint32_t i = 0;
    int chunks_to_write = ((image_size + (PAGE_DATA_SIZE - 1)) / PAGE_DATA_SIZE);

    spi_init_burst(sensor);
    for (int j = 0; j < chunks_to_write; j++) {
        for (i = 0; i < PAGE_DATA_SIZE; i++) {
            image[i] = spi_read_burst(sensor);
        }
        size_remaining = i;
        while (size_remaining > 0) {
            int size_to_write = size_remaining > PAGE_DATA_SIZE ? PAGE_DATA_SIZE : size_remaining;
            ret = NANDfs_write(file, size_to_write, image);
            if (ret < 0) {
                iris_log("not able to write to file %d failed: %d", file, nand_errno);
                return -1;
            }
            size_remaining -= size_to_write;
        }
    }
    spi_deinit_burst(sensor);
    file->node.file_name = file_timestamp;

    image_file_infos_queue[image_count].file_id = file->node.id;
    image_file_infos_queue[image_count].file_name = file->node.file_name;
    image_file_infos_queue[image_count].file_size = file->node.file_size;

    ret = NANDfs_close(file);
    if (ret < 0) {
        iris_log("not able to close file %d failed: %d", file, nand_errno);
        return -1;
    }

    iris_log("%d|%s|%d", image_file_infos_queue[image_count].file_id,
             image_file_infos_queue[image_count].file_name, image_file_infos_queue[image_count].file_size);

    image_count += 1;
    return 0;
}

int delete_image_file_from_queue(uint16_t index) {
    int ret;

    ret = NANDfs_delete(image_file_infos_queue[index].file_id);
    if (ret < 0) {
        iris_log("not able to delete file %d failed: %d\r\n", image_file_infos_queue[index].file_id, nand_errno);
        return -1;
    }
    image_file_infos_queue[index].file_id = -1;
    return 0;
}

NAND_FILE *get_image_file_from_queue(uint8_t index) {
    NAND_FILE *file = NANDfs_open(image_file_infos_queue[index].file_id);
    if (!file) {
        iris_log("not able to open file %d failed: %d\r\n", file, nand_errno);
    }
    return file;
}


/*
 * @brief Store information from existing files from NAND to
 * 		  RAM (buffer)
 *
 * 		  The purpose of transferring file info (file_id, file_name,
 * 		  file_size) from NAND to RAM during on-boot is to set up a
 * 		  quick lookup table for image transfer APIs to quickly
 * 		  extract file information, instead of accessing NAND structures
 * 		  directly.
 */
int store_file_infos_in_buffer() {
    uint8_t index;
    DIRENT cur_node;
    NAND_DIR *cur_dir;
    int ret = 0;

    index = 0;
    cur_dir = NANDfs_opendir();

    do {
        cur_node = *(NANDfs_getdir(cur_dir));

        ret = NANDfs_nextdir(cur_dir);
        if (ret < 0) {
            if (nand_errno == NAND_EBADF) {
                iris_log("Reached end of last inode. Total image files: %d\r\n", image_count);
                return 0;
            } else {
                iris_log("Moving to next directory entry failed: %d\r\n", nand_errno);
                return -1;
            }
        }

        image_file_infos_queue[index].file_id = cur_node.id;
        image_file_infos_queue[index].file_name = cur_node.file_name;
        image_file_infos_queue[index].file_size = cur_node.file_size;

        image_count++;
        index += 1;
    } while (ret != 0);

    return 0;
}

/******************************************************************************
 *                      UART Operations (Not used in flight)
 *****************************************************************************/

static inline const char *next_token(const char *ptr) {
    /* move to the next space */
    while (*ptr && *ptr != ' ')
        ptr++;
    /* move past any whitespace */
    while (*ptr && isspace(*ptr))
        ptr++;

    return (*ptr) ? ptr : NULL;
}

/**
 * @brief UART command handler
 *
 * @param pointer to char command from main
 */
void uart_handle_command(char *cmd) {
    uint8_t in[sizeof(housekeeping_packet_t)];
    switch (*cmd) {
    case 'h':
        help();
        break;
    case 'n':
        uart_handle_nand_commands(cmd);
        break;

    default:
        help();
        break;
    }
}
