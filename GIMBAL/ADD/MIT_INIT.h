#ifndef MIT_INIT_H
#define MIT_INIT_H

#include "main.h"

#include "bsp_flash.h"
#include "string.h"


#define FLASH_USER_MIT_ADDR         ADDR_FLASH_SECTOR_10 //write flash page 10,保存的flash页地址
#define MIT_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len))     //flash write function,flash 写入函数
#define MIT_flash_read(address, buf, len)  flash_read((address), (buf), (len))                     //flash read function, flash 读取函数
#define MIT_flash_erase(address, page_num) flash_erase_address((address), (page_num))              //flash erase function,flash擦除函数
#define MIT_WRITE_BUF_LENGHT  16

typedef struct
{
    float MIT_INIT_ANGLE[4]; //ABCD
} MIT_cali_t;





extern float mit_init_angle_form_flash[4];
extern int while_write_OK;


void MIT_init_angle_read_from_flash(void);
void MIT_init_angle_WRITE_to_flash(void);

#endif
