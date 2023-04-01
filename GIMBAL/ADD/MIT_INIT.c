#include "MIT.h"
#include "MIT_INIT.h"

/**
 * @brief          从flash读取校准数据
 * @param[in]      none
 * @retval         none
 */
static void MIT_data_read(void);

float mit_init_angle_form_flash[4];
uint16_t offset = 0;
int while_write_OK = 0;

static uint8_t MIT_flash_write_buf[MIT_WRITE_BUF_LENGHT];

void MIT_init_angle_read_from_flash()
{

  MIT_flash_read(FLASH_USER_MIT_ADDR, (uint32_t *)mit_init_angle_form_flash, 4); // 一次读取4个4字节,4个float

  MIT_A.MIT_TZG = mit_init_angle_form_flash[0];
  MIT_B.MIT_TZG = mit_init_angle_form_flash[1];
  MIT_C.MIT_TZG = mit_init_angle_form_flash[2];
  MIT_D.MIT_TZG = mit_init_angle_form_flash[3];

  // MIT_A.MIT_TZG_ARRIVE=mit_init_angle_form_flash[0];
  // MIT_B.MIT_TZG_ARRIVE=mit_init_angle_form_flash[1];
  // MIT_C.MIT_TZG_ARRIVE=mit_init_angle_form_flash[2];
  // MIT_D.MIT_TZG_ARRIVE=mit_init_angle_form_flash[3];

  MIT_A.MIT_TSZ = MIT_A.MIT_TZG + 99;
  MIT_B.MIT_TSZ = MIT_B.MIT_TZG - 99;
  MIT_C.MIT_TSZ = MIT_C.MIT_TZG + 99;
  MIT_D.MIT_TSZ = MIT_D.MIT_TZG - 99;
}

void MIT_init_angle_WRITE_to_flash()
{
  float flash_write_text_buf[4] = {1.1, 2.2, 3.3, 4.4};

  flash_write_text_buf[0] = MIT_A.ANGLE_JD;
  flash_write_text_buf[1] = MIT_B.ANGLE_JD;
  flash_write_text_buf[2] = MIT_C.ANGLE_JD;
  flash_write_text_buf[3] = MIT_D.ANGLE_JD;

  // erase the page
  // 擦除flash页面 1页 1个扇区
  MIT_flash_erase(FLASH_USER_MIT_ADDR, 1);
  memcpy((void *)MIT_flash_write_buf, (void *)flash_write_text_buf, 16);

  while_write_OK = MIT_flash_write(FLASH_USER_MIT_ADDR, (uint32_t *)flash_write_text_buf, 4);
}
