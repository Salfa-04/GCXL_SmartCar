#include "fsetting.h"

#include <type_def.h>

#define FLASH_ADDR 0x080E0000U        // Sector 11
#define FLASH_SIZE 0x00020000U        // 128k bytes
#define BLOCK_SIZE 0x80U              // 128  bytes
#define FLASH_SECTOR FLASH_SECTOR_11  // Start at 896k bytes

/// n: block number; 0 ~ 1023
/// data: 0 ~ 126 bytes; -1: ?used
#define BLOCK_ADDR(n) (FLASH_ADDR + (n) * BLOCK_SIZE + 1U)
#define BLOCK_NUMS (FLASH_SIZE / BLOCK_SIZE)  // 1024 blocks

uint8_t fSettings[BLOCK_SIZE] = {0};
static uint16_t fOffset = 0;
static uint8_t *ptr = NULL;

void fset_seek(void);
void fset_erase_all(void);

void fset_save(uint8_t *data, int8_t len) {
  if (data == NULL || len < 1) return;

  fset_seek();
  HAL_FLASH_Unlock();

  while (len--)
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)ptr + len, data[len]);
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)ptr - 1, 0x00);

  HAL_FLASH_Lock();
}

void fset_read(uint8_t **ptr) {
  /* Get Setting Buffer */
  *ptr = (void *)BLOCK_ADDR(fOffset);
}

void fset_seek(void) {
  do {
    fset_read(&ptr);
    if (ptr[-1] == 0xFF) return;
  } while (++fOffset < BLOCK_NUMS);

  // if all blocks used
  fset_erase_all();
  fOffset = 0;
  fset_read(&ptr);
}

void fset_erase_all(void) {
  uint32_t error;

  FLASH_EraseInitTypeDef erase;
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR;
  erase.NbSectors = 1;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&erase, &error);
  HAL_FLASH_Lock();
}
