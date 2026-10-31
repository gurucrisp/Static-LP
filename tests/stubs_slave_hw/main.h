/* A model of the HAL flash API the slave's flash driver uses, with an in-memory flash, so that the driver's address checks, doubleword programming and bank/page arithmetic can be tested on a PC. It models the API, not the silicon. */
#ifndef MAIN_H_STUB_SLAVE
#define MAIN_H_STUB_SLAVE
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { uint32_t TypeErase, Banks, Page, NbPages; } FLASH_EraseInitTypeDef;
typedef struct { volatile uint32_t OPTR; } FLASH_TypeDef;
extern FLASH_TypeDef flash_m;
#define FLASH (&flash_m)
#define FLASH_OPTR_DBANK  (1UL << 22)
#define FLASH_TYPEERASE_PAGES 0x01U
#define FLASH_TYPEPROGRAM_DOUBLEWORD 0x01U
#define FLASH_BANK_1 1U
#define FLASH_BANK_2 2U
HAL_StatusTypeDef HAL_FLASH_Unlock(void); HAL_StatusTypeDef HAL_FLASH_Lock(void); uint32_t HAL_FLASH_GetError(void);
/* v0.9.79: the user option byte and the reset flags (the BOR check) */
#define FLASH_OPTR_BOR_EN   (0x1UL << 8U)
#define FLASH_OPTR_BORF_LEV (0x3UL << 9U)
#define FLASH_OPTR_BORR_LEV (0x3UL << 11U)
typedef struct { volatile uint32_t CSR; } RCC_TypeDef;
extern RCC_TypeDef rcc_m;
#define RCC (&rcc_m)
#define RCC_CSR_OBLRSTF 0x02000000U
typedef struct { uint32_t OptionType, WRPArea, WRPStartOffset, WRPEndOffset, RDPLevel, USERType, USERConfig; } FLASH_OBProgramInitTypeDef;
#define OPTIONBYTE_USER 0x00000004U
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void); HAL_StatusTypeDef HAL_FLASH_OB_Lock(void); HAL_StatusTypeDef HAL_FLASH_OB_Launch(void);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *ob);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t addr, uint64_t data);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *err);
#endif
