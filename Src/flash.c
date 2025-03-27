#include "flash.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include "usart.h"
#include "stdio.h"

#define DEBUG
#define FLASH_SIZE 64                   /* ËùÑ¡MCUµÄFLASHÈÝÁ¿´óÐ¡(µ¥Î»ÎªK) */

#if FLASH_SIZE < 256                    /* flashÐ¡ÓÚ256K×Ö½ÚµÄÐ¾Æ¬µÄÒ»¸öÉÈÇøµØÖ·Îª1K£¬·ñÔòÎª2K */
#define SECTOR_SIZE 1024                /* ×Ö½Ú */
#else
#define SECTOR_SIZE 2048                /* ×Ö½Ú */
#endif

_FLASH_flag flash_flag = {0};


/* Variable used for Erase procedure */
static FLASH_EraseInitTypeDef EraseInitStruct;
static uint32_t PageError;

/**
  * @brief   ÏòflashÐ´Èë°ë×ÖÊý¾Ý£¨16Î»£©
  * @param   startAddress Ð´Èë´¦µÄÆðÊ¼µØÖ·
  * @param   *writeData 16Î»Êý¾ÝÖ¸Õë±äÁ¿
  * @param   countToWrite Ð´ÈëµÄ°ë×ÖÊý¾ÝÊýÁ¿
  * @retval  x
  */
void FLASH_WriteHalfWordData( uint32_t startAddress, uint16_t *writeData, uint16_t countToWrite )
{
	uint16_t	i;
	uint32_t	offsetAddress;  /* Æ«ÒÆµØÖ· */
	uint32_t	sectorPosition; /* ÉÈÇøÎ»ÖÃ */
	uint32_t	sectorStartAddress;
	if ( startAddress < FLASH_BASE || ( (startAddress + countToWrite) >= (FLASH_BASE + 1024 * FLASH_SIZE) ) )
	{
		return;                 /* ·Ç·¨µØÖ· */
	}
	/* ½âËøÐ´±£»¤ */
	HAL_FLASH_Unlock();

	/* ¼ÆËãÈ¥µô0X08000000ºóµÄÊµ¼ÊÆ«ÒÆµØÖ· */
	offsetAddress = startAddress - FLASH_BASE;
	/* ¼ÆËãÉÈÇøµØÖ· */
	sectorPosition = offsetAddress / SECTOR_SIZE;
	/* ¶ÔÓ¦ÉÈÇøµÄÊ×µØÖ· */
	sectorStartAddress = sectorPosition * SECTOR_SIZE + FLASH_BASE;

	/* ²Á³ýÕâ¸öÉÈÇø */
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = sectorStartAddress;
  EraseInitStruct.NbPages = 1;
  printf("²Á³ýÒ³Êý£º%d\r\n",EraseInitStruct.NbPages);
  if ( HAL_FLASHEx_Erase( &EraseInitStruct, &PageError) == HAL_OK )      //Ð´ÈëÇ°ÏÈ½øÐÐÒ³²Á³ý
  {
    printf("²Á³ý³É¹¦\r\n");
  } 
	for ( i = 0; i < countToWrite; i++ )
	{
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,startAddress, writeData[i]);
		startAddress = startAddress + 2;
	}
  
	HAL_FLASH_Lock(); /*ÉÏËøÐ´±£»¤ */
}
/**
  * @brief   ÏòflashÐ´Èë×ÖÊý¾Ý£¨32Î»£©
  * @param   startAddress Ð´Èë´¦µÄÆðÊ¼µØÖ·
  * @param   *writeData 32Î»Êý¾ÝÖ¸Õë±äÁ¿
  * @param   countToWrite Ð´ÈëµÄ°ë×ÖÊý¾ÝÊýÁ¿
  * @retval  x
  */
void FLASH_WriteWordData( uint32_t startAddress, uint32_t *writeData, uint16_t countToWrite )
{
	uint16_t	i;
	uint32_t	offsetAddress;  /* Æ«ÒÆµØÖ· */
	uint32_t	sectorPosition; /* ÉÈÇøÎ»ÖÃ */
	uint32_t	sectorStartAddress;
	if ( startAddress < FLASH_BASE || ( (startAddress + countToWrite) >= (FLASH_BASE + 1024 * FLASH_SIZE) ) )
	{
		return;                 /* ·Ç·¨µØÖ· */
	}
	/* ½âËøÐ´±£»¤ */
	HAL_FLASH_Unlock();

	/* ¼ÆËãÈ¥µô0X08000000ºóµÄÊµ¼ÊÆ«ÒÆµØÖ· */
	offsetAddress = startAddress - FLASH_BASE;
	/* ¼ÆËãÉÈÇøµØÖ· */
	sectorPosition = offsetAddress / SECTOR_SIZE;
	/* ¶ÔÓ¦ÉÈÇøµÄÊ×µØÖ· */
	sectorStartAddress = sectorPosition * SECTOR_SIZE + FLASH_BASE;

	/* ²Á³ýÕâ¸öÉÈÇø */
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = sectorStartAddress;
  EraseInitStruct.NbPages = 1;
  printf("²Á³ýÒ³Êý£º%d\r\n",EraseInitStruct.NbPages);
  if ( HAL_FLASHEx_Erase( &EraseInitStruct, &PageError) == HAL_OK )      //Ð´ÈëÇ°ÏÈ½øÐÐÒ³²Á³ý
  {
    printf("²Á³ý³É¹¦\r\n");
  } 
	for ( i = 0; i < countToWrite; i++ )
	{
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,startAddress, writeData[i]);
		startAddress = startAddress + 4;
	}
  
	HAL_FLASH_Lock(); /*ÉÏËøÐ´±£»¤ */
}

/**
  * @brief   ÏòflashÐ´ÈëÈý¸ö¸¡µãÐÍÊý¾Ý
  * @param   startAddress Ð´Èë´¦µÄÆðÊ¼µØÖ·
  * @param   writeData1 ¸¡µãÊý¾Ý1
  * @param   writeData2 ¸¡µãÊý¾Ý2
  * @param   writeData3 ¸¡µãÊý¾Ý3
  * @retval  x
  */
void FLASH_WriteThreeFloatData( uint32_t startAddress,  float writeData1, 
                                                        float writeData2,
                                                        float writeData3)
{
  uint32_t wData[3];
  
  /* ´ýÐ´ÈëµÄ¸¡µãÊý¾ÝÇ¿ÖÆ×ªÎªÕûÐÎÊý¾Ý£¬·½±ãÊý¾ÝÐ´Èë */
  wData[0] = *(uint32_t *)(&writeData1);
  wData[1] = *(uint32_t *)(&writeData2);
  wData[2] = *(uint32_t *)(&writeData3);
  
  FLASH_WriteWordData(startAddress,&wData[0],3);
}

/*  Î´Ê¹ÓÃ£¬ÔÝÊ±ÆÁ±Î

//¶ÁÈ¡Ö¸¶¨µØÖ·µÄ°ë×Ö(16Î»Êý¾Ý)
static uint16_t FLASH_ReadHalfWord(uint32_t address)
{
  return *(__IO uint16_t*) address; 
}
//¶ÁÈ¡Ö¸¶¨µØÖ·µÄ×Ö(32Î»Êý¾Ý)
static uint32_t FLASH_ReadWord(uint32_t address)
{
  return *(__IO uint32_t*) address; 
}
//¶ÁÈ¡Ö¸¶¨µØÖ·µÄ°ë×Ö(32Î»Êý¾Ý)
static float FLASH_ReadFloatWord(uint32_t address)
{
  return *(__IO float*) address; 
}
*/

/**
  * @brief   ´Óflash¶Á³ö¸¡µãÐÍÊý¾Ý
  * @param   startAddress ¶Á³ö´¦µÄÆðÊ¼µØÖ·
  * @param   *readData ´æ´¢¶Á³öÊý¾ÝµÄÖ¸Õë±äÁ¿
  * @param   countToRead ¶Á³öÊý¾Ý³¤¶È
  * @retval  x
  */
void FLASH_ReadFloatData(uint32_t startAddress,float *readData,uint16_t countToRead)
{
  uint16_t dataIndex;
  for( dataIndex = 0 ; dataIndex < countToRead ; dataIndex++ )
  {
    readData[dataIndex] = *(__IO float*) (startAddress + dataIndex * 4);
  }
}












