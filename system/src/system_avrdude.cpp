
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIB_AVRDUDE_H
#define __LIB_AVRDUDE_H

#if (PLATFORM_ID==88)

#include "spark_wiring.h"
#include "system_task.h"
#include "system_update.h"
#include "system_avrdude.h"
#include "ota_flash_hal.h"
#include "rgbled.h"
#include "file_transfer.h"
#include "spi_flash.h"

static Stream *avrdudeSeial = NULL;
static uint32_t NAK_TIMEOUT = (0x10000000);

static uint8_t rx_tx_buf[160];

static uint8_t  rw_flag=0, invalid_flag=0, first_block=0;
static uint32_t block_size, page_addr;
static uint32_t currentAddr, start_addr, end_addr, firmware_size, recieve_size;
static uint32_t computedCRC, rawCRC, remainder_len;


static int32_t Avrdude_recieve_byte(uint8_t& c, uint32_t timeout)
{
    while (timeout-- > 0)
    {
        if (avrdudeSeial->available())
        {
            c = avrdudeSeial->read();
            return 0;
        }
    }
    return -1;
}


int32_t receive_firmware(FileTransfer::Descriptor& file)
{
    uint8_t c;
    uint8_t i;
    uint8_t buf[3];

    for(;;)
    {
        if(Avrdude_recieve_byte(c, NAK_TIMEOUT) != 0)
        {
            return -1;
        }
        switch(c)
        {
        case 'S' :
            rw_flag = 0;
            invalid_flag = 0;
            first_block = 0;
            firmware_size = 0;
            recieve_size = 0;
            computedCRC = 0;
            rawCRC = 0;
            remainder_len = 0;
            avrdudeSeial->print("RBL-DUO");
            break;
        case 'V' :
            avrdudeSeial->write((uint8_t)'1');
            avrdudeSeial->write((uint8_t)'0');
            break;
        case 'v' :
            avrdudeSeial->write((uint8_t)'1');
            avrdudeSeial->write((uint8_t)'0');
            break;
        case 'p' :
            avrdudeSeial->write((uint8_t)'S');
            break;
        case 'a' :
            avrdudeSeial->write((uint8_t)'Y');
            break;
        case 'b' :
            avrdudeSeial->write((uint8_t)'Y');
            avrdudeSeial->write((uint8_t)0x00);
            avrdudeSeial->write((uint8_t)0x80);
            break;
        case 't' :
            avrdudeSeial->write((uint8_t)'D');
            avrdudeSeial->write((uint8_t)0x00);
            break;
        case 'T' :
            if(Avrdude_recieve_byte(c, NAK_TIMEOUT) !=0 )
            {
                return -1;
            }
            if(c == 'D')
            {
                avrdudeSeial->write((uint8_t)0x0d);
            }
            break;
        case 'P' :
            avrdudeSeial->write((uint8_t)0x0d);
            break;
        case 's' :
            avrdudeSeial->write((uint8_t)0x0B);
            avrdudeSeial->write((uint8_t)0x90);
            avrdudeSeial->write((uint8_t)0x1E);
            break;
        case 'L' :
            avrdudeSeial->write((uint8_t)0x0d);
            return 1;
            break;
        case 'E' :
            avrdudeSeial->write((uint8_t)0x0d);
            return 1;
            break;
        case 'A' :
            for (i = 0; i < 2; i++)
            {
                if (Avrdude_recieve_byte(buf[i], NAK_TIMEOUT) != 0)
                {
                    return -1;
                }
            }
            //Page address.
            page_addr = (buf[0] << 9);
            page_addr |= (buf[1] << 1);
            if(page_addr == 0)
            {	//Initialize all variables.
                first_block = 1;
                if(rw_flag == 0)
                {
                    rw_flag = 1;

                    RGB.control(true);
                    RGB.color(RGB_COLOR_MAGENTA);
                    SPARK_FLASH_UPDATE = 1;
                    TimingFlashUpdateTimeout = 0;
                }
                //Get external flash start address
                invalid_flag = 0;
                remainder_len=0;
                recieve_size = 0;
                currentAddr = HAL_OTA_FlashAddress();
            }

            avrdudeSeial->write((uint8_t)'\r');
            break;
        case 'g':
            //External flash readback.
            for (i = 0; i < 3; i++)
            {
                if (Avrdude_recieve_byte(buf[i], NAK_TIMEOUT) != 0)
                {
                    return -1;
                }
            }
            block_size = buf[0];
            block_size = (block_size<<8) + buf[1];

            if(buf[2] == 'F')
            {
                memset(rx_tx_buf, 0x00, block_size);

                LED_Toggle(LED_RGB);
                sFLASH_Init();
                sFLASH_ReadBuffer(rx_tx_buf, currentAddr, block_size);
                currentAddr += block_size;

                if(invalid_flag == 0)
                {
                    if(recieve_size+block_size < firmware_size)
                    {
                        recieve_size += block_size;
                    }
                    else
                    {
                        if(recieve_size+block_size >= (firmware_size+4))
                        {
                            rx_tx_buf[firmware_size-recieve_size]   = (uint8_t)(rawCRC>>24);
                            rx_tx_buf[firmware_size-recieve_size+1] = (uint8_t)(rawCRC>>16);
                            rx_tx_buf[firmware_size-recieve_size+2] = (uint8_t)(rawCRC>>8);
                            rx_tx_buf[firmware_size-recieve_size+3] = (uint8_t)(rawCRC);

                            invalid_flag=2;
                        }
                        else
                        {
                            uint32_t temp = rawCRC;
                            for(uint8_t i=0; i<(recieve_size+block_size-firmware_size); i++)
                            {
                                temp = (uint8_t)((rawCRC&0xFF000000)>>24);
                                rawCRC <<= 8;
                                rx_tx_buf[firmware_size-recieve_size+i] = temp;
                            }
                            remainder_len = (4-(recieve_size+block_size-firmware_size));
                            invalid_flag=1;
                        }
                        recieve_size += block_size;
                    }
                }
                else if(invalid_flag == 1)
                {
                    uint32_t temp = rawCRC;
                    for(uint8_t i=0; i<remainder_len; i++)
                    {
                        temp = (uint8_t)((rawCRC&0xFF000000)>>24);
                        rawCRC <<= 8;
                        rx_tx_buf[i] = temp;
                    }
                    invalid_flag = 2;
                }

                for(i=0; i<block_size; i++)
                {
                    avrdudeSeial->write((uint8_t)rx_tx_buf[i]);
                }

                //avrdudeSeial->write((uint8_t)(computedCRC>>24));
                //avrdudeSeial->write((uint8_t)(computedCRC>>16));
                //avrdudeSeial->write((uint8_t)(computedCRC>>8));
                //avrdudeSeial->write((uint8_t)(computedCRC>>0));
            }
            else if(buf[2] == 'E')
            {
                //Not support eeprom.
                avrdudeSeial->write((uint8_t)'?');
            }
            break;
        case 'B' :
            for (i = 0; i < 3; i++)
            {
                if (Avrdude_recieve_byte(buf[i], NAK_TIMEOUT) != 0)
                {
                    return -1;
                }
            }
            //Block size.
            block_size = buf[0];
            block_size = (block_size<<8) + buf[1];
            if(buf[2] == 'F')
            {	//Write external flash
                uint16_t index;

                memset(rx_tx_buf, 0x00, block_size);
                //Page size is 4096, erase consecutive two pages.Maybe burning CRC32 would cross page.
                if( (invalid_flag == 0) && ((currentAddr & 0x00000FFF) == 0) )
                    HAL_FLASH_Begin(currentAddr, 8192, NULL);
                //Get block.
                for (index = 0; index<block_size; index++)
                {
                    if (Avrdude_recieve_byte(rx_tx_buf[index], NAK_TIMEOUT) != 0)
                    {
                        return -1;
                    }
                }

                if(first_block)
                {	//If first block, get firmware information.
                    first_block = 0;
                    start_addr = *((uint32_t *)(&rx_tx_buf[0]));
                    end_addr = *((uint32_t *)(&rx_tx_buf[4]));
                    if(end_addr>start_addr)
                    {
                        firmware_size = end_addr - start_addr;
                    }
                    else
                    {
                        return -3;
                    }
                }
                if(invalid_flag == 0)
                {
                    if(recieve_size+block_size < firmware_size)
                    {	//Not at the end.
                        if(HAL_FLASH_Update(rx_tx_buf, currentAddr, block_size, NULL) != 0)
                        {
                            return -2;
                        }
                        recieve_size += block_size;
                        currentAddr += block_size;
                    }
                    else
                    {
                        uint32_t receive_len = firmware_size-recieve_size;

                        if(recieve_size+block_size >= firmware_size+4)
                        {	//This packet contains 4bytes-CRC32.
                            rawCRC = rx_tx_buf[receive_len];
                            rawCRC = (rawCRC<<8) + rx_tx_buf[receive_len+1];
                            rawCRC = (rawCRC<<8) + rx_tx_buf[receive_len+2];
                            rawCRC = (rawCRC<<8) + rx_tx_buf[receive_len+3];
                            invalid_flag = 2;
                        }
                        else
                        {
                            //This packet not contains enough 4Bytes-CRC32
                            for(uint8_t i=0; i<(recieve_size+block_size-firmware_size); i++)
                            {
                                rawCRC = (rawCRC<<8) +  rx_tx_buf[receive_len+i];
                            }
                            remainder_len = 4 - (recieve_size+block_size-firmware_size);
                            invalid_flag = 1;
                        }
                        //Update data but CRC32
                        if(HAL_FLASH_Update(rx_tx_buf, currentAddr, receive_len, NULL) != 0)
                        {
                            return -2;
                        }

                        currentAddr += receive_len;
                        recieve_size += receive_len;
                        //Calculate CRC32.
                        computedCRC = sFLASH_Compute_CRC32((HAL_OTA_FlashAddress()), recieve_size);
                        uint8_t buf[4];
                        buf[0] = (uint8_t)((computedCRC & 0xFF000000) >> 24);
                        buf[1] = (uint8_t)((computedCRC & 0xFF0000) >> 16);
                        buf[2] = (uint8_t)((computedCRC & 0xFF00) >> 8);
                        buf[3] = (uint8_t)(computedCRC & 0xFF);
                        //Update CRC32.
                        if(HAL_FLASH_Update(buf, currentAddr, 4, NULL) != 0)
                        {
                            return -2;
                        }
                    }
                }
                else if(invalid_flag == 1)
                {	//Just get remainder raw crc32.
                    for(uint8_t i=0; i<remainder_len; i++)
                    {
                        rawCRC = (rawCRC<<8) +  rx_tx_buf[i];
                    }
                    invalid_flag = 2;
                }

                LED_Toggle(LED_RGB);

                avrdudeSeial->write((uint8_t)'\r');
            }
            else if(buf[2] == 'E')
            {
                avrdudeSeial->write((uint8_t)'?');
            }
            break;
        default:
            break;
        }
    }
}

bool Avrdude_Serial_Flash_Update(Stream *serialObj, FileTransfer::Descriptor& file, void* reserved)
{
    int32_t size;

    avrdudeSeial = serialObj;

    size = receive_firmware(file);
    if (size > 0)
    {
        avrdudeSeial->println("\r\nDownloaded file successfully!");
        avrdudeSeial->print("Size: ");
        avrdudeSeial->print(size);
        avrdudeSeial->println(" bytes");
        avrdudeSeial->flush();

        Spark_Finish_Firmware_Update(file, size>0 ? 1 : 0, NULL);
        return true;
    }
    else if (size == -1)
    {
        avrdudeSeial->println("Recieve timeout!");
    }
    else if (size == -2)
    {
        avrdudeSeial->println("Write flash fail!");
    }
    else if (size == -3)
    {
        avrdudeSeial->println("Firmware size fail!");
    }
    else
    {
        avrdudeSeial->println("Unknow fail!");
    }
    return false;
}

#endif  /* PLATFORM_ID==88 */

#endif  /* __LIB_AVRDUDE_H */
