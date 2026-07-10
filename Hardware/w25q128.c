#include "w25q128.h"

#define W25Q128_CMD_WRITE_ENABLE        (0x06u)
#define W25Q128_CMD_READ_STATUS1        (0x05u)
#define W25Q128_CMD_READ_DATA           (0x03u)
#define W25Q128_CMD_PAGE_PROGRAM        (0x02u)
#define W25Q128_CMD_SECTOR_ERASE        (0x20u)
#define W25Q128_CMD_READ_ID             (0x90u)
#define W25Q128_DUMMY_BYTE              (0xFFu)
#define W25Q128_SECTOR_SIZE             (4096u)
#define W25Q128_BUSY_TIMEOUT            (1000000u)

uint8_t spi_read_write_byte(uint8_t dat)
{
    uint8_t data;
    volatile uint32_t timeout = 100000u;

    DL_SPI_transmitData8(SPI_INST, dat);
    while(DL_SPI_isBusy(SPI_INST) && (timeout > 0u))
    {
        timeout--;
    }
    data = DL_SPI_receiveData8(SPI_INST);
    timeout = 100000u;
    while(DL_SPI_isBusy(SPI_INST) && (timeout > 0u))
    {
        timeout--;
    }

    return data;
}

uint16_t W25Q128_readID(void)
{
    uint16_t id = 0u;

    SPI_CS(0);
    spi_read_write_byte(W25Q128_CMD_READ_ID);
    spi_read_write_byte(0x00u);
    spi_read_write_byte(0x00u);
    spi_read_write_byte(0x00u);
    id |= (uint16_t)spi_read_write_byte(W25Q128_DUMMY_BYTE) << 8;
    id |= spi_read_write_byte(W25Q128_DUMMY_BYTE);
    SPI_CS(1);

    return id;
}

void W25Q128_write_enable(void)
{
    SPI_CS(0);
    spi_read_write_byte(W25Q128_CMD_WRITE_ENABLE);
    SPI_CS(1);
}

void W25Q128_wait_busy(void)
{
    uint8_t status;
    uint32_t timeout = W25Q128_BUSY_TIMEOUT;

    do
    {
        SPI_CS(0);
        spi_read_write_byte(W25Q128_CMD_READ_STATUS1);
        status = spi_read_write_byte(W25Q128_DUMMY_BYTE);
        SPI_CS(1);
        timeout--;
    } while(((status & 0x01u) != 0u) && (timeout > 0u));
}

void W25Q128_erase_sector(uint32_t addr)
{
    addr *= W25Q128_SECTOR_SIZE;

    W25Q128_write_enable();
    W25Q128_wait_busy();

    SPI_CS(0);
    spi_read_write_byte(W25Q128_CMD_SECTOR_ERASE);
    spi_read_write_byte((uint8_t)(addr >> 16));
    spi_read_write_byte((uint8_t)(addr >> 8));
    spi_read_write_byte((uint8_t)addr);
    SPI_CS(1);

    W25Q128_wait_busy();
}

void W25Q128_write(uint8_t *buffer, uint32_t addr, uint16_t numbyte)
{
    uint16_t i;

    W25Q128_erase_sector(addr / W25Q128_SECTOR_SIZE);
    W25Q128_write_enable();
    W25Q128_wait_busy();

    SPI_CS(0);
    spi_read_write_byte(W25Q128_CMD_PAGE_PROGRAM);
    spi_read_write_byte((uint8_t)(addr >> 16));
    spi_read_write_byte((uint8_t)(addr >> 8));
    spi_read_write_byte((uint8_t)addr);
    for(i = 0u; i < numbyte; i++)
    {
        spi_read_write_byte(buffer[i]);
    }
    SPI_CS(1);

    W25Q128_wait_busy();
}

void W25Q128_read(uint8_t *buffer, uint32_t read_addr, uint16_t read_length)
{
    uint16_t i;

    SPI_CS(0);
    spi_read_write_byte(W25Q128_CMD_READ_DATA);
    spi_read_write_byte((uint8_t)(read_addr >> 16));
    spi_read_write_byte((uint8_t)(read_addr >> 8));
    spi_read_write_byte((uint8_t)read_addr);
    for(i = 0u; i < read_length; i++)
    {
        buffer[i] = spi_read_write_byte(W25Q128_DUMMY_BYTE);
    }
    SPI_CS(1);
}
