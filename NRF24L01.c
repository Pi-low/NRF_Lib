#include "NRF24L01.h"

static t_NRF_Config NRF_REG_CONFIG;
static t_NRF_STATUS NRF_REG_STATUS;
static t_NRF_RX_ADDR_5_BYTES NRF_ADDR_PIPE_0;

uint8_t (*p_NRF_SPI_Exchange)(uint8_t);

void NRF24L01_Init(void)
{
    Set_SPI_Handler(SoftSPI_Exchange);
    NRF_PIN_CE = 0;
    NRF_PIN_CSN = 1;
    NRF_REG_STATUS.byte = 0;
    NRF_REG_CONFIG.byte = 0;
    
    NRF_ADDR_PIPE_0.Byte0 = 1;
    NRF_ADDR_PIPE_0.Byte1 = 2;
    NRF_ADDR_PIPE_0.Byte2 = 3;
    NRF_ADDR_PIPE_0.Byte3 = 4;
    NRF_ADDR_PIPE_0.Byte4 = 5;
}

void NRF_SET_PRX(void)
{
    t_NRF_Config R_CONFIG;
    R_CONFIG.byte = 0;
    
    NRF_REG_CONFIG.PWR_UP = 1u;
    NRF_REG_CONFIG.EN_CRC = 1u;
    NRF_REG_CONFIG.PRIM_RX = 1u;
    
    NRF_REG_STATUS.byte = NRF_Write_Register(REG_NRF_CONFIG, &NRF_REG_CONFIG.byte, 1);
    PrintUART("\r\nSTATUS REGISTER (1): ");
    UART_PNbase(NRF_REG_STATUS.byte, UART_BIN, "\r\n");
    
    NRF_REG_STATUS.byte = NRF_Read_Register(REG_NRF_RX_ADDR_P0, NRF_ADDR_PIPE_0.ByteArray, 5);
    PrintUART("\r\nADDR PIPE 0: : ");
    UART_PNbase(NRF_ADDR_PIPE_0.Byte4, UART_HEX, " ");
    UART_PNbase(NRF_ADDR_PIPE_0.Byte3, UART_HEX, " ");
    UART_PNbase(NRF_ADDR_PIPE_0.Byte2, UART_HEX, " ");
    UART_PNbase(NRF_ADDR_PIPE_0.Byte1, UART_HEX, " ");
    UART_PNbase(NRF_ADDR_PIPE_0.Byte0, UART_HEX, "\r\n");
}

uint8_t NRF_Write_Register(uint8_t Register, uint8_t *Bytes, uint8_t Length)
{
    uint8_t Status = 0u;
    NRF_PIN_CSN = 0;
    Status = p_NRF_SPI_Exchange(CMD_NRF_W_REGISTER | Register);
    while (Length--)
    {
        p_NRF_SPI_Exchange(*Bytes++);
    }
    NRF_PIN_CSN = 1;
    return Status;
}

uint8_t NRF_Read_Register(uint8_t Register, uint8_t *Bytes, uint8_t Length)
{
    uint8_t Status = 0u;
    NRF_PIN_CSN = 0;
    Status = p_NRF_SPI_Exchange(CMD_NRF_R_REGISTER | Register);
    while (Length--)
    {
        *Bytes++ = p_NRF_SPI_Exchange(0xFF);
    }
    NRF_PIN_CSN = 1;
    return Status;
}

void NRF_IRQ_ISR_Handler(void)
{
    
}

void Set_SPI_Handler (uint8_t(*SPI_Handler)(uint8_t))
{
    p_NRF_SPI_Exchange = SPI_Handler;
}