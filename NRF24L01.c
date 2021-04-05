#include <stdint.h>
#include "NRF24L01.h"
#include "../target.h"

static uint8_t (*p_NRF_SPI_Exchange)(uint8_t);
static t_NRF_RX_PIPE NFR_RxPipes[6];
static t_NRF_Registers NRFChip;

void NRF24L01_Init(uint8_t (*SPI_Exchange)(uint8_t))
{
    NRF_Set_SPI_Handler(SPI_Exchange);
    NRF_PIN_CE = 0;
    NRF_PIN_CSN = 1;
}

void NRF_OpenReadingPipe(uint8_t PipeNo, uint8_t PipeAddr[], uint8_t PayloadLength, uint8_t AutoAck, uint8_t Enable)
{
    uint8_t i;
    uint8_t AW = NRFChip.AW.byte + 2u;
    
    NFR_RxPipes[PipeNo].PAY_LEN = PayloadLength;
    
    for (i = 0; i < AW; i++)
    {
        NFR_RxPipes[PipeNo].PIPE_ADDR[i] = PipeAddr[i];
    }
    if (Enable != 0u)
    {
        NRF_PipeEnable(PipeNo);
    }
    else { /* Do Nothing */ }
    
    if (AutoAck != 0u)
    {
        NRF_PipeEnableAA(PipeNo);
    }
    else { /* Do Nothing */ }
    
    NRF_Write_Register(REG_NRF_RX_ADDR_P0 + PipeNo, NFR_RxPipes[PipeNo].PIPE_ADDR, (PipeNo > 1u) ? 1u : AW);
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_RX_PW_P0 + PipeNo, &NFR_RxPipes[PipeNo].PAY_LEN, 1u);
}

void NRF_SetTxAddr(uint8_t *PipeAddr)
{
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_TX_ADDR, PipeAddr, 5u);
}

void NRF_PipeEnable(uint8_t PipeNo)
{
    uint8_t EN;
    NRF_Read_Register(REG_NRF_EN_RXADDR, &EN, 1u);
    EN |= 1u << PipeNo;
    NRF_Write_Register(REG_NRF_EN_RXADDR, &EN, 1u);
}

void NRF_PipeDisable(uint8_t PipeNo)
{
    uint8_t EN;
    NRF_Read_Register(REG_NRF_EN_RXADDR, &EN, 1u);
    EN &= (1u << PipeNo) ^ 0xFFu;
    NRF_Write_Register(REG_NRF_EN_RXADDR, &EN, 1u);
}

void NRF_PipeEnableAA(uint8_t PipeNo)
{
    uint8_t AA;
    NRF_Read_Register(REG_NRF_EN_AA, &AA, 1u);
    AA |= 1u << PipeNo;
    NRF_Write_Register(REG_NRF_EN_AA, &AA, 1u);
}

void NRF_PipeDisableAA(uint8_t PipeNo)
{
    uint8_t AA;
    NRF_Read_Register(REG_NRF_EN_AA, &AA, 1u);
    AA &= (1u << PipeNo) ^ 0xFFu;
    NRF_Write_Register(REG_NRF_EN_AA, &AA, 1u);
}

void NRF_SetPrimaryAs(uint8_t asPrimary)
{
    NRF_Read_Register(REG_NRF_CONFIG, &NRFChip.CONFIG.byte, 1u);
    NRFChip.CONFIG.s.PRIM_RX = asPrimary;
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_CONFIG, &NRFChip.CONFIG.byte, 1u);
}

void NRF_SetCRCLen(uint8_t Len)
{
    NRF_Read_Register(REG_NRF_CONFIG, &NRFChip.CONFIG.byte, 1u);
    NRFChip.CONFIG.s.CRCO = Len;
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_CONFIG, &NRFChip.CONFIG.byte, 1u);
}

void NRF_SetRFChannel(uint8_t RF_Channel)
{
    NRFChip.RF_CH.byte = (RF_Channel > 125u) ? 125u : RF_Channel;
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_RF_CH, &NRFChip.RF_CH.byte, 1u);
}

void NRF_SetRFPower(uint8_t RF_Pow)
{
    NRF_Read_Register(REG_NRF_RF_SETUP, &NRFChip.RF_SETUP.byte, 1u);
    NRFChip.RF_SETUP.s.RF_PWR = (RF_Pow > 3u) ? 3u : RF_Pow;
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_RF_SETUP, &NRFChip.RF_SETUP.byte, 1u);
}

void NRF_SetRFDataRate(uint8_t Datarate)
{
    NRF_Read_Register(REG_NRF_RF_SETUP, &NRFChip.RF_SETUP.byte, 1u);
    NRFChip.RF_SETUP.s.RF_DR = Datarate & 1u;
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_RF_SETUP, &NRFChip.RF_SETUP.byte, 1u);
}

void NRF_SetAddrWidth(uint8_t AddressWidth)
{
    AddressWidth += (AddressWidth == 0u) ? 1u : 0u;
    AddressWidth = (AddressWidth > 3u) ? 3u : 0u;
    NRFChip.AW.byte = AddressWidth;
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_SETUP_AW, &NRFChip.AW.byte, 1u);
}

void NRF_SetART(uint8_t count, uint8_t delay)
{
    NRFChip.SETUP_RETR.s.ARC = count;
    NRFChip.SETUP_RETR.s.ARD = delay;
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_SETUP_RETR, &NRFChip.SETUP_RETR.byte, 1u);
}

void NRF_StartListening(void)
{
    NRF_Read_Register(REG_NRF_CONFIG, &NRFChip.CONFIG.byte, 1u);
    NRFChip.CONFIG.s.PRIM_RX = 1u;
    NRFChip.CONFIG.s.PWR_UP = 1u;
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_CONFIG, &NRFChip.CONFIG.byte, 1u);
    NRF_PIN_CE = 1;
}

void NRF_StopListening(void)
{
    NRF_Read_Register(REG_NRF_CONFIG, &NRFChip.CONFIG.byte, 1u);
    NRFChip.CONFIG.s.PRIM_RX = 0u;
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_CONFIG, &NRFChip.CONFIG.byte, 1u);
    NRF_PIN_CE = 0;
}

uint8_t NRF_Available(void)
{
    uint8_t PipiNo = 255;
    NRF_GetStatus();
    if (NRFChip.STATUS.s.RX_DR == 1)
    {
        PipiNo = NRFChip.STATUS.s.RX_P_NO;
    }
    return PipiNo;
}

uint8_t NRF_GetStatus(void)
{
    NRF_PIN_CSN = 0;
    NRFChip.STATUS.byte = p_NRF_SPI_Exchange(CMD_NRF_NOP);
    NRF_PIN_CSN = 1;
    return NRFChip.STATUS.byte;
}

void NRF_SetMaskIRQ(uint8_t IRQMask)
{
    NRF_Read_Register(REG_NRF_CONFIG, &NRFChip.CONFIG.byte, 1u);
    NRFChip.CONFIG.byte |= IRQMask & 0xF0;
    NRFChip.CONFIG.byte &= IRQMask | 0x0F;
    NRFChip.STATUS.byte = NRF_Write_Register(REG_NRF_CONFIG, &NRFChip.CONFIG.byte, 1u);
}

void NRF_ReadPayload(uint8_t *Payload, uint8_t PayloadLength)
{
    uint8_t i;
    NRF_PIN_CSN = 0;
    NRFChip.STATUS.byte = p_NRF_SPI_Exchange(CMD_NRF_R_RX_PAYLOAD);
    for (i = 0; i < PayloadLength; i++)
    {
        Payload[i] = p_NRF_SPI_Exchange(CMD_NRF_NOP);
    }
    NRF_PIN_CSN = 1;
    NRFChip.STATUS.s.RX_DR = 1u;
    NRF_Write_Register(REG_NRF_STATUS, &NRFChip.STATUS.byte, 1u);
}

void NRF_WritePayload(uint8_t *Payload, uint8_t PayloadLength)
{
    uint8_t i;
    NRF_PIN_CSN = 0;
    NRFChip.STATUS.byte = p_NRF_SPI_Exchange(CMD_NRF_W_TX_REGISTER);
    for (i = 0; i < PayloadLength; i++)
    {
        p_NRF_SPI_Exchange(Payload[i]);
    }
    NRF_PIN_CE = 1;
    __delay_us(25);
    NRF_PIN_CE = 0;
    NRF_PIN_CSN = 1;
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
        *Bytes++ = p_NRF_SPI_Exchange(CMD_NRF_NOP);
    }
    NRF_PIN_CSN = 1;
    return Status;
}

void NRF_StatusHandler(void)
{
    u_NRF_Status StatusReg;
    u_NRF_FIFO_Status NRF_Fifo;
    StatusReg.byte = NRF_Read_Register(REG_NRF_FIFO_STATUS, &NRF_Fifo.byte, 1u);
    switch (StatusReg.byte >> 4)
    {
        case 1:     /* MAX_RT on going */
            break;
        case 2:     /* TX_DS on going */
            break;
        case 4:     /* RX_DR on going */
            if (NRF_Fifo.s.RX_FULL == 1u)
            {
                p_NRF_SPI_Exchange(CMD_NRF_FLUSH_RX);
            }
            else { /* Do nothing */ }
            break;
        default:
            break;
    }
}

void NRF_Set_SPI_Handler (uint8_t(*SPI_Handler)(uint8_t))
{
    p_NRF_SPI_Exchange = SPI_Handler;
}