/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

#ifndef NRF24L01_H
#define	NRF24L01_H

#include <xc.h>
#include "../../mcc_generated_files/mcc.h"
#include "../01_Appli/Application.h"
#include "../02_Soft_SPI/Soft_SPI.h"

/*------------------------------------------------*/
/*              NRF24L01 PHYSICAL INTERFACE       */
/*------------------------------------------------*/
#define NRF_PIN_CE                  CE_LAT
#define NRF_PIN_CSN                 CS__LAT

/*------------------------------------------------*/
/*              NRF24L01 SPI commands             */
/*------------------------------------------------*/
#define CMD_NRF_R_REGISTER          0x00
#define CMD_NRF_W_REGISTER          0x20
#define CMD_NRF_R_RX_PAYLOA         0x61
#define CMD_NRF_W_TX_REGISTR        0xA0
#define CMD_NRF_FLUSH_TX            0xE1
#define CMD_NRF_FLUSH_RX            0xE2
#define CMD_NRF_REUSE_TX_PL         0xE3
#define CMD_NRF_ACTIVATE            0x50
#define CMD_NRF_R_RX_PL_WID         0x60
#define CMD_NRF_W_ACK_PAYLOAD       0xA8
#define CMD_NRF_W_TX_PAYLOAD_NO_ACK 0xB0
#define CMD_NRF_NOP                 0x00

/*------------------------------------------------*/
/*              NRF24L01 Register Addresses       */
/*------------------------------------------------*/
#define REG_NRF_CONFIG              0x00
#define REG_NRF_EN_AA               0x01
#define REG_NRF_EN_RXADDR           0x02
#define REG_NRF_SETUP_AW            0x03
#define REG_NRF_SETUP_RETR          0x04
#define REG_NRF_RF_CH               0x05
#define REG_NRF_RF_SETUP            0x06
#define REG_NRF_STATUS              0x07
#define REG_NRF_OBSERVE_TX          0x08
#define REG_NRF_CD                  0x09
#define REG_NRF_RX_ADDR_P0          0x0A
#define REG_NRF_RX_ADDR_P1          0x0B
#define REG_NRF_RX_ADDR_P2          0x0C
#define REG_NRF_RX_ADDR_P3          0x0D
#define REG_NRF_RX_ADDR_P4          0x0E
#define REG_NRF_RX_ADDR_P5          0x0F
#define REG_NRF_TX_ADDR             0x10
#define REG_NRF_RX_PW_P0            0x11
#define REG_NRF_RX_PW_P1            0x12
#define REG_NRF_RX_PW_P2            0x13
#define REG_NRF_RX_PW_P3            0x14
#define REG_NRF_RX_PW_P4            0x15
#define REG_NRF_RX_PW_P5            0x16
#define REG_NRF_FIFO_STATUS         0x17
#define REG_NRF_DYNPD               0x1C
#define REG_NRF_FEATURE             0x1D

/*----------------------------------------------*/
/*                ADDR 0: CONFIG                */
/*----------------------------------------------*/
typedef union {
    uint8_t byte;
    struct {
        uint8_t PRIM_RX:        1;
        uint8_t PWR_UP:         1;
        uint8_t CRCO:           1;
        uint8_t EN_CRC:         1;
        uint8_t MASK_MAX_RT:    1;
        uint8_t MASK_TX_DS:     1;
        uint8_t MASK_RX_DR:     1;
    };
} t_NRF_Config;

/*----------------------------------------------*/
/*                ADDR 1: ENAA                  */
/*----------------------------------------------*/
typedef union {
    uint8_t byte;
    struct {
        uint8_t ENAA_P0:        1;
        uint8_t ENAA_P1:        1;
        uint8_t ENAA_P2:        1;
        uint8_t ENAA_P3:        1;
        uint8_t ENAA_P4:        1;
        uint8_t ENAA_P5:        1;
    };
}t_NFR_EN_AA;

/*----------------------------------------------*/
/*                ADDR 2: EN_RXADDR             */
/*----------------------------------------------*/
typedef union {
    uint8_t byte;
    struct {
        uint8_t ERX_P0:         1;
        uint8_t ERX_P1:         1;
        uint8_t ERX_P2:         1;
        uint8_t ERX_P3:         1;
        uint8_t ERX_P4:         1;
        uint8_t ERX_P5:         1;
    };
}t_NRF_EN_RXADDR;

/*----------------------------------------------*/
/*                ADDR 3: SETUP_AW              */
/*----------------------------------------------*/
typedef enum{
    NRF_AW_3_BYTES = 1u,
    NRF_AW_4_BYTES = 2u,
    NRF_AW_5_BYTES = 3u
}e_NRF_Address_Width;

typedef union {
    uint8_t byte;
    struct {
        uint8_t AW:             2; /* Address width */
    };
}t_NRF_SETUP_AW;

/*----------------------------------------------*/
/*                ADDR 4: SETUP_RETR            */
/*----------------------------------------------*/
typedef union {
    uint8_t byte;
    struct {
        uint8_t ARC:             4; /* Auto Retransmit Count*/
        uint8_t ARD:             4; /* Auto Retransmit Delay n*250µs*/
    };
}t_NRF_SETUP_RETR;

/*----------------------------------------------*/
/*                ADDR 5: RF_CH                 */
/*----------------------------------------------*/
typedef union {
    uint8_t byte;
    struct {
        uint8_t RF_CH:          7; /* RF CH 0 -> 125 */
    };
}t_NRF_RF_CH;

/*----------------------------------------------*/
/*                ADDR 6: RF_SETUP              */
/*----------------------------------------------*/
typedef enum{
    NRF_ePWR_MIN =  0u,
    NRF_ePWR_LOW =  1u,
    NRF_ePWR_HIGH = 2u,
    NRF_ePWR_MAX =  3u
}e_NRF_RF_Power;

typedef union {
    uint8_t byte;
    struct {
        uint8_t LNA_HCURR:      1; /* Setup LNA gain */
        uint8_t RF_PW:          2; /* Set RF power */
        uint8_t RF_DR:          1; /* Set aire data rate */
        uint8_t PLL_LOCK:       1; /* Only used in test */
    };
}t_NRF_RF_SETUP;

/*----------------------------------------------*/
/*                ADDR 7: STATUS                */
/*----------------------------------------------*/
typedef enum{
    NRF_ePIPE0 = 0u,
    NRF_ePIPE1 = 1u,
    NRF_ePIPE2 = 2u,
    NRF_ePIPE3 = 3u,
    NRF_ePIPE4 = 4u,
    NRF_ePIPE5 = 5u,
    NRF_eEMPTY = 7u
            
}e_NRF_RX_P_No;

typedef union {
    uint8_t byte;
    struct {
        uint8_t TX_FULL:        1;
        uint8_t RX_P_NO:        3;
        uint8_t MAX_RT:         1;
        uint8_t TX_DS:          1;
        uint8_t RX_DR:          1;
    };
}t_NRF_STATUS;

/*----------------------------------------------*/
/*                ADDR 8: OBSERVE_TX            */
/*----------------------------------------------*/
typedef union {
    uint8_t byte;
    struct {
        uint8_t ACR_CNT:        4; /* Number of retransmitted packets */
        uint8_t PLOS_CNT:       4; /* Number of lost packets */
    };
}t_NRF_OBSERVE_TX;

/*----------------------------------------------*/
/*                ADDR 9: CARRIVER_DETECT       */
/*----------------------------------------------*/
typedef union {
    uint8_t byte;
    struct {
        uint8_t CD: 1;
    };
}t_NRF_DC;

/*----------------------------------------------*/
/*                ADDR 10-11: RX_ADDR_P0-P1     */
/*----------------------------------------------*/
typedef union {
    uint8_t ByteArray[5];
    struct {
        uint8_t Byte0;
        uint8_t Byte1;
        uint8_t Byte2;
        uint8_t Byte3;
        uint8_t Byte4;
    };
}t_NRF_RX_ADDR_5_BYTES;
typedef unsigned char t_NRF_RX_ADDR_1_BYTE;

/*----------------------------------------------*/
/*                ADDR 23: FIFO_STATUS          */
/*----------------------------------------------*/
typedef union {
    uint8_t byte;
    struct {
        uint8_t RX_EMPTY:       1;
        uint8_t RX_FULL:        1;
        uint8_t reserved:       2;
        uint8_t TX_EMPTY:       1;
        uint8_t TX_FULL:        1;
        uint8_t TX_REUSE:       1;
    };
}t_NRF_FIFO_STATUS;

/*----------------------------------------------*/
/*                ADDR 28: DYNPD                */
/*----------------------------------------------*/
typedef union {
    uint8_t byte;
    struct {
        uint8_t DPL_P0:          1;
        uint8_t DPL_P1:          1;
        uint8_t DPL_P2:          1;
        uint8_t DPL_P3:          1;
        uint8_t DPL_P4:          1;
        uint8_t DPL_P5:          1;
    };
}t_NRF_DYNPD;

/*----------------------------------------------*/
/*                ADDR 29: FEATURE              */
/*----------------------------------------------*/
typedef union {
    uint8_t byte;
    struct {
        uint8_t EN_DYN_ACK:     1;
        uint8_t EN_ACK_PAY:     1;
        uint8_t EN_DPL:         1;
    };
}t_NRF_FEATURE;

extern uint8_t (*p_NRF_SPI_Exchange)(uint8_t);

void NRF24L01_Init(void);
void NRF_SET_PRX(void);
uint8_t NRF_Write_Register(uint8_t Register, uint8_t *Bytes, uint8_t Length);
uint8_t NRF_Read_Register(uint8_t Register, uint8_t *Bytes, uint8_t Length);
void NRF_IRQ_ISR_Handler(void);
void Set_SPI_Handler(uint8_t (*SPI_Handler)(uint8_t));
#endif /* NRF24L01_H */