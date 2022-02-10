#ifndef INC_ADS7843_H_
#define INC_ADS7843_H_

#include "main.h"

/** PD0 */
#define ADS_CTRL_PD0              (1 << 0)
/** PD1 */
#define ADS_CTRL_PD1              (1 << 1)
/** SER/DFR */
#define ADS_CTRL_DFR              (1 << 2)
/** Mode */
#define ADS_CTRL_EIGHT_BITS_MOD   (1 << 3)
/** Start Bit */
#define ADS_CTRL_START            (1 << 7)
/** Address setting */
#define ADS_CTRL_SWITCH_SHIFT     4

/** Get X position command */
#define CMD_Y_POSITION ((1 << ADS_CTRL_SWITCH_SHIFT) | ADS_CTRL_START |\
		ADS_CTRL_PD0 | ADS_CTRL_PD1)

/** Get Y position command */
#define CMD_X_POSITION ((5 << ADS_CTRL_SWITCH_SHIFT) | ADS_CTRL_START |\
		ADS_CTRL_PD0 | ADS_CTRL_PD1)

/** Enable penIRQ */
#define CMD_ENABLE_PENIRQ  ((1 << ADS_CTRL_SWITCH_SHIFT) | ADS_CTRL_START)

#define ADS7843_TIMEOUT        5000000
/** 2us min (tCSS) <=> 200/100 000 000 = 2us */
#define DELAY_BEFORE_SPCK          200
/** 5us min (tCSH) <=> (32 * 15) / (100 000 000) = 5us */
#define DELAY_BETWEEN_CONS_COM     0xf



uint32_t ADS7843_Init(SPI_HandleTypeDef *spiPointer, uint16_t dispWidth, uint16_t dispHeight);
uint32_t ADS7843_SendCmd(uint8_t cmd, uint16_t *retVal);
uint32_t ADS7843_IsPressed(void);
void ADS7843_GetRawPoint(uint16_t *p_x, uint16_t *p_y);
void ADS7843_GetXYCords(uint16_t *cordX, uint16_t *cordY);
uint8_t ADS7843_Calibrate(void);

#endif /* INC_ADS7843_H_ */
