
#include "ads7843.h"
#include "stm32_adafruit_lcd.h"



	/*	 edit to match current CS, IRQ pins */

GPIO_TypeDef *IRQ_Port = TC_IRQ_GPIO_Port;
uint16_t IRQ_Pin = TC_IRQ_Pin;

GPIO_TypeDef *CS_Port = SPI2_CS_GPIO_Port;
uint16_t CS_Pin = SPI2_CS_Pin;



uint16_t width = 320;
uint16_t height = 240;

uint16_t calibX[3] = {1850, 90, 0};			// raw value at x cord 0, raw value at x cord max, reserved
uint16_t calibY[3] = {100, 1900, 0};		// raw value at y cord 0, raw value at y cord max, reserved

uint8_t swapXY = 1;
uint8_t invX = 1;
uint8_t invY = 0;

int16_t xOffset = 10;			// extra offsets for x and y
int16_t yOffset = 0;

uint8_t calibrated = 1;


SPI_HandleTypeDef *spi;

/**
 * \brief Send a command to the ADS7843 touch controller.
 */
uint32_t ADS7843_SendCmd(uint8_t cmd, uint16_t *retVal){
	uint32_t ret = HAL_OK;
	uint8_t bufferRX[2] = {0};
	uint8_t bufferTX[2] = {0};

	bufferTX[0] = cmd;

	HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);

 	ret = HAL_SPI_Transmit(spi, bufferTX, 1, 10);

	if(ret != HAL_OK){
		return ret;
	}

	while(HAL_SPI_GetState(spi) == HAL_SPI_STATE_BUSY);

	for(uint8_t i = 0; i < 2; i++){
		ret = HAL_SPI_Receive(spi, &bufferRX[i], 1, 10);

		if(ret != HAL_OK){
			return ret;
		}

		while(HAL_SPI_GetState(spi) == HAL_SPI_STATE_BUSY);
	}

	HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

	uint16_t temp;
	temp = bufferRX[0] << 8;
	temp |= bufferRX[1];
	temp = temp >> 4;

	*retVal = temp;

	return ret;
}

uint32_t ADS7843_IsPressed(void){
	return (HAL_GPIO_ReadPin(IRQ_Port, IRQ_Pin) == GPIO_PIN_RESET);
}


/**
 * \brief Send a command to the ADS7843 touch controller.
 *  @retval 1 if calibrated succesfuly
 */
uint8_t ADS7843_Calibrate(void){
	 /* BSP_LCD_Clear(LCD_COLOR_WHITE);

	  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

	  uint8_t text[30] = "Touch red circle to Calibrate";
	  uint8_t textWidth = BSP_LCD_GetFont()->Width * 15;
	  BSP_LCD_DisplayStringAt((width / 2) - (textWidth / 2), height / 2, text, LEFT_MODE);

	  calibX[0] = width >> 2;
	  calibY[0] = height >> 2;

	  calibX[1] = calibX[0] * 3;
	  calibY[1] = calibY[0] * 3;



	  for(uint8_t i = 0; i < 2; i++){

		  BSP_LCD_SetTextColor(LCD_COLOR_RED);
		  BSP_LCD_DrawCircle(calibX[i], calibY[i], 10);

		  while(1){
			  if(ADS7843_IsPressed()){
				  ADS7843_GetRawPoint((uint16_t *)&calibX[i+2], (uint16_t *)&calibY[i+2]);
				  break;
			  }
		  }

		  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		  BSP_LCD_DrawCircle(calibX[i], calibY[i], 10);

		  uint32_t timeout;
		  uint32_t start = HAL_GetTick();
		  while(1){
			  timeout = HAL_GetTick();

			  if(ADS7843_IsPressed()){
				  start = HAL_GetTick();
			  }

			  if((start - timeout) > 750){
				  break;
			  }
		  }
	  }

	  if(calibY[2] > calibY[3]){
		  invY = 1;
	  }

	  if(calibY[2] > calibY[3]){
		  invX = 1;
	  }


	  calibY[0] = calibY[1] - calibY[0];	// middle of screen
	  calibX[0] = calibX[1] - calibX[0];	// middle of screen



	  if(invY){
		  calibY[1] = calibY[3] - calibY[2];	// middle of touch point
	  }else{
		  calibY[1] = calibY[2] - calibY[3];	// middle of touch point
	  }

	  if(invX){
		  calibX[1] = calibX[3] - calibX[2];	// middle of touch point
	  }else{
		  calibX[1] = calibX[2] - calibX[3];	// middle of touch point
	  }



	  calibY[3] = calibY[1] / calibY[0];	// value per pixel
	  calibX[3] = calibX[1] / calibX[0];	// value per pixel

	  if(calibY[3] != 0 && calibX[3] != 0){
		  calibrated = 1;
	  }

	  return calibrated;*/
	return 1;
}

void ADS7843_GetRawPoint(uint16_t *pointX, uint16_t *pointY)
{
	/** Get X position */
	if(swapXY){
		ADS7843_SendCmd(CMD_X_POSITION, pointY);
	}else{
		ADS7843_SendCmd(CMD_X_POSITION, pointX);
	}

	/** Get Y position */
	if(swapXY){
		ADS7843_SendCmd(CMD_Y_POSITION, pointX);
	}else{
		ADS7843_SendCmd(CMD_Y_POSITION, pointY);
	}

	uint16_t trowAway;
	/** Switch to full power mode */
	ADS7843_SendCmd(CMD_ENABLE_PENIRQ, &trowAway);

}

void ADS7843_GetXYCords(uint16_t *cordX, uint16_t *cordY)
{
	int16_t x = 0;
	int16_t y = 0;

	if(calibrated){
		ADS7843_GetRawPoint((uint16_t *)&x, (uint16_t *)&y);

		if(calibX[1] > calibX[0]){
			x -= calibX[0];
		}else{
			x -= calibX[1];
		}
		x /= calibX[2];

		if(invX){
			x = width - x;
		}


		if(calibY[1] > calibY[0]){
			y -= calibY[0];
		}else{
			y -= calibY[1];
		}
		y /= calibY[2];

		if(invY){
			y = height - y;
		}

		x += xOffset;
		y += yOffset;

		if(x < 0){
			x = 0;
		}else if(x > width){
			x = width;
		}

		if(y < 0){
			y = 0;
		}else if(y > height){
			y = height;
		}

	}

	*cordX = x;
	*cordY = y;

}

uint32_t ADS7843_Init(SPI_HandleTypeDef *spiPointer, uint16_t dispWidth, uint16_t dispHeight){
	uint16_t trowAway;
	uint32_t ret;

	calibrated = 1;

	//width = dispWidth;
	//height = dispHeight;

	if(calibX[1] > calibX[0]){
		calibX[2] = (calibX[1] - calibX[0]) / width;
	}else{
		calibX[2] = (calibX[0] - calibX[1]) / width;
	}

	if(calibY[1] > calibY[0]){
		calibY[2] = (calibY[1] - calibY[0]) / height;
	}else{
		calibY[2] = (calibY[0] - calibY[1]) / height;
	}



	spi = spiPointer;
	ret = ADS7843_SendCmd(CMD_ENABLE_PENIRQ, &trowAway);

	return ret;
}
