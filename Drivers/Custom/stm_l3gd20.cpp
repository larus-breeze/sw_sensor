#include "stm_l3gd20.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "my_assert.h"
#include "common.h"

static void L3GD20_LowLevel_Init(void);

COMMON static uint8_t __ALIGNED(4) L3GD20_tmpreg;

void L3GD20_Write( uint8_t datum, uint8_t WriteAddr)
{
	COMMON static uint8_t __ALIGNED(4) buf[2];
	buf[0] = WriteAddr;
	buf[1] = datum;
    L3GD20_CS_LOW();

    SPI_Transmit(&hspi2, buf, sizeof(buf));

    L3GD20_CS_HIGH();
}


/**
 * @brief  Set L3GD20 Initialization.
 * @param  L3GD20_InitStruct: pointer to a L3GD20_InitTypeDef structure
 *         that contains the configuration setting for the L3GD20.
 */
void L3GD20_Init(L3GD20_InitTypeDef *L3GD20_InitStruct)
{  
	uint8_t ctrl1 = 0x00, ctrl4 = 0x00;

	/* Configure the low level interface ---------------------------------------*/
	L3GD20_LowLevel_Init();

	/* Configure MEMS: data rate, power mode, full scale and axes */
	ctrl1 |= (uint8_t) (L3GD20_InitStruct->Power_Mode | L3GD20_InitStruct->Output_DataRate | \
			L3GD20_InitStruct->Axes_Enable | L3GD20_InitStruct->Band_Width);

	ctrl4 |= (uint8_t) (L3GD20_InitStruct->BlockData_Update | L3GD20_InitStruct->Endianness | \
			L3GD20_InitStruct->Full_Scale);

	/* Write value to MEMS CTRL_REG1 register */
	L3GD20_Write( ctrl1, L3GD20_CTRL_REG1_ADDR);

	/* Write value to MEMS CTRL_REG4 register */
	L3GD20_Write( ctrl4, L3GD20_CTRL_REG4_ADDR);
}

/**
 * @brief  Reboot memory content of L3GD20
 */
void L3GD20_RebootCmd(void)
{
	/* Read CTRL_REG5 register */
	L3GD20_Read(&L3GD20_tmpreg, L3GD20_CTRL_REG5_ADDR, 1);

	/* Enable or Disable the reboot memory */
	L3GD20_tmpreg |= L3GD20_BOOT_REBOOTMEMORY;

	/* Write value to MEMS CTRL_REG5 register */
	L3GD20_Write( L3GD20_tmpreg, L3GD20_CTRL_REG5_ADDR);
}

/**
 * @brief Set L3GD20 Interrupt configuration
 * @param  L3GD20_InterruptConfig_TypeDef: pointer to a L3GD20_InterruptConfig_TypeDef
 *         structure that contains the configuration setting for the L3GD20 Interrupt.
 */
void L3GD20_INT1InterruptConfig(L3GD20_InterruptConfigTypeDef *L3GD20_IntConfigStruct)
{
	uint8_t ctrl_cfr = 0x00, ctrl3 = 0x00;

	/* Read INT1_CFG register */
	L3GD20_Read(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);

	/* Read CTRL_REG3 register */
	L3GD20_Read(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);

	ctrl_cfr &= 0x80;

	ctrl3 &= 0xDF;

	/* Configure latch Interrupt request and axe interrupts */
	ctrl_cfr |= (uint8_t)(L3GD20_IntConfigStruct->Latch_Request| \
			L3GD20_IntConfigStruct->Interrupt_Axes);

	ctrl3 |= (uint8_t)(L3GD20_IntConfigStruct->Interrupt_ActiveEdge);

	/* Write value to MEMS INT1_CFG register */
	L3GD20_Write( ctrl_cfr, L3GD20_INT1_CFG_ADDR);

	/* Write value to MEMS CTRL_REG3 register */
	L3GD20_Write( ctrl3, L3GD20_CTRL_REG3_ADDR);
}

/**
 * @brief  Enable or disable INT1 interrupt
 * @param  InterruptState: State of INT1 Interrupt
 *      This parameter can be:
 *        @arg L3GD20_INT1INTERRUPT_DISABLE
 *        @arg L3GD20_INT1INTERRUPT_ENABLE
 */
void L3GD20_INT1InterruptCmd(uint8_t InterruptState)
{  
	/* Read CTRL_REG3 register */
	L3GD20_Read(&L3GD20_tmpreg, L3GD20_CTRL_REG3_ADDR, 1);

	L3GD20_tmpreg &= 0x7F;
	L3GD20_tmpreg |= InterruptState;

	/* Write value to MEMS CTRL_REG3 regsister */
	L3GD20_Write( L3GD20_tmpreg, L3GD20_CTRL_REG3_ADDR);
}

/**
 * @brief  Enable or disable INT2 interrupt
 * @param  InterruptState: State of INT1 Interrupt
 *      This parameter can be:
 *        @arg L3GD20_INT2INTERRUPT_DISABLE
 *        @arg L3GD20_INT2INTERRUPT_ENABLE
 */
void L3GD20_INT2InterruptCmd(uint8_t InterruptState)
{  
	/* Read CTRL_REG3 register */
	L3GD20_Read(&L3GD20_tmpreg, L3GD20_CTRL_REG3_ADDR, 1);

	L3GD20_tmpreg &= 0xF7;
	L3GD20_tmpreg |= InterruptState;

	/* Write value to MEMS CTRL_REG3 regsister */
	L3GD20_Write( L3GD20_tmpreg, L3GD20_CTRL_REG3_ADDR);
}

/**
 * @brief  Set High Pass Filter Modality
 * @param  L3GD20_FilterStruct: pointer to a L3GD20_FilterConfigTypeDef structure
 *         that contains the configuration setting for the L3GD20.
 */
void L3GD20_FilterConfig(L3GD20_FilterConfigTypeDef *L3GD20_FilterStruct) 
{
	/* Read CTRL_REG2 register */
	L3GD20_Read(&L3GD20_tmpreg, L3GD20_CTRL_REG2_ADDR, 1);

	L3GD20_tmpreg &= 0xC0;

	/* Configure MEMS: mode and cutoff frquency */
	L3GD20_tmpreg |= (uint8_t) (L3GD20_FilterStruct->HighPassFilter_Mode_Selection |\
			L3GD20_FilterStruct->HighPassFilter_CutOff_Frequency);

	/* Write value to MEMS CTRL_REG2 regsister */
	L3GD20_Write( L3GD20_tmpreg, L3GD20_CTRL_REG2_ADDR);
}

/**
 * @brief  Enable or Disable High Pass Filter
 * @param  HighPassFilterState: new state of the High Pass Filter feature.
 *      This parameter can be:
 *         @arg: L3GD20_HIGHPASSFILTER_DISABLE
 *         @arg: L3GD20_HIGHPASSFILTER_ENABLE
 */
void L3GD20_FilterCmd(uint8_t HighPassFilterState)
{
	/* Read CTRL_REG5 register */
	L3GD20_Read(&L3GD20_tmpreg, L3GD20_CTRL_REG5_ADDR, 1);

	L3GD20_tmpreg &= 0xEF;

	L3GD20_tmpreg |= HighPassFilterState;

	/* Write value to MEMS CTRL_REG5 regsister */
	L3GD20_Write( L3GD20_tmpreg, L3GD20_CTRL_REG5_ADDR);
}

/**
 * @brief  Get status for L3GD20 data
 * @param  None
 * @retval Data status in a L3GD20 Data
 */
uint8_t L3GD20_GetDataStatus(void)
{
	/* Read STATUS_REG register */
	L3GD20_Read(&L3GD20_tmpreg, L3GD20_STATUS_REG_ADDR, 1);

	return L3GD20_tmpreg;
}

/**
 * @brief  Reads a block of data from the L3GD20.
 * @param  pBuffer : pointer to the buffer that receives the data read from the L3GD20.
 * @param  ReadAddr : L3GD20's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the L3GD20.
 */
void L3GD20_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
	COMMON static uint8_t __ALIGNED(4) buf;   //For SPI DMA Usage

	if(NumByteToRead > 0x01)
	{
		ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
	}
	else
	{
		ReadAddr |= (uint8_t)READWRITE_CMD;
	}
	/* Set chip select Low at the start of the transmission */
	L3GD20_CS_LOW();

	/* Send the Address of the indexed register */
	buf = ReadAddr; // Aligned for SPI via DMA!
	(void)L3GD20_SendByte(buf);

	/* read data */
	SPI_Receive(&hspi2, pBuffer, NumByteToRead);

	/* Set chip select High at the end of the transmission */
	L3GD20_CS_HIGH();
}  
/**
 * @brief  Initializes the low level interface used to drive the L3GD20
 * @param  None
 * @retval None
 */
static void L3GD20_LowLevel_Init(void)
{

}  

#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
/**
 * @brief  Basic management of the timeout situation.
 * @param  None.
 * @retval None.
 */
uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
	/* Block communication and all processes */
	while (1)
	{
	}
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */

/**
 * @brief  Configure the Mems to gyroscope application.
 * @param  None
 * @retval None
 */
bool L3GD20_Initialize(void)
{
	L3GD20_InitTypeDef L3GD20_InitStructure;
	L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;

	/* Configure Mems L3GD20 */
	L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
	L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_4;
	L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
	L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_1;
	L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
	L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_250;

	L3GD20_Init(&L3GD20_InitStructure);

	L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
	L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
	L3GD20_FilterConfig(&L3GD20_FilterStructure) ;

	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_DISABLE);

	L3GD20_Write( L3GD20_FIFO_ENABLE, L3GD20_CTRL_REG5_ADDR);

	L3GD20_Write( L3GD20_FIFO_MODE_STREAM, L3GD20_FIFO_CTRL_REG_ADDR);

	L3GD20_Read(&L3GD20_tmpreg, L3GD20_WHO_AM_I_ADDR, 1);
	if(L3GD20_tmpreg == I_AM_L3GD20)
	{
		return true;  // L3GD20 Sensor recognized
	}
	else
	{
		return false;
	}
}

void L3GD20_ReadData( float data[3])
{
	volatile uint8_t axis, fifoEntry, fifoStatus, fifoFilled;
	int32_t value;
	int32_t all_values;
	COMMON static uint8_t  __ALIGNED(4) sensordata[ 3 * sizeof( int16_t) * L3GD20_MAX_FIFO_ENTRIES]; // buffer for measurement data
	for(;;)
	  {

	    L3GD20_Read( (uint8_t *)&L3GD20_tmpreg,L3GD20_FIFO_SRC_REG_ADDR, 1);
	    fifoStatus = L3GD20_tmpreg;

#if PARANOID_SPI_CHECK == 1
	    ASSERT( (fifoStatus & L3GD20_FIFO_EMPTY) ==0);
	    ASSERT( (fifoStatus & L3GD20_FIFO_OVRN)  ==0);
#endif

	    fifoFilled = (fifoStatus & L3GD20_FIFO_FILLED);

	    if( fifoFilled > 0)
	      break;
	    vTaskDelay(1);
	  }

	L3GD20_Read( sensordata, L3GD20_OUT_X_L_ADDR, 6 * fifoFilled);

	for( axis=0; axis < 3; ++axis)
	{
		all_values=0;
		for( fifoEntry = 0; fifoEntry < fifoFilled; ++fifoEntry)
		{
			value = ( sensordata[(axis << 1) + 1 + fifoEntry * 6] << 8) + sensordata[(axis << 1) + fifoEntry * 6];
			if(( value & 0x8000) !=0)
				value |= 0xffff0000;
			all_values += value;
		}
		all_values /= fifoFilled;
		ASSERT( all_values <= 32767 && all_values >=-32768);
		switch( axis)
		{
		case 0:
			data[0] = -all_values;
		break;
		case 1:
			data[1] = -all_values;
		break;
		case 2:
			data[2] = +all_values;
		break;
		}
	}
}
