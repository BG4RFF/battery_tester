/**
  ******************************************************************************
  * @file    touch_tsc2046.c
  * @author  Krasutski Denis (Krasutski.Denis@gmail.com)
  * @version V1.0.0
  * @date    05-January-2015
  * @brief   This file contains the defines for work with tsc2046
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT 2015 </center></h2>
  ******************************************************************************
  */
#include "touch_tsc2046.h"
#include "bsp.h"

/* Kernel includes. */
//#include "FreeRTOS.h"
//#include "task.h"

static uint32_t _local_x, _local_y;

#define TSC2046_SAMPLE_COUNT	10
static uint16_t buffer_x[TSC2046_SAMPLE_COUNT];
static uint16_t buffer_y[TSC2046_SAMPLE_COUNT];

static void _cs_hi()
{
	bsp_cs_touch_set_high();
}
static void _cs_low()
{
	bsp_cs_touch_set_low();
}

static void _delay(uint32_t ms)
{
		 bsp_delay_ms(ms);
		//vTaskDelay(ms);
}

static uint8_t _wr_rd(uint8_t wr)
{
    uint8_t rd = 0;

    bsp_touch_wr_rd(&wr, &rd,sizeof(rd));

	return rd;
}


static uint16_t _read_x(void)
{
	uint16_t x = 0;

	/* Select Chip */
	_cs_low();
	_delay(1);
	_wr_rd(TSC2046_REG_START_BIT | TSC2046_REG_CHAN1_BIT);

	_delay(1);
	x = _wr_rd(TSC2046_REG_READ_DATA) << 8;
	x |= _wr_rd(TSC2046_REG_READ_DATA);

	/* Release Chip */
	_cs_hi();

    x >>=4;

	return x;
}


static uint16_t _read_y(void)
{
	uint16_t y = 0;

	/* Select Chip */
	_cs_low();
	_delay(1);
	_wr_rd(TSC2046_REG_START_BIT | TSC2046_REG_CHAN5_BIT);

	_delay(1);
	y = _wr_rd(TSC2046_REG_READ_DATA) << 8;
	y += _wr_rd(TSC2046_REG_READ_DATA);

	/* Release Chip */
	_cs_hi();

    y >>= 4;

	return y;
}


static uint32_t _read_once(void)
{
   _local_x = (uint32_t)_read_x();
   _local_y = (uint32_t)_read_y();

	if (
		(_local_x != 0x00) && (_local_x != 0xFFFF) &&
		(_local_y != 0x00) && (_local_y != 0xFFFF) )
	{
		//TSC2046_LOG("X = %04X, Y = %04X\r\n", _localX, _localy);
		return 1;
	}

	return 0;
}

uint32_t _read(void)
{
	uint32_t i, sort_ready, result = 0;

	uint16_t temp=0;

	for(i=0; i < TSC2046_SAMPLE_COUNT; i++)
	{
		if(_read_once())
		{
			buffer_x[i]=_local_x;
			buffer_y[i]=_local_y;
		}
		else
		{
			TSC2046_LOG("TSC2046:Error read data\r\n");
			break;
		}
	}

	if(i == TSC2046_SAMPLE_COUNT)
	{
		do
		{
			sort_ready = 1;
			for(i = 0; i<TSC2046_SAMPLE_COUNT - 1; i++)
			{
				if(buffer_x[i] > buffer_x[i+1])
				{
					temp = buffer_x[i+1];
					buffer_x[i+1] = buffer_x[i];
					buffer_x[i] = temp;
					sort_ready = 0;
				}
			}
		}while(!sort_ready);

		do
		{
			sort_ready = 1;
			for(i=0; i<TSC2046_SAMPLE_COUNT - 1; i++)
			{
				if(buffer_y[i] > buffer_y[i+1])
				{
					temp = buffer_y[i+1];
					buffer_y[i+1] = buffer_y[i];
					buffer_y[i] = temp;
					sort_ready = 0;
				}
			}
		}while(!sort_ready);

		_local_x=(uint32_t)((uint32_t)buffer_x[4]+ (uint32_t)buffer_x[5] +
						(uint32_t)buffer_x[6] + (uint32_t)buffer_x[7])>>2;
		_local_y=(uint32_t)((uint32_t)buffer_y[4]+ (uint32_t)buffer_y[5] +
						(uint32_t)buffer_y[6] + (uint32_t)buffer_y[7])>>2;

		TSC2046_LOG("X = %04X, Y = %04X\r\n", _local_x, _local_y);

		result =  1;
	}

	return result;
}

uint32_t Touch_GetPos(uint32_t *x, uint32_t *y)
{
	uint32_t res;

	res = _read();
	if (res)
	{
		*x = _local_x;
		*y = _local_y;
	}
	return res;
}

