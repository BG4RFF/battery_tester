//#include "sys.h"
#include "mmc_sd.h"
#include "bsp.h"
uint8_t SD_Type = 0;//SD卡的类型
//Mini STM32开发板

//2010/5/13
//增加了一些延时,实测可以支持TF卡(1G/2G),金士顿2G,4G 16G SD卡
//2010/6/24
//加入了uint8_t SD_GetResponse(uint8_t Response)函数
//修改了uint8_t SD_WaitDataReady(void)函数
//增加了USB读卡器支持的uint8_t MSD_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
//和uint8_t MSD_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite);两个函数

void SD_CS(uint32_t en){
    if( en ) {
        bsp_cs_sd_set_high();
    } else {
        bsp_cs_sd_set_low();
    }
}

uint8_t SPIx_ReadWriteByte(uint8_t tx) {
    uint8_t rx = 0;

    bsp_sd_wr_rd(&tx, &rx, sizeof(tx));

    return rx;
}

void SPIx_Init(void) {
    //init after start in BSP init
}

typedef enum SPIxSpeed_e {
    SPIxSpeed_Low,
    SPIxSpeed_High,
}SPIxSpeed_t;

void SPIx_SetSpeed(SPIxSpeed_t speed) {
    if(speed == SPIxSpeed_Low) {

    } else {

    }
}

//等待SD卡回应
//Response:要得到的回应值
//返回值:0,成功得到了该回应值
//    其他,得到回应值失败
uint8_t SD_GetResponse(uint8_t Response) {
	uint16_t Count = 0xFFF;//等待次数
	while ((SPIx_ReadWriteByte(0XFF) != Response) && Count)
		Count--;//等待得到准确的回应
	if (Count == 0)
		return MSD_RESPONSE_FAILURE;//得到回应失败
	else
		return MSD_RESPONSE_NO_ERROR;//正确回应
}
//等待SD卡写入完成
//返回值:0,成功;
//    其他,错误代码;
uint8_t SD_WaitDataReady(void) {
	uint8_t r1 = MSD_DATA_OTHER_ERROR;
	uint32_t retry;
	retry = 0;
	do {
		r1 = SPIx_ReadWriteByte(0xFF) & 0X1F;//读到回应
		if (retry == 0xfffe)
			return 1;
		retry++;
		switch (r1) {
		case MSD_DATA_OK://数据接收正确了
			r1 = MSD_DATA_OK;
			break;
		case MSD_DATA_CRC_ERROR: //CRC校验错误
			return MSD_DATA_CRC_ERROR;
		case MSD_DATA_WRITE_ERROR://数据写入错误
			return MSD_DATA_WRITE_ERROR;
		default://未知错误
			r1 = MSD_DATA_OTHER_ERROR;
			break;
		}
	} while (r1 == MSD_DATA_OTHER_ERROR); //数据错误时一直等待
	retry = 0;
	while (SPIx_ReadWriteByte(0XFF) == 0)//读到数据为0,则数据还未写完成
	{
		retry++;
		//delay_us(10);//SD卡写等待需要较长的时间
		if (retry >= 0XFFFFFFFE)
			return 0XFF;//等待失败了
	};
	return 0;//成功了
}
//向SD卡发送一个命令
//输入: uint8_t cmd   命令
//      uint32_t arg  命令参数
//      uint8_t crc   crc校验值
//返回值:SD卡返回的响应
uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
	uint8_t r1;
	uint8_t Retry = 0;
	SD_CS(1);
	SPIx_ReadWriteByte(0xff);//高速写命令延时
	SPIx_ReadWriteByte(0xff);
	SPIx_ReadWriteByte(0xff);
	//片选端置低，选中SD卡
	SD_CS(0);
	//发送
	SPIx_ReadWriteByte(cmd | 0x40);//分别写入命令
	SPIx_ReadWriteByte(arg >> 24);
	SPIx_ReadWriteByte(arg >> 16);
	SPIx_ReadWriteByte(arg >> 8);
	SPIx_ReadWriteByte(arg);
	SPIx_ReadWriteByte(crc);
	//等待响应，或超时退出
	while ((r1 = SPIx_ReadWriteByte(0xFF)) == 0xFF) {
		Retry++;
		if (Retry > 200)
			break;
	}
	//关闭片选
	SD_CS(1);
	//在总线上额外增加8个时钟，让SD卡完成剩下的工作
	SPIx_ReadWriteByte(0xFF);
	//返回状态值
	return r1;
}
//向SD卡发送一个命令(结束是不失能片选，还有后续数据传来）
//输入:uint8_t cmd   命令
//     uint32_t arg  命令参数
//     uint8_t crc   crc校验值
//返回值:SD卡返回的响应
uint8_t SD_SendCommand_NoDeassert(uint8_t cmd, uint32_t arg, uint8_t crc) {
	uint8_t Retry = 0;
	uint8_t r1;
	SPIx_ReadWriteByte(0xff);//高速写命令延时
	SPIx_ReadWriteByte(0xff);
	SD_CS(0);//片选端置低，选中SD卡
	//发送
	SPIx_ReadWriteByte(cmd | 0x40); //分别写入命令
	SPIx_ReadWriteByte(arg >> 24);
	SPIx_ReadWriteByte(arg >> 16);
	SPIx_ReadWriteByte(arg >> 8);
	SPIx_ReadWriteByte(arg);
	SPIx_ReadWriteByte(crc);
	//等待响应，或超时退出
	while ((r1 = SPIx_ReadWriteByte(0xFF)) == 0xFF) {
		Retry++;
		if (Retry > 200)
			break;
	}
	//返回响应值
	return r1;
}


uint8_t SD_Idle_Sta(void) {
	uint16_t i;
	uint8_t retry;
	for (i = 0; i < 0xf00; i++)
		;
	for (i = 0; i < 10; i++)
		SPIx_ReadWriteByte(0xFF);
	retry = 0;
	do {
		i = SD_SendCommand(CMD0, 0, 0x95);
		retry++;
	} while ((i != 0x01) && (retry < 200));
	if (retry == 200)
		return 1;
	return 0;
}

//Return: 0 NO_ERR
//        1 TIME_OUT
//       99 NO_CARD
uint8_t SD_Init(void) {
	uint8_t r1;
	uint16_t retry;
	uint8_t buff[6];


	SPIx_Init();
	SPIx_SetSpeed(SPIxSpeed_Low);
	SD_CS(1);
	if (SD_Idle_Sta())
		return 1;

	SD_CS(0);
	r1 = SD_SendCommand_NoDeassert(8, 0x1aa, 0x87);

	if (r1 == 0x05) {
		SD_Type = SD_TYPE_V1;
		SD_CS(1);
		SPIx_ReadWriteByte(0xFF);
		retry = 0;
		do {
			r1 = SD_SendCommand(CMD55, 0, 0);
			if (r1 == 0XFF)
				return r1;
			r1 = SD_SendCommand(ACMD41, 0, 0);
			retry++;
		} while ((r1 != 0x00) && (retry < 400));
		if (retry == 400) {
			retry = 0;
			do {
				r1 = SD_SendCommand(1, 0, 0);
				retry++;
			} while ((r1 != 0x00) && (retry < 400));
			if (retry == 400)
				return 1;
			SD_Type = SD_TYPE_MMC;
		}
		SPIx_SetSpeed(SPIxSpeed_High);
		SPIx_ReadWriteByte(0xFF);
		r1 = SD_SendCommand(CMD59, 0, 0x95);
		if (r1 != 0x00)
			return r1;
		r1 = SD_SendCommand(CMD16, 512, 0x95);
		if (r1 != 0x00)
			return r1;
	}
	else if (r1 == 0x01) {
		buff[0] = SPIx_ReadWriteByte(0xFF); //should be 0x00
		buff[1] = SPIx_ReadWriteByte(0xFF); //should be 0x00
		buff[2] = SPIx_ReadWriteByte(0xFF); //should be 0x01
		buff[3] = SPIx_ReadWriteByte(0xFF); //should be 0xAA
		SD_CS(1);
		SPIx_ReadWriteByte(0xFF);//the next 8 clocks
		//if(buff[2]==0x01 && buff[3]==0xAA)
		{
			retry = 0;
			do {
				r1 = SD_SendCommand(CMD55, 0, 0);
				if (r1 != 0x01)
					return r1;
				r1 = SD_SendCommand(ACMD41, 0x40000000, 0);
				if (retry > 200)
					return r1;
			} while (r1 != 0);
			r1 = SD_SendCommand_NoDeassert(CMD58, 0, 0);
			if (r1 != 0x00) {
				SD_CS(1);
				return r1;
			}
			buff[0] = SPIx_ReadWriteByte(0xFF);
			buff[1] = SPIx_ReadWriteByte(0xFF);
			buff[2] = SPIx_ReadWriteByte(0xFF);
			buff[3] = SPIx_ReadWriteByte(0xFF);
			SD_CS(1);
			SPIx_ReadWriteByte(0xFF);
			if (buff[0] & 0x40)
				SD_Type = SD_TYPE_V2HC;
			else
				SD_Type = SD_TYPE_V2;
			SPIx_SetSpeed(SPIxSpeed_High);
		}
	}
	return r1;
}


//从SD卡中读回指定长度的数据，放置在给定位置
//输入: uint8_t *data(存放读回数据的内存>len)
//      uint16_t len(数据长度）
//      uint8_t release(传输完成后是否释放总线CS置高 0：不释放 1：释放）
//返回值:0：NO_ERR
//  	 other：错误信息
uint8_t SD_ReceiveData(uint8_t *data, uint16_t len, uint8_t release) {
	SD_CS(0);
	if (SD_GetResponse(0xFE))
	{
		SD_CS(1);
		return 1;
	}
	while (len--)
	{
		*data = SPIx_ReadWriteByte(0xFF);
		data++;
	}

	SPIx_ReadWriteByte(0xFF);
	SPIx_ReadWriteByte(0xFF);
	if (release == RELEASE)
	{
		SD_CS(1);
		SPIx_ReadWriteByte(0xFF);
	}
	return 0;
}

//输入: uint8_t *cid_data(存放CID的内存，至少16Byte）
//返回值:0：NO_ERR
//		 1：TIME_OUT
//       other：错误信息
uint8_t SD_GetCID(uint8_t *cid_data) {
	uint8_t r1;
	//发CMD10命令，读CID
	r1 = SD_SendCommand(CMD10, 0, 0xFF);
	if (r1 != 0x00)
		return r1; //没返回正确应答，则退出，报错
	SD_ReceiveData(cid_data, 16, RELEASE);//接收16个字节的数据
	return 0;
}
//获取SD卡的CSD信息，包括容量和速度信息
//输入:uint8_t *cid_data(存放CID的内存，至少16Byte）
//返回值:0：NO_ERR
//       1：TIME_OUT
//       other：错误信息
uint8_t SD_GetCSD(uint8_t *csd_data) {
	uint8_t r1;
	r1 = SD_SendCommand(CMD9, 0, 0xFF);//发CMD9命令，读CSD
	if (r1)
		return r1; //没返回正确应答，则退出，报错
	SD_ReceiveData(csd_data, 16, RELEASE);//接收16个字节的数据
	return 0;
}


uint32_t SD_GetCapacity(void) {
	uint8_t csd[16];
	uint32_t Capacity;
	uint8_t r1;
	uint16_t i;
	uint16_t temp;

	if (SD_GetCSD(csd) != 0)
		return 0;

	if ((csd[0] & 0xC0) == 0x40) {
		Capacity = ((uint32_t) csd[8]) << 8;
		Capacity += (uint32_t) csd[9] + 1;
		Capacity = (Capacity) * 1024;
		Capacity *= 512;
	} else {
		i = csd[6] & 0x03;
		i <<= 8;
		i += csd[7];
		i <<= 2;
		i += ((csd[8] & 0xc0) >> 6);

		r1 = csd[9] & 0x03;
		r1 <<= 1;
		r1 += ((csd[10] & 0x80) >> 7);
		r1 += 2;//BLOCKNR
		temp = 1;
		while (r1) {
			temp *= 2;
			r1--;
		}
		Capacity = ((uint32_t) (i + 1)) * ((uint32_t) temp);
		// READ_BL_LEN
		i = csd[5] & 0x0f;
		//BLOCK_LEN
		temp = 1;
		while (i) {
			temp *= 2;
			i--;
		}
		//The final result
		Capacity *= (uint32_t) temp;
	}
	return (uint32_t) Capacity;
}
//读SD卡的一个block
//输入:uint32_t sector 取地址（sector值，非物理地址）
//     uint8_t *buffer 数据存储地址（大小至少512byte）
//返回值:0： 成功
//       other：失败
uint8_t SD_ReadSingleBlock(uint32_t sector, uint8_t *buffer) {
	uint8_t r1;
	//设置为高速模式
	SPIx_SetSpeed(SPIxSpeed_High);
	//如果不是SDHC，给定的是sector地址，将其转换成byte地址
	if (SD_Type != SD_TYPE_V2HC) {
		sector = sector << 9;
	}
	r1 = SD_SendCommand(CMD17, sector, 0);//读命令
	if (r1 != 0x00)
		return r1;
	r1 = SD_ReceiveData(buffer, 512, RELEASE);
	if (r1 != 0)
		return r1; //读数据出错！
	else
		return 0;
}
////////////////////////////下面2个函数为USB读写所需要的/////////////////////////
//定义SD卡的块大小
#define BLOCK_SIZE 512
//写入MSD/SD数据
//pBuffer:数据存放区
//ReadAddr:写入的首地址
//NumByteToRead:要写入的字节数
//返回值:0,写入完成
//    其他,写入失败
uint8_t MSD_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite) {
	uint32_t i, NbrOfBlock = 0, Offset = 0;
	uint32_t sector;
	uint8_t r1;
	NbrOfBlock = NumByteToWrite / BLOCK_SIZE;//得到要写入的块的数目
	SD_CS(0);
	while (NbrOfBlock--)//写入一个扇区
	{
		sector = WriteAddr + Offset;
		if (SD_Type == SD_TYPE_V2HC)
			sector >>= 9;//执行与普通操作相反的操作
		r1 = SD_SendCommand_NoDeassert(CMD24, sector, 0xff);//写命令
		if (r1) {
			SD_CS(1);
			return 1;//应答不正确，直接返回
		}
		SPIx_ReadWriteByte(0xFE);//放起始令牌0xFE
		//放一个sector的数据
		for (i = 0; i < 512; i++)
			SPIx_ReadWriteByte(*pBuffer++);
		//发2个Byte的dummy CRC
		SPIx_ReadWriteByte(0xff);
		SPIx_ReadWriteByte(0xff);
		if (SD_WaitDataReady())//等待SD卡数据写入完成
		{
			SD_CS(1);
			return 2;
		}
		Offset += 512;
	}
	//写入完成，片选置1
	SD_CS(1);
	SPIx_ReadWriteByte(0xff);
	return 0;
}
//读取MSD/SD数据
//pBuffer:数据存放区
//ReadAddr:读取的首地址
//NumByteToRead:要读出的字节数
//返回值:0,读出完成
//    其他,读出失败
uint8_t MSD_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead) {
	uint32_t NbrOfBlock = 0, Offset = 0;
	uint32_t sector = 0;
	uint8_t r1 = 0;
	NbrOfBlock = NumByteToRead / BLOCK_SIZE;
	SD_CS(0);
	while (NbrOfBlock--) {
		sector = ReadAddr + Offset;
		if (SD_Type == SD_TYPE_V2HC)
			sector >>= 9;//执行与普通操作相反的操作
		r1 = SD_SendCommand_NoDeassert(CMD17, sector, 0xff);//读命令
		if (r1)//命令发送错误
		{
			SD_CS(1);
			return r1;
		}
		r1 = SD_ReceiveData(pBuffer, 512, RELEASE);
		if (r1)//读数错误
		{
			SD_CS(1);
			return r1;
		}
		pBuffer += 512;
		Offset += 512;
	}
	SD_CS(1);
	SPIx_ReadWriteByte(0xff);
	return 0;
}
//////////////////////////////////////////////////////////////////////////
//写入SD卡的一个block(未实际测试过)
//输入:uint32_t sector 扇区地址（sector值，非物理地址）
//     uint8_t *buffer 数据存储地址（大小至少512byte）
//返回值:0： 成功
//       other：失败
uint8_t SD_WriteSingleBlock(uint32_t sector, const uint8_t *data) {
	uint8_t r1;
	uint16_t i;
	uint16_t retry;

	//设置为高速模式
	//SPIx_SetSpeed(SPI_SPEED_HIGH);
	//如果不是SDHC，给定的是sector地址，将其转换成byte地址
	if (SD_Type != SD_TYPE_V2HC) {
		sector = sector << 9;
	}
	r1 = SD_SendCommand(CMD24, sector, 0x00);
	if (r1 != 0x00) {
		return r1; //应答不正确，直接返回
	}

	//开始准备数据传输
	SD_CS(0);
	//先放3个空数据，等待SD卡准备好
	SPIx_ReadWriteByte(0xff);
	SPIx_ReadWriteByte(0xff);
	SPIx_ReadWriteByte(0xff);
	//放起始令牌0xFE
	SPIx_ReadWriteByte(0xFE);

	//放一个sector的数据
	for (i = 0; i < 512; i++) {
		SPIx_ReadWriteByte(*data++);
	}
	//发2个Byte的dummy CRC
	SPIx_ReadWriteByte(0xff);
	SPIx_ReadWriteByte(0xff);

	//等待SD卡应答
	r1 = SPIx_ReadWriteByte(0xff);
	if ((r1 & 0x1F) != 0x05) {
		SD_CS(1);
		return r1;
	}

	//等待操作完成
	retry = 0;
	while (!SPIx_ReadWriteByte(0xff)) {
		retry++;
		if (retry > 0xfffe) //如果长时间写入没有完成，报错退出
		{
			SD_CS(1);
			return 1; //写入超时返回1
		}
	}
	//写入完成，片选置1
	SD_CS(1);
	SPIx_ReadWriteByte(0xff);

	return 0;
}
//读SD卡的多个block(实际测试过)
//输入:uint32_t sector 扇区地址（sector值，非物理地址）
//     uint8_t *buffer 数据存储地址（大小至少512byte）
//     uint8_t count 连续读count个block
//返回值:0： 成功
//       other：失败
uint8_t SD_ReadMultiBlock(uint32_t sector, uint8_t *buffer, uint8_t count) {
	uint8_t r1;
	//SPIx_SetSpeed(SPI_SPEED_HIGH);//设置为高速模式
	//如果不是SDHC，将sector地址转成byte地址
	if (SD_Type != SD_TYPE_V2HC)
		sector = sector << 9;
	//SD_WaitDataReady();
	//发读多块命令
	r1 = SD_SendCommand(CMD18, sector, 0);//读命令
	if (r1 != 0x00)
		return r1;
	do//开始接收数据
	{
		if (SD_ReceiveData(buffer, 512, NO_RELEASE) != 0x00)
			break;
		buffer += 512;
	} while (--count);
	//全部传输完毕，发送停止命令
	SD_SendCommand(CMD12, 0, 0);
	//释放总线
	SD_CS(1);
	SPIx_ReadWriteByte(0xFF);
	if (count != 0)
		return count; //如果没有传完，返回剩余个数
	else
		return 0;
}
//写入SD卡的N个block(未实际测试过)
//输入:uint32_t sector 扇区地址（sector值，非物理地址）
//     uint8_t *buffer 数据存储地址（大小至少512byte）
//     uint8_t count 写入的block数目
//返回值:0： 成功
//       other：失败
uint8_t SD_WriteMultiBlock(uint32_t sector, const uint8_t *data, uint8_t count) {
	uint8_t r1;
	uint16_t i;
	//SPIx_SetSpeed(SPI_SPEED_HIGH);//设置为高速模式
	if (SD_Type != SD_TYPE_V2HC)
		sector = sector << 9;//如果不是SDHC，给定的是sector地址，将其转换成byte地址
	if (SD_Type != SD_TYPE_MMC)
		r1 = SD_SendCommand(ACMD23, count, 0x00);//如果目标卡不是MMC卡，启用ACMD23指令使能预擦除
	r1 = SD_SendCommand(CMD25, sector, 0x00);//发多块写入指令
	if (r1 != 0x00)
		return r1; //应答不正确，直接返回
	SD_CS(0);//开始准备数据传输
	SPIx_ReadWriteByte(0xff);//先放3个空数据，等待SD卡准备好
	SPIx_ReadWriteByte(0xff);
	//--------下面是N个sector写入的循环部分
	do {
		//放起始令牌0xFC 表明是多块写入
		SPIx_ReadWriteByte(0xFC);
		//放一个sector的数据
		for (i = 0; i < 512; i++) {
			SPIx_ReadWriteByte(*data++);
		}
		//发2个Byte的dummy CRC
		SPIx_ReadWriteByte(0xff);
		SPIx_ReadWriteByte(0xff);

		//等待SD卡应答
		r1 = SPIx_ReadWriteByte(0xff);
		if ((r1 & 0x1F) != 0x05) {
			SD_CS(1); //如果应答为报错，则带错误代码直接退出
			return r1;
		}
		//等待SD卡写入完成
		if (SD_WaitDataReady() == 1) {
			SD_CS(1); //等待SD卡写入完成超时，直接退出报错
			return 1;
		}
	} while (--count);//本sector数据传输完成
	//发结束传输令牌0xFD
	r1 = SPIx_ReadWriteByte(0xFD);
	if (r1 == 0x00) {
		count = 0xfe;
	}
	if (SD_WaitDataReady()) //等待准备好
	{
		SD_CS(1);
		return 1;
	}
	//写入完成，片选置1
	SD_CS(1);
	SPIx_ReadWriteByte(0xff);
	return count; //返回count值，如果写完则count=0，否则count=1
}
//在指定扇区,从offset开始读出bytes个字节
//输入:uint32_t sector 扇区地址（sector值，非物理地址）
//     uint8_t *buf     数据存储地址（大小<=512byte）
//     uint16_t offset  在扇区里面的偏移量
//     uint16_t bytes   要读出的字节数
//返回值:0： 成功
//       other：失败
uint8_t SD_Read_Bytes(unsigned long address, unsigned char *buf,
		unsigned int offset, unsigned int bytes) {
	uint8_t r1;
	uint16_t i = 0;
	r1 = SD_SendCommand(CMD17, address << 9, 0);//发送读扇区命令
	if (r1)
		return r1; //应答不正确，直接返回
	SD_CS(0);//选中SD卡
	if (SD_GetResponse(0xFE))//等待SD卡发回数据起始令牌0xFE
	{
		SD_CS(1); //关闭SD卡
		return 1;//读取失败
	}
	for (i = 0; i < offset; i++)
		SPIx_ReadWriteByte(0xff);//跳过offset位
	for (; i < offset + bytes; i++)
		*buf++ = SPIx_ReadWriteByte(0xff);//读取有用数据
	for (; i < 512; i++)
		SPIx_ReadWriteByte(0xff); //读出剩余字节
	SPIx_ReadWriteByte(0xff);//发送伪CRC码
	SPIx_ReadWriteByte(0xff);
	SD_CS(1);//关闭SD卡
	return 0;
}

