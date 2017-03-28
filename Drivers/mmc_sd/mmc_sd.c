//#include "sys.h"
#include "mmc_sd.h"
#include "bsp.h"
uint8_t SD_Type = 0;//SD��������
//Mini STM32������

//2010/5/13
//������һЩ��ʱ,ʵ�����֧��TF��(1G/2G),��ʿ��2G,4G 16G SD��
//2010/6/24
//������uint8_t SD_GetResponse(uint8_t Response)����
//�޸���uint8_t SD_WaitDataReady(void)����
//������USB������֧�ֵ�uint8_t MSD_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
//��uint8_t MSD_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite);��������

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

//�ȴ�SD����Ӧ
//Response:Ҫ�õ��Ļ�Ӧֵ
//����ֵ:0,�ɹ��õ��˸û�Ӧֵ
//    ����,�õ���Ӧֵʧ��
uint8_t SD_GetResponse(uint8_t Response) {
	uint16_t Count = 0xFFF;//�ȴ�����
	while ((SPIx_ReadWriteByte(0XFF) != Response) && Count)
		Count--;//�ȴ��õ�׼ȷ�Ļ�Ӧ
	if (Count == 0)
		return MSD_RESPONSE_FAILURE;//�õ���Ӧʧ��
	else
		return MSD_RESPONSE_NO_ERROR;//��ȷ��Ӧ
}
//�ȴ�SD��д�����
//����ֵ:0,�ɹ�;
//    ����,�������;
uint8_t SD_WaitDataReady(void) {
	uint8_t r1 = MSD_DATA_OTHER_ERROR;
	uint32_t retry;
	retry = 0;
	do {
		r1 = SPIx_ReadWriteByte(0xFF) & 0X1F;//������Ӧ
		if (retry == 0xfffe)
			return 1;
		retry++;
		switch (r1) {
		case MSD_DATA_OK://���ݽ�����ȷ��
			r1 = MSD_DATA_OK;
			break;
		case MSD_DATA_CRC_ERROR: //CRCУ�����
			return MSD_DATA_CRC_ERROR;
		case MSD_DATA_WRITE_ERROR://����д�����
			return MSD_DATA_WRITE_ERROR;
		default://δ֪����
			r1 = MSD_DATA_OTHER_ERROR;
			break;
		}
	} while (r1 == MSD_DATA_OTHER_ERROR); //���ݴ���ʱһֱ�ȴ�
	retry = 0;
	while (SPIx_ReadWriteByte(0XFF) == 0)//��������Ϊ0,�����ݻ�δд���
	{
		retry++;
		//delay_us(10);//SD��д�ȴ���Ҫ�ϳ���ʱ��
		if (retry >= 0XFFFFFFFE)
			return 0XFF;//�ȴ�ʧ����
	};
	return 0;//�ɹ���
}
//��SD������һ������
//����: uint8_t cmd   ����
//      uint32_t arg  �������
//      uint8_t crc   crcУ��ֵ
//����ֵ:SD�����ص���Ӧ
uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
	uint8_t r1;
	uint8_t Retry = 0;
	SD_CS(1);
	SPIx_ReadWriteByte(0xff);//����д������ʱ
	SPIx_ReadWriteByte(0xff);
	SPIx_ReadWriteByte(0xff);
	//Ƭѡ���õͣ�ѡ��SD��
	SD_CS(0);
	//����
	SPIx_ReadWriteByte(cmd | 0x40);//�ֱ�д������
	SPIx_ReadWriteByte(arg >> 24);
	SPIx_ReadWriteByte(arg >> 16);
	SPIx_ReadWriteByte(arg >> 8);
	SPIx_ReadWriteByte(arg);
	SPIx_ReadWriteByte(crc);
	//�ȴ���Ӧ����ʱ�˳�
	while ((r1 = SPIx_ReadWriteByte(0xFF)) == 0xFF) {
		Retry++;
		if (Retry > 200)
			break;
	}
	//�ر�Ƭѡ
	SD_CS(1);
	//�������϶�������8��ʱ�ӣ���SD�����ʣ�µĹ���
	SPIx_ReadWriteByte(0xFF);
	//����״ֵ̬
	return r1;
}
//��SD������һ������(�����ǲ�ʧ��Ƭѡ�����к������ݴ�����
//����:uint8_t cmd   ����
//     uint32_t arg  �������
//     uint8_t crc   crcУ��ֵ
//����ֵ:SD�����ص���Ӧ
uint8_t SD_SendCommand_NoDeassert(uint8_t cmd, uint32_t arg, uint8_t crc) {
	uint8_t Retry = 0;
	uint8_t r1;
	SPIx_ReadWriteByte(0xff);//����д������ʱ
	SPIx_ReadWriteByte(0xff);
	SD_CS(0);//Ƭѡ���õͣ�ѡ��SD��
	//����
	SPIx_ReadWriteByte(cmd | 0x40); //�ֱ�д������
	SPIx_ReadWriteByte(arg >> 24);
	SPIx_ReadWriteByte(arg >> 16);
	SPIx_ReadWriteByte(arg >> 8);
	SPIx_ReadWriteByte(arg);
	SPIx_ReadWriteByte(crc);
	//�ȴ���Ӧ����ʱ�˳�
	while ((r1 = SPIx_ReadWriteByte(0xFF)) == 0xFF) {
		Retry++;
		if (Retry > 200)
			break;
	}
	//������Ӧֵ
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


//��SD���ж���ָ�����ȵ����ݣ������ڸ���λ��
//����: uint8_t *data(��Ŷ������ݵ��ڴ�>len)
//      uint16_t len(���ݳ��ȣ�
//      uint8_t release(������ɺ��Ƿ��ͷ�����CS�ø� 0�����ͷ� 1���ͷţ�
//����ֵ:0��NO_ERR
//  	 other��������Ϣ
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

//����: uint8_t *cid_data(���CID���ڴ棬����16Byte��
//����ֵ:0��NO_ERR
//		 1��TIME_OUT
//       other��������Ϣ
uint8_t SD_GetCID(uint8_t *cid_data) {
	uint8_t r1;
	//��CMD10�����CID
	r1 = SD_SendCommand(CMD10, 0, 0xFF);
	if (r1 != 0x00)
		return r1; //û������ȷӦ�����˳�������
	SD_ReceiveData(cid_data, 16, RELEASE);//����16���ֽڵ�����
	return 0;
}
//��ȡSD����CSD��Ϣ�������������ٶ���Ϣ
//����:uint8_t *cid_data(���CID���ڴ棬����16Byte��
//����ֵ:0��NO_ERR
//       1��TIME_OUT
//       other��������Ϣ
uint8_t SD_GetCSD(uint8_t *csd_data) {
	uint8_t r1;
	r1 = SD_SendCommand(CMD9, 0, 0xFF);//��CMD9�����CSD
	if (r1)
		return r1; //û������ȷӦ�����˳�������
	SD_ReceiveData(csd_data, 16, RELEASE);//����16���ֽڵ�����
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
//��SD����һ��block
//����:uint32_t sector ȡ��ַ��sectorֵ���������ַ��
//     uint8_t *buffer ���ݴ洢��ַ����С����512byte��
//����ֵ:0�� �ɹ�
//       other��ʧ��
uint8_t SD_ReadSingleBlock(uint32_t sector, uint8_t *buffer) {
	uint8_t r1;
	//����Ϊ����ģʽ
	SPIx_SetSpeed(SPIxSpeed_High);
	//�������SDHC����������sector��ַ������ת����byte��ַ
	if (SD_Type != SD_TYPE_V2HC) {
		sector = sector << 9;
	}
	r1 = SD_SendCommand(CMD17, sector, 0);//������
	if (r1 != 0x00)
		return r1;
	r1 = SD_ReceiveData(buffer, 512, RELEASE);
	if (r1 != 0)
		return r1; //�����ݳ���
	else
		return 0;
}
////////////////////////////����2������ΪUSB��д����Ҫ��/////////////////////////
//����SD���Ŀ��С
#define BLOCK_SIZE 512
//д��MSD/SD����
//pBuffer:���ݴ����
//ReadAddr:д����׵�ַ
//NumByteToRead:Ҫд����ֽ���
//����ֵ:0,д�����
//    ����,д��ʧ��
uint8_t MSD_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite) {
	uint32_t i, NbrOfBlock = 0, Offset = 0;
	uint32_t sector;
	uint8_t r1;
	NbrOfBlock = NumByteToWrite / BLOCK_SIZE;//�õ�Ҫд��Ŀ����Ŀ
	SD_CS(0);
	while (NbrOfBlock--)//д��һ������
	{
		sector = WriteAddr + Offset;
		if (SD_Type == SD_TYPE_V2HC)
			sector >>= 9;//ִ������ͨ�����෴�Ĳ���
		r1 = SD_SendCommand_NoDeassert(CMD24, sector, 0xff);//д����
		if (r1) {
			SD_CS(1);
			return 1;//Ӧ����ȷ��ֱ�ӷ���
		}
		SPIx_ReadWriteByte(0xFE);//����ʼ����0xFE
		//��һ��sector������
		for (i = 0; i < 512; i++)
			SPIx_ReadWriteByte(*pBuffer++);
		//��2��Byte��dummy CRC
		SPIx_ReadWriteByte(0xff);
		SPIx_ReadWriteByte(0xff);
		if (SD_WaitDataReady())//�ȴ�SD������д�����
		{
			SD_CS(1);
			return 2;
		}
		Offset += 512;
	}
	//д����ɣ�Ƭѡ��1
	SD_CS(1);
	SPIx_ReadWriteByte(0xff);
	return 0;
}
//��ȡMSD/SD����
//pBuffer:���ݴ����
//ReadAddr:��ȡ���׵�ַ
//NumByteToRead:Ҫ�������ֽ���
//����ֵ:0,�������
//    ����,����ʧ��
uint8_t MSD_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead) {
	uint32_t NbrOfBlock = 0, Offset = 0;
	uint32_t sector = 0;
	uint8_t r1 = 0;
	NbrOfBlock = NumByteToRead / BLOCK_SIZE;
	SD_CS(0);
	while (NbrOfBlock--) {
		sector = ReadAddr + Offset;
		if (SD_Type == SD_TYPE_V2HC)
			sector >>= 9;//ִ������ͨ�����෴�Ĳ���
		r1 = SD_SendCommand_NoDeassert(CMD17, sector, 0xff);//������
		if (r1)//����ʹ���
		{
			SD_CS(1);
			return r1;
		}
		r1 = SD_ReceiveData(pBuffer, 512, RELEASE);
		if (r1)//��������
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
//д��SD����һ��block(δʵ�ʲ��Թ�)
//����:uint32_t sector ������ַ��sectorֵ���������ַ��
//     uint8_t *buffer ���ݴ洢��ַ����С����512byte��
//����ֵ:0�� �ɹ�
//       other��ʧ��
uint8_t SD_WriteSingleBlock(uint32_t sector, const uint8_t *data) {
	uint8_t r1;
	uint16_t i;
	uint16_t retry;

	//����Ϊ����ģʽ
	//SPIx_SetSpeed(SPI_SPEED_HIGH);
	//�������SDHC����������sector��ַ������ת����byte��ַ
	if (SD_Type != SD_TYPE_V2HC) {
		sector = sector << 9;
	}
	r1 = SD_SendCommand(CMD24, sector, 0x00);
	if (r1 != 0x00) {
		return r1; //Ӧ����ȷ��ֱ�ӷ���
	}

	//��ʼ׼�����ݴ���
	SD_CS(0);
	//�ȷ�3�������ݣ��ȴ�SD��׼����
	SPIx_ReadWriteByte(0xff);
	SPIx_ReadWriteByte(0xff);
	SPIx_ReadWriteByte(0xff);
	//����ʼ����0xFE
	SPIx_ReadWriteByte(0xFE);

	//��һ��sector������
	for (i = 0; i < 512; i++) {
		SPIx_ReadWriteByte(*data++);
	}
	//��2��Byte��dummy CRC
	SPIx_ReadWriteByte(0xff);
	SPIx_ReadWriteByte(0xff);

	//�ȴ�SD��Ӧ��
	r1 = SPIx_ReadWriteByte(0xff);
	if ((r1 & 0x1F) != 0x05) {
		SD_CS(1);
		return r1;
	}

	//�ȴ��������
	retry = 0;
	while (!SPIx_ReadWriteByte(0xff)) {
		retry++;
		if (retry > 0xfffe) //�����ʱ��д��û����ɣ������˳�
		{
			SD_CS(1);
			return 1; //д�볬ʱ����1
		}
	}
	//д����ɣ�Ƭѡ��1
	SD_CS(1);
	SPIx_ReadWriteByte(0xff);

	return 0;
}
//��SD���Ķ��block(ʵ�ʲ��Թ�)
//����:uint32_t sector ������ַ��sectorֵ���������ַ��
//     uint8_t *buffer ���ݴ洢��ַ����С����512byte��
//     uint8_t count ������count��block
//����ֵ:0�� �ɹ�
//       other��ʧ��
uint8_t SD_ReadMultiBlock(uint32_t sector, uint8_t *buffer, uint8_t count) {
	uint8_t r1;
	//SPIx_SetSpeed(SPI_SPEED_HIGH);//����Ϊ����ģʽ
	//�������SDHC����sector��ַת��byte��ַ
	if (SD_Type != SD_TYPE_V2HC)
		sector = sector << 9;
	//SD_WaitDataReady();
	//�����������
	r1 = SD_SendCommand(CMD18, sector, 0);//������
	if (r1 != 0x00)
		return r1;
	do//��ʼ��������
	{
		if (SD_ReceiveData(buffer, 512, NO_RELEASE) != 0x00)
			break;
		buffer += 512;
	} while (--count);
	//ȫ��������ϣ�����ֹͣ����
	SD_SendCommand(CMD12, 0, 0);
	//�ͷ�����
	SD_CS(1);
	SPIx_ReadWriteByte(0xFF);
	if (count != 0)
		return count; //���û�д��꣬����ʣ�����
	else
		return 0;
}
//д��SD����N��block(δʵ�ʲ��Թ�)
//����:uint32_t sector ������ַ��sectorֵ���������ַ��
//     uint8_t *buffer ���ݴ洢��ַ����С����512byte��
//     uint8_t count д���block��Ŀ
//����ֵ:0�� �ɹ�
//       other��ʧ��
uint8_t SD_WriteMultiBlock(uint32_t sector, const uint8_t *data, uint8_t count) {
	uint8_t r1;
	uint16_t i;
	//SPIx_SetSpeed(SPI_SPEED_HIGH);//����Ϊ����ģʽ
	if (SD_Type != SD_TYPE_V2HC)
		sector = sector << 9;//�������SDHC����������sector��ַ������ת����byte��ַ
	if (SD_Type != SD_TYPE_MMC)
		r1 = SD_SendCommand(ACMD23, count, 0x00);//���Ŀ�꿨����MMC��������ACMD23ָ��ʹ��Ԥ����
	r1 = SD_SendCommand(CMD25, sector, 0x00);//�����д��ָ��
	if (r1 != 0x00)
		return r1; //Ӧ����ȷ��ֱ�ӷ���
	SD_CS(0);//��ʼ׼�����ݴ���
	SPIx_ReadWriteByte(0xff);//�ȷ�3�������ݣ��ȴ�SD��׼����
	SPIx_ReadWriteByte(0xff);
	//--------������N��sectorд���ѭ������
	do {
		//����ʼ����0xFC �����Ƕ��д��
		SPIx_ReadWriteByte(0xFC);
		//��һ��sector������
		for (i = 0; i < 512; i++) {
			SPIx_ReadWriteByte(*data++);
		}
		//��2��Byte��dummy CRC
		SPIx_ReadWriteByte(0xff);
		SPIx_ReadWriteByte(0xff);

		//�ȴ�SD��Ӧ��
		r1 = SPIx_ReadWriteByte(0xff);
		if ((r1 & 0x1F) != 0x05) {
			SD_CS(1); //���Ӧ��Ϊ��������������ֱ���˳�
			return r1;
		}
		//�ȴ�SD��д�����
		if (SD_WaitDataReady() == 1) {
			SD_CS(1); //�ȴ�SD��д����ɳ�ʱ��ֱ���˳�����
			return 1;
		}
	} while (--count);//��sector���ݴ������
	//��������������0xFD
	r1 = SPIx_ReadWriteByte(0xFD);
	if (r1 == 0x00) {
		count = 0xfe;
	}
	if (SD_WaitDataReady()) //�ȴ�׼����
	{
		SD_CS(1);
		return 1;
	}
	//д����ɣ�Ƭѡ��1
	SD_CS(1);
	SPIx_ReadWriteByte(0xff);
	return count; //����countֵ�����д����count=0������count=1
}
//��ָ������,��offset��ʼ����bytes���ֽ�
//����:uint32_t sector ������ַ��sectorֵ���������ַ��
//     uint8_t *buf     ���ݴ洢��ַ����С<=512byte��
//     uint16_t offset  �����������ƫ����
//     uint16_t bytes   Ҫ�������ֽ���
//����ֵ:0�� �ɹ�
//       other��ʧ��
uint8_t SD_Read_Bytes(unsigned long address, unsigned char *buf,
		unsigned int offset, unsigned int bytes) {
	uint8_t r1;
	uint16_t i = 0;
	r1 = SD_SendCommand(CMD17, address << 9, 0);//���Ͷ���������
	if (r1)
		return r1; //Ӧ����ȷ��ֱ�ӷ���
	SD_CS(0);//ѡ��SD��
	if (SD_GetResponse(0xFE))//�ȴ�SD������������ʼ����0xFE
	{
		SD_CS(1); //�ر�SD��
		return 1;//��ȡʧ��
	}
	for (i = 0; i < offset; i++)
		SPIx_ReadWriteByte(0xff);//����offsetλ
	for (; i < offset + bytes; i++)
		*buf++ = SPIx_ReadWriteByte(0xff);//��ȡ��������
	for (; i < 512; i++)
		SPIx_ReadWriteByte(0xff); //����ʣ���ֽ�
	SPIx_ReadWriteByte(0xff);//����αCRC��
	SPIx_ReadWriteByte(0xff);
	SD_CS(1);//�ر�SD��
	return 0;
}

