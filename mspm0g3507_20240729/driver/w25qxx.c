#include "ti_msp_dl_config.h"
#include "system.h"
#include "w25qxx.h"




#define W25Q64_CS_1   DL_GPIO_setPins(PORTB_PORT,   PORTB_W25Q64_CS_PIN)    //PB25
#define W25Q64_CS_0   DL_GPIO_clearPins(PORTB_PORT,   PORTB_W25Q64_CS_PIN)

uint16_t w25qxx_id = 0;
void W25QXX_PowerDown(void);
void W25QXX_WAKEUP(void);
void W25QXX_Erase_Chip(void);
void W25Q64_erase_sector(uint32_t addr);


void w25qxx_gpio_init(void)
{
  W25Q64_CS_1;
//	W25QXX_Erase_Chip();
//	W25Q64_erase_sector(0x00);

  W25QXX_PowerDown();
  Delay_Ms(10);
  W25QXX_WAKEUP();
  Delay_Ms(10);

  w25qxx_id = W25Q64_readID();	      //��ȡFLASH ID.
}


void w25qxx_erase_all(void)
{
  W25QXX_Erase_Chip();
  W25Q64_erase_sector(0x00);
}

#define fast_spi_enable 0



uint8_t spi_read_write_byte(uint8_t Byte)
{
  #if fast_spi_enable==1
  SPI_0_INST->TXDATA = Byte;

  while((SPI_0_INST->STAT & SPI_STAT_BUSY_MASK) == SPI_STAT_BUSY_ACTIVE);

  return ((uint8_t)(SPI_0_INST->RXDATA));
  #else
  DL_SPI_transmitData8(SPI_0_INST, Byte);  //����1�ֽ�����

  while(DL_SPI_isBusy(SPI_0_INST));        //�ȴ��������

  return DL_SPI_receiveData8(SPI_0_INST);  //��ս���FIFO�е�����
  #endif
}






/******************************************************************
 * �� �� �� �ƣ�W25Q64_readID
 * �� �� ˵ ������ȡW25Q64�ĳ���ID���豸ID
 * �� �� �� �Σ���
 * �� �� �� �أ��豸��������EF16
 * ��       �ߣ�LCKFB
 * ��       ע����
******************************************************************/
uint16_t W25Q64_readID(void)
{
  uint16_t  temp = 0;
  W25Q64_CS_0;

  spi_read_write_byte(0x90);//���Ͷ�ȡID����
  spi_read_write_byte(0x00);
  spi_read_write_byte(0x00);
  spi_read_write_byte(0x00);
  //��������
  temp |= spi_read_write_byte(0xFF) << 8;
  temp |= spi_read_write_byte(0xFF);

  W25Q64_CS_1;
  return temp;
}



/**********************************************************
 * �� �� �� �ƣ�W25Q64_wait_busy
 * �� �� �� �ܣ��ж�W25Q64�Ƿ�æ
 * �� �� �� ������
 * �� �� �� �أ���
 * ��       �ߣ�LCKFB
 * ��       ע����
**********************************************************/
void W25Q64_wait_busy(void)
{
  unsigned char byte = 0;

  do
  {
    W25Q64_CS_0;
    spi_read_write_byte(0x05);
    byte = spi_read_write_byte(0Xff);
    W25Q64_CS_1;
  }
  while( ( byte & 0x01 ) == 1 );
}


//�������ģʽ
void W25QXX_PowerDown(void)
{
  W25Q64_CS_0;                            //ʹ������
  spi_read_write_byte(W25X_PowerDown);     //���͵�������
  W25Q64_CS_1;                            //ȡ��Ƭѡ
  delay_us(3);                            //�ȴ�TPD
}

//����
void W25QXX_WAKEUP(void)
{
  W25Q64_CS_0;                                //ʹ������
  spi_read_write_byte(W25X_ReleasePowerDown);  //send W25X_PowerDown command 0xAB
  W25Q64_CS_1;                                //ȡ��Ƭѡ
  delay_us(3);                                //�ȴ�TRES1
}



/**********************************************************
 * �� �� �� �ƣ�W25Q64_write_enable
 * �� �� �� �ܣ�����дʹ��
 * �� �� �� ������
 * �� �� �� �أ���
 * ��       �ߣ�LCKFB
 * ��       ע����
**********************************************************/
void W25Q64_write_enable(void)
{
  W25Q64_CS_0;
  spi_read_write_byte(0x06);
  W25Q64_CS_1;
}

/**********************************************************
 * �� �� �� �ƣ�W25Q64_erase_sector
 * �� �� �� �ܣ�����һ������
 * �� �� �� ����addr=������������
 * �� �� �� �أ���
 * ��       �ߣ�LCKFB
 * ��       ע��addr=�����������ţ���Χ=0~15
**********************************************************/
void W25Q64_erase_sector(uint32_t addr)
{
  addr *= 4096;
  W25Q64_write_enable();  //дʹ��
  W25Q64_wait_busy();     //�ж�æ
  W25Q64_CS_0;
  spi_read_write_byte(0x20);
  spi_read_write_byte((uint8_t)((addr) >> 16));
  spi_read_write_byte((uint8_t)((addr) >> 8));
  spi_read_write_byte((uint8_t)addr);
  W25Q64_CS_1;
  //�ȴ��������
  W25Q64_wait_busy();
}


/**********************************************************
 * �� �� �� �ƣ�W25Q64_write
 * �� �� �� �ܣ�д���ݵ�W25Q64���б���
 * �� �� �� ����buffer=д�����������	addr=д���ַ	numbyte=д�����ݵĳ���
 * �� �� �� �أ���
 * ��       �ߣ�LCKFB
 * ��       ע����
**********************************************************/
void W25Q64_write(uint8_t* buffer, uint32_t addr, uint16_t numbyte)
{
  //0x02e21
  unsigned int i = 0;
  W25Q64_erase_sector(addr / 4096); //������������
  W25Q64_write_enable();//дʹ��
  W25Q64_wait_busy(); //æ���
  //д������
  W25Q64_CS_0;
  spi_read_write_byte(0x02);
  spi_read_write_byte((uint8_t)((addr) >> 16));
  spi_read_write_byte((uint8_t)((addr) >> 8));
  spi_read_write_byte((uint8_t)addr);

  for(i = 0; i < numbyte; i++)
  {
    spi_read_write_byte(buffer[i]);
  }

  W25Q64_CS_1;
  W25Q64_wait_busy(); //æ���
}

/**********************************************************
 * �� �� �� �ƣ�W25Q64_read
 * �� �� �� �ܣ���ȡW25Q64������
 * �� �� �� ����buffer=�������ݵı����ַ  read_addr=��ȡ��ַ		read_length=��ȥ����
 * �� �� �� �أ���
 * ��       �ߣ�LCKFB
 * ��       ע����
**********************************************************/
void W25Q64_read(uint8_t* buffer, uint32_t read_addr, uint16_t read_length)
{
  uint16_t i;
  W25Q64_CS_0;
  spi_read_write_byte(0x03);
  spi_read_write_byte((uint8_t)((read_addr) >> 16));
  spi_read_write_byte((uint8_t)((read_addr) >> 8));
  spi_read_write_byte((uint8_t)read_addr);

  for(i = 0; i < read_length; i++)
  {
    buffer[i] = spi_read_write_byte(0XFF);
  }

  W25Q64_CS_1;
}



//namelesstech
void W25QXX_Erase_Chip(void)
{
  W25Q64_write_enable();                  //SET WEL
  W25Q64_wait_busy();
  W25Q64_CS_0; //ʹ������
  spi_read_write_byte(0xC7);     //����Ƭ��������
  W25Q64_CS_1; //ȡ��Ƭѡ
  W25Q64_wait_busy();   				          //�ȴ�оƬ��������
}


void W25Q64_write_page(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint16_t i;
  W25Q64_write_enable();                  //SET WEL
  W25Q64_CS_0; //ʹ������
  spi_read_write_byte(0x02);   //����дҳ����
  spi_read_write_byte((uint8_t)((WriteAddr) >> 16)); //����24bit��ַ
  spi_read_write_byte((uint8_t)((WriteAddr) >> 8));
  spi_read_write_byte((uint8_t)WriteAddr);

  for(i = 0; i < NumByteToWrite; i++)	spi_read_write_byte(pBuffer[i]); //ѭ��д��

  W25Q64_CS_1; //ȡ��Ƭѡ
  W25Q64_wait_busy();					   					//�ȴ�д�����
}


//�޼���дSPI FLASH
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ����
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint16_t pageremain;
  pageremain = 256 - WriteAddr % 256; //��ҳʣ����ֽ���

  if(NumByteToWrite <= pageremain)pageremain = NumByteToWrite; //������256���ֽ�

  while(1)
  {
    W25Q64_write_page(pBuffer, WriteAddr, pageremain);

    if(NumByteToWrite == pageremain)	break; //д�������
    else //NumByteToWrite>pageremain
    {
      pBuffer += pageremain;
      WriteAddr += pageremain;

      NumByteToWrite -= pageremain;			 //��ȥ�Ѿ�д���˵��ֽ���

      if(NumByteToWrite > 256)pageremain = 256; //һ�ο���д��256���ֽ�
      else pageremain = NumByteToWrite; 	 //����256���ֽ���
    }
  }
}

//дSPI FLASH
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
uint8_t W25QXX_BUFFER[4096];
void W25QXX_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint32_t secpos;
  uint16_t secoff;
  uint16_t secremain;
  uint16_t i;
  uint8_t * W25QXX_BUF;
  W25QXX_BUF = W25QXX_BUFFER;
  secpos = WriteAddr / 4096; //������ַ
  secoff = WriteAddr % 4096; //�������ڵ�ƫ��
  secremain = 4096 - secoff; //����ʣ��ռ��С

  //printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//������
  if(NumByteToWrite <= secremain)secremain = NumByteToWrite; //������4096���ֽ�

  while(1)
  {
    W25Q64_read(W25QXX_BUF, secpos * 4096, 4096); //������������������

    for(i = 0; i < secremain; i++) //У������
    {
      if(W25QXX_BUF[secoff + i] != 0XFF)break; //��Ҫ����
    }

    if(i < secremain) //��Ҫ����
    {
      W25Q64_erase_sector(secpos); //�����������

      for(i = 0; i < secremain; i++)	 //����
      {
        W25QXX_BUF[i + secoff] = pBuffer[i];
      }

      W25QXX_Write_NoCheck(W25QXX_BUF, secpos * 4096, 4096); //д����������

    }
    else W25QXX_Write_NoCheck(pBuffer, WriteAddr, secremain); //д�Ѿ������˵�,ֱ��д������ʣ������.

    if(NumByteToWrite == secremain)break; //д�������
    else//д��δ����
    {
      secpos++;//������ַ��1
      secoff = 0; //ƫ��λ��Ϊ0

      pBuffer += secremain; //ָ��ƫ��
      WriteAddr += secremain; //д��ַƫ��
      NumByteToWrite -= secremain;				//�ֽ����ݼ�

      if(NumByteToWrite > 4096)secremain = 4096;	//��һ����������д����
      else secremain = NumByteToWrite;					//��һ����������д����
    }
  };
}


void W25QXX_Write_Data(float *data, uint32_t addr)
{
  uint8_t tempwr[4];
  tempwr[0] = (*(uint32_t *)(data)) & 0xff;
  tempwr[1] = (*(uint32_t *)(data)) >> 8;
  tempwr[2] = (*(uint32_t *)(data)) >> 16;
  tempwr[3] = (*(uint32_t *)(data)) >> 24;
  //W25QXX_Write_Page(tempwr,addr,4);
  W25QXX_Write(tempwr, addr, 4);
}


typedef union
{
  unsigned char Bdata[4];
  float Fdata;
  uint32_t Idata;
} Float_2_Byte;


void W25QXX_Read_Float(float *data, uint32_t addr)
{
  Float_2_Byte temp;
  W25Q64_read(temp.Bdata, addr, 4);
  *data = temp.Fdata;
}

void W25QXX_Read_f(float *pui32Data, uint32_t ui32Address, uint32_t ui32Count)
{
  Float_2_Byte temp;
  uint16_t i = 0;

  for(i = 0; i < ui32Count; i++)
  {
    W25Q64_read(temp.Bdata, ui32Address + 4 * i, 4);
    *pui32Data = temp.Fdata;
    pui32Data++;
  }
}


void W25QXX_Read_fs(float *Data, uint32_t ui32Address, uint32_t ui32Count)
{
  uint16_t i = 0;

  for(i = 0; i < ui32Count; i++)
  {
    Float_2_Byte temp;
    W25Q64_read(&temp.Bdata[0], ui32Address + 4 * i + 0, 1);
    W25Q64_read(&temp.Bdata[1], ui32Address + 4 * i + 1, 1);
    W25Q64_read(&temp.Bdata[2], ui32Address + 4 * i + 2, 1);
    W25Q64_read(&temp.Bdata[3], ui32Address + 4 * i + 3, 1);
    *Data = temp.Fdata;
    Data++;
  }
}


void W25QXX_Write_f(float *data, uint32_t ui32Address, uint32_t ui32Count)
{
  Float_2_Byte temp;
  uint16_t i = 0;

  for(i = 0; i < ui32Count; i++)
  {
    temp.Fdata = *data;
    W25QXX_Write(temp.Bdata, ui32Address + 4 * i, 4);
    data++;
  }
}






void Resume_Factory_Setting(void)
{
  W25QXX_Erase_Chip();
  W25Q64_erase_sector(0x00);
}

