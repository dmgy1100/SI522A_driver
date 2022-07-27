#include <si522a.h>

unsigned char  g_fwi = 4;
unsigned char flag_read_ic_ok;
#define PICC_CID 0x00

void si5xx_reg_write (si5xx_dev *dev, uint8_t addr, uint8_t value)
{
  uint8_t tx_buf[10] = {0};
  uint8_t rx_buf[10] = {0};
  addr = (addr & 0x3f) << 1;
  tx_buf[0] = addr;
  tx_buf[1] = value;
  dev->cs_low();
  dev->writeData(tx_buf, 2);
  dev->cs_high();
}
uint8_t si5xx_reg_read (si5xx_dev *dev, uint8_t addr)
{
  uint8_t tx_buf[10] = {0};
  uint8_t rx_buf[10] = {0};

  addr = (addr & 0x3f) << 1 | 0x80;   //code the first byte
  tx_buf[0] = addr;

  dev->cs_low ();
  dev->write_read_data(tx_buf, rx_buf, 2);
  dev->cs_high ();

  return rx_buf[1];
}

//***********************************//修改新增内容
uint8_t ACDConfigRegK_Val;
uint8_t ACDConfigRegC_Val;

//开启天线
//每次启动或关闭天险发射之间应至少有1ms的间隔
void si5xx_pcdantenna_on (si5xx_dev *dev)
{
    si5xx_reg_write(dev, TxControlReg,si5xx_reg_read(dev, TxControlReg) | 0x03);  //Tx1RFEn=1  Tx2RFEn=1
    i = si5xx_reg_read(dev, TxControlReg);
    dev->delay_ms(1);      //这里加一个小延时，会好一点！
}
//关闭天线
void si5xx_pcdantenna_off (si5xx_dev *dev)
{
  si5xx_reg_write(dev, TxControlReg,si5xx_reg_read(dev, TxControlReg)&(~0x03));
  i = si522a_reg_read(dev, TxControlReg);
}

//用MF522计算CRC16函数
static void si5xx_caluate_crc (si5xx_dev *dev, unsigned char *pIndata, unsigned char len,
                  unsigned char *pOutData)
{
  uint8_t i, n;
  si5xx_reg_clear_bitmask (dev, DivIrqReg, 0x04);
  si5xx_reg_write (dev, CommandReg, PCD_IDLE);
  si52xx_reg_set_bitmask (dev, FIFOLevelReg, 0x80);
  for (i = 0; i < len; i++)
  {
    si5xx_reg_write (dev, FIFODataReg, *(pIndata + i));
  }
  si5xx_reg_write (dev, CommandReg, PCD_CALCCRC);
  i = 0xFF;
  do
  {
    n = si5xx_reg_read (dev, DivIrqReg);
    i--;
  }
  while ((i != 0) && !(n & 0x04));
  pOutData[0] = si5xx_reg_read (dev, CRCResultRegL);
  pOutData[1] = si5xx_reg_read (dev, CRCResultRegH);
}

//功    能：通过RC522和ISO14443卡通讯
//参数说明：Command[IN]:RC522命令字
//          pInData[IN]:通过RC522发送到卡片的数据
//          InLenByte[IN]:发送数据的字节长度
//          pOutData[OUT]:接收到的卡片返回数据
//          *pOutLenBit[OUT]:返回数据的位长度
static int8_t si5xx_com_mf522 (si5xx_dev *dev, uint8_t Command, uint8_t *pInData,
                         uint8_t InLenByte, uint8_t *pOutData,
                         uint8_t *pOutLenBit)
{
  int8_t status = MI_ERR;
  uint8_t irqEn = 0x00;
  uint8_t waitFor = 0x00;
  uint8_t lastBits = 0;
  uint8_t n = 0;
  uint8_t i = 0;
  switch (Command)
  {
    case PCD_AUTHENT:
      irqEn = 0x12;
      waitFor = 0x10;
      break;
    case PCD_TRANSCEIVE:
      irqEn = 0x77;
      waitFor = 0x30;
      break;
    default:
      break;
  }

  si5xx_reg_write(dev, ComIEnReg,irqEn|0x80);
  si5xx_reg_clear_bitmask (dev, ComIrqReg, 0x80);

  si5xx_reg_write (dev, CommandReg, PCD_IDLE);

  dev->delay_ms(1);
  si5xx_reg_set_bitmask (dev, FIFOLevelReg, 0x80);

  for (i = 0; i < InLenByte; i++)
  {
    si5xx_reg_write (dev, FIFODataReg, pInData[i]);
  }
  si5xx_reg_write (dev, CommandReg, Command);

  if (Command == PCD_TRANSCEIVE)
  {
    si5xx_reg_set_bitmask (dev, BitFramingReg, 0x80);
  }

  //i = 600;//根据时钟频率调整，操作M1卡最大等待时间25ms
  i = 200;
  do
  {
    n = si5xx_reg_read (dev, ComIrqReg);
    i--;
  }
  while ((i != 0) && !(n & 0x01) && !(n & waitFor));
  si5xx_reg_clear_bitmask (dev, BitFramingReg, 0x80);

  if (i != 0)
  {
    if (!(si5xx_reg_read (dev, ErrorReg) & 0x1B))
    {
      status = MI_OK;
      if (n & irqEn & 0x01)
      {
        status = MI_NOTAGERR;
      }
      if (Command == PCD_TRANSCEIVE)
      {
        n = si5xx_reg_read (dev, FIFOLevelReg);
        lastBits = si5xx_reg_read (dev, ControlReg) & 0x07;
        if (lastBits)
        {
          *pOutLenBit = (n - 1) * 8 + lastBits;
        }
        else
        {
          *pOutLenBit = n * 8;
        }
        if (n == 0)
        {
          n = 1;
        }
        if (n > MAXRLEN)
        {
          n = MAXRLEN;
        }
        for (i = 0; i < n; i++)
        {
          pOutData[i] = si5xx_reg_read (dev, FIFODataReg);
        }
      }
    }
    else
    {
      status = MI_ERR;
    }

  }

  si5xx_reg_set_bitmask (dev, ControlReg, 0x80);           // stop timer now
  si5xx_reg_write (dev, CommandReg, PCD_IDLE);
  return status;
}
//功    能：寻卡
//参数说明: req_code[IN]:寻卡方式
//                0x52 = 寻感应区内所有符合14443A标准的卡
//                0x26 = 寻未进入休眠状态的卡
//          pTagType[OUT]：卡片类型代码
//                0x4400 = Mifare_UltraLight
//                0x0400 = Mifare_One(S50)
//                0x0200 = Mifare_One(S70)
//                0x0800 = Mifare_Pro(X)
//                0x4403 = Mifare_DESFire
//返    回: 成功返回MI_OK
int8_t si5xx_pcd_request (si5xx_dev *dev, uint8_t req_code, uint8_t *pTagType)
{
  int8_t status;
  uint8_t unLen;
  uint8_t ucComMF522Buf[MAXRLEN] = {0};

  si5xx_reg_clear_bitmask (dev, Status2Reg, 0x08);            //复位接收机和发射机状态
  si5xx_reg_write (dev, BitFramingReg, 0x07);             //
  si5xx_reg_set_bitmask (dev, TxControlReg, 0x03);

  ucComMF522Buf[0] = req_code;

  status = si5xx_com_mf522 (dev, PCD_TRANSCEIVE, ucComMF522Buf, 1, ucComMF522Buf,
                        &unLen);
  if ((status == MI_OK) && (unLen == 0x10))
  {
    *pTagType = ucComMF522Buf[0];
    *(pTagType + 1) = ucComMF522Buf[1];
  }
  else
  {
    status = MI_ERR;
    dev->err_massage = PCD_REQUEST;
  }

  return status;
}
//功    能：防冲撞
//参数说明: pSnr[OUT]:卡片序列号，4字节
//返    回: 成功返回MI_OK
int8_t si5xx_pcd_anticoll (si5xx_dev *dev, unsigned char *pSnr, unsigned char anticollision_level)
{
  int8_t status;
  uint8_t i, snr_check = 0;
  uint8_t unLen;
  uint8_t ucComMF522Buf[MAXRLEN];

  si5xx_reg_clear_bitmask (dev, Status2Reg, 0x08);
  si5xx_reg_write (dev, BitFramingReg, 0x00);
  si5xx_reg_clear_bitmask (dev, CollReg, 0x80);

  ucComMF522Buf[0] = anticollision_level;
  ucComMF522Buf[1] = 0x20;

  status = si5xx_com_mf522 (dev, PCD_TRANSCEIVE, ucComMF522Buf, 2, ucComMF522Buf,
                        &unLen);

  if (status == MI_OK)
  {
    for (i = 0; i < 4; i++)
    {
      *(pSnr + i) = ucComMF522Buf[i];
      snr_check ^= ucComMF522Buf[i];
    }
    if (snr_check != ucComMF522Buf[i])
    {
      status = MI_ERR;
      dev->err_massage = PCD_ANTICOLL;
    }
  }
  si5xx_reg_set_bitmask (dev, CollReg, 0x80);
  return status;
}
//功    能：选定卡片
//参数说明: pSnr[IN]:卡片序列号，4字节
//返    回: 成功返回MI_OK
int8_t si5xx_pcd_select (si5xx_dev *dev, uint8_t *pSnr, uint8_t *sak)
{
  int8_t status;
  uint8_t i;
  uint8_t unLen;
  uint8_t ucComMF522Buf[MAXRLEN];

  ucComMF522Buf[0] = PICC_ANTICOLL1;
  ucComMF522Buf[1] = 0x70;
  ucComMF522Buf[6] = 0;
  for (i = 0; i < 4; i++)
  {
    ucComMF522Buf[i + 2] = *(pSnr + i);
    ucComMF522Buf[6] ^= *(pSnr + i);
  }
  si5xx_caluate_crc (dev, ucComMF522Buf, 7, &ucComMF522Buf[7]);

  si5xx_reg_clear_bitmask (dev, Status2Reg, 0x08);

  status = si5xx_com_mf522 (dev, PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf,
                        &unLen);

  if ((status == MI_OK) && (unLen == 0x18))
  {
    *sak = ucComMF522Buf[0];
    status = MI_OK;
  }
  else
  {
    status = MI_ERR;
    dev->err_massage = PCD_SELECT;
  }
  return status;
}

int8_t si5xx_pcd_select1 (si5xx_dev *dev, uint8_t *pSnr, uint8_t *sak)
{
  int8_t status;
  uint8_t i;
  uint8_t unLen;
  uint8_t ucComMF522Buf[MAXRLEN];

  ucComMF522Buf[0] = PICC_ANTICOLL1;
  ucComMF522Buf[1] = 0x70;
  ucComMF522Buf[6] = 0;
  for (i = 0; i < 4; i++)
  {
    ucComMF522Buf[i + 2] = *(pSnr + i);
    ucComMF522Buf[6] ^= *(pSnr + i);
  }
  si5xx_caluate_crc (dev, ucComMF522Buf, 7, &ucComMF522Buf[7]);

  si5xx_reg_clear_bitmask (dev, Status2Reg, 0x08);

  status = si5xx_com_mf522 (dev, PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf,
                        &unLen);

  if ((status == MI_OK) && (unLen == 0x18))
  {
    *sak = ucComMF522Buf[0];
    status = MI_OK;
  }
  else
  {
    status = MI_ERR;
    dev->err_massage = PCD_SELECT;
  }

  return status;
}

int8_t si5xx_pcd_select2 (si5xx_dev *dev, uint8_t *pSnr, uint8_t *sak)
{
  int8_t status;
  uint8_t i;
  uint8_t unLen;
  uint8_t ucComMF522Buf[MAXRLEN];

  ucComMF522Buf[0] = PICC_ANTICOLL2;
  ucComMF522Buf[1] = 0x70;
  ucComMF522Buf[6] = 0;
  for (i = 0; i < 4; i++)
  {
    ucComMF522Buf[i + 2] = *(pSnr + i);
    ucComMF522Buf[6] ^= *(pSnr + i);
  }
  si5xx_caluate_crc (dev, ucComMF522Buf, 7, &ucComMF522Buf[7]);

  si5xx_reg_clear_bitmask (dev, Status2Reg, 0x08);

  status = si5xx_com_mf522 (dev, PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf,
                        &unLen);

  if ((status == MI_OK) && (unLen == 0x18))
  {
    *sak = ucComMF522Buf[0];
    status = MI_OK;
  }
  else
  {
    status = MI_ERR;
    dev->err_massage = PCD_SELECT;
  }

  return status;
}

int8_t si5xx_pcd_select3 (si5xx_dev *dev, uint8_t *pSnr, uint8_t *sak)
{
  int8_t status;
  uint8_t i;
  uint8_t unLen;
  uint8_t ucComMF522Buf[MAXRLEN];

  ucComMF522Buf[0] = PICC_ANTICOLL2;
  ucComMF522Buf[1] = 0x70;
  ucComMF522Buf[6] = 0;
  for (i = 0; i < 4; i++)
  {
    ucComMF522Buf[i + 2] = *(pSnr + i);
    ucComMF522Buf[6] ^= *(pSnr + i);
  }
  si5xx_caluate_crc (dev, ucComMF522Buf, 7, &ucComMF522Buf[7]);

  si5xx_reg_clear_bitmask (dev, Status2Reg, 0x08);

  status = si5xx_com_mf522 (dev, PCD_TRANSCEIVE, ucComMF522Buf, 9, ucComMF522Buf,
                        &unLen);

  if ((status == MI_OK) && (unLen == 0x18))
  {
    *sak = ucComMF522Buf[0];
    status = MI_OK;
  }
  else
  {
    status = MI_ERR;
    dev->err_massage = PCD_SELECT;
  }

  return status;
}
//功    能：命令卡片进入休眠状态
//返    回: 成功返回MI_OK
int8_t si522a_pcd_halt (si522_dev *dev)
{
  int8_t status;
  uint8_t unLen;
  uint8_t ucComMF522Buf[MAXRLEN];

  ucComMF522Buf[0] = PICC_HALT;
  ucComMF522Buf[1] = 0;
  si522a_caluate_crc (dev, ucComMF522Buf, 2, &ucComMF522Buf[2]);

  status = si522a_com_mf522 (dev, PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf,
                        &unLen);
  if (status != MI_OK)
    dev->err_massage = PCD_HALT;
  return status;
}
//功    能：验证卡片密码
//参数说明: auth_mode[IN]: 密码验证模式
//                 0x60 = 验证A密钥
//                 0x61 = 验证B密钥
//          addr[IN]：块地址
//          pKey[IN]：密码
//          pSnr[IN]：卡片序列号，4字节
//返    回: 成功返回MI_OK
int8_t si5xx_pcd_read (si5xx_dev *dev, uint8_t addr, uint8_t *pData)
{
  int8_t status;
  uint8_t unLen;
  uint8_t ucComMF522Buf[MAXRLEN];

  ucComMF522Buf[0] = PICC_READ;
  ucComMF522Buf[1] = addr;
  si5xx_caluate_crc (dev, ucComMF522Buf, 2, &ucComMF522Buf[2]);

  status = si5xx_com_mf522 (dev, PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf,
                        &unLen);
  if ((status == MI_OK) && (unLen == 0x90))
  {
    memcpy (pData, ucComMF522Buf, 16);
    memcpy (dev->card_massage.read_block_date, pData, 16);
    dev->card_massage.read_block_number = addr;
  }
  else
  {
    status = MI_ERR;
    dev->err_massage = PCD_READ_BLOCK;
  }

  return status;
}

//功    能：写数据到M1卡一块
//参数说明: addr[IN]：块地址
//          pData[IN]：写入的数据，16字节
//返    回: 成功返回MI_OK
int8_t si5xx_pcd_write (si5xx_dev *dev, uint8_t addr, uint8_t *pData)
{
  int8_t status;
  uint8_t unLen;
  uint8_t ucComMF522Buf[MAXRLEN];

  ucComMF522Buf[0] = PICC_WRITE;
  ucComMF522Buf[1] = addr;
  si5xx_caluate_crc (dev, ucComMF522Buf, 2, &ucComMF522Buf[2]);

  status = si5xx_com_mf522 (dev, PCD_TRANSCEIVE, ucComMF522Buf, 4, ucComMF522Buf,
                        &unLen);

  if ((status != MI_OK) || (unLen != 4) || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
  {
    status = MI_ERR;
    dev->err_massage = PCD_WRITE_BLOCK;
  }

  if (status == MI_OK)
  {
    memcpy (ucComMF522Buf, pData, 16);
    si5xx_caluate_crc (dev, ucComMF522Buf, 16, &ucComMF522Buf[16]);

    status = si5xx_com_mf522 (dev, PCD_TRANSCEIVE, ucComMF522Buf, 18, ucComMF522Buf,
                          &unLen);
    if ((status != MI_OK) || (unLen != 4)
        || ((ucComMF522Buf[0] & 0x0F) != 0x0A))
    {
      status = MI_ERR;
      dev->err_massage = PCD_WRITE_BLOCK;
    }
  }
  return status;
}

//设置SI522A的工作方式初始化
void si5xx_pcd_typea_init (si5xx_dev *dev)
{
    si5xx_reset(dev);
    si5xx_pcdantenna_off(dev);
    si5xx_pcd_m500_config_isotype(dev);
}

void si5xx_pcd_config_isotype(si5xx_dev *dev, uint8_t type)
{
//    dev->reset_low();
//    dev->delay_ms(0.5);
//    dev->reset_high();
//    dev->delay_ms(0.5);
//    si5xx_pcdantenna_off(dev);
//    dev->delay_ms(0.5);

  if (type == 'A') //ISO14443_A卡
  {
    si5xx_reg_clear_bitmask (dev, Status2Reg, BIT3); //清MFCrypto1On
    si5xx_reg_set_bitmask (dev, ComIEnReg, BIT7); //低电平触发中断
    si5xx_reg_write (dev, ModeReg, 0x3D);  //和Mifare卡通讯，CRC初始值0x6363
    si5xx_reg_write (dev, RxSelReg, 0x86);  //RxWait,延迟RxWait个比特时间后激活接收机
    si5xx_reg_write (dev, RFCfgReg, 0x48);  //接收增益,0x38 - 0x78,最大0x7F
    si5xx_reg_write (dev, TxASKReg, 0x40);  // typeA
    si5xx_reg_write (dev, TxModeReg, 0x00); //Tx Framing A
    si5xx_reg_write (dev, RxModeReg, 0x00);  //Rx framing A
    si5xx_reg_write (dev, ControlReg, 0x10);
    si5xx_reg_simodify (dev, 0x01, 0, 0x20); // Turn on the analog part of receiver

    //      I_SI522A_IO_Write(RxSelReg,0x88); //延迟RxWait个比特，更长？
    //      I_SI522A_IO_Write(ModGsPReg,0x12);  //0x0F);//调制指数 P驱动电导
    si522a_pcdantenna_on(dev);
  }
  else if ('B' == type)
  {
    si5xx_reg_clear_bitmask (dev, Status2Reg, 0X08); //清MFCrypto1On
    si5xx_reg_set_bitmask (dev, ComIEnReg, 0X80); //低电平触发中断
    si5xx_reg_write (dev, ModeReg, 0x3F);  //CRC初始值0xFFFF
    si5xx_reg_write (dev, RxSelReg, 0x85);  //RxWait,延迟RxWait个比特时间后激活接收机
    si5xx_reg_write (dev, RFCfgReg, 0x58);  //接收增益,0x38 - 0x78,最大0x7F
    //发射部分配置
    si5xx_reg_write (dev, GsNReg, 0xF8);  //调制系数
    si5xx_reg_write (dev, CWGsPReg, 0x3F);
    si5xx_reg_write (dev, ModGsPReg, 0x07);  //调制系数
    si5xx_reg_write (dev, AutoTestReg, 0x00); //AmpRcv为1，接收内部信号处理过程是非线性的
    si5xx_reg_write (dev, TxASKReg, 0x00);  // typeB
    si5xx_reg_write (dev, TypeBReg, 0x13);
    si5xx_reg_write (dev, TxModeReg, 0x83); //Tx Framing B
    si5xx_reg_write (dev, RxModeReg, 0x83);  //Rx framing B
    si5xx_reg_write (dev, BitFramingReg, 0x00);  ////TxLastBits=0
    si5xx_reg_write (dev, ControlReg, 0x10);
    si5xx_reg_simodify (dev, 0x01, 0, 0x20); // Turn on the analog part of receiver
    //  I_SI522A_IO_Write(RxThresholdReg, 0x65);    //55 65      // 高四位->最小信号强度，低三位->冲突最小信号强度,最大0xF7,最小0x44
    si522a_pcdantenna_on(dev);
  }
  else if ('C' == type)
  {
    si5xx_reg_clear_bitmask (dev, Status2Reg, BIT3);   //清MFCrypto1On
    si5xx_reg_set_bitmask (dev, ComIEnReg, BIT7);      //低电平触发中断
    si5xx_reg_write (dev, ControlReg, 0x10);       //设置为发起者
    si5xx_reg_write (dev, ModeReg, 0x38);         //CRC初始值0x0000
    si5xx_reg_write (dev, RxSelReg, 0x84);        //RxWait,延迟RxWait个比特时间后激活接收机
    si5xx_reg_write (dev, RFCfgReg, 0x58); //接收增益,0x38 - 0x78,可设置0x79,增加检测器的灵敏度,最大0x7F
    //发射部分配置
    si5xx_reg_write (dev, GsNReg, 0xf8);            //调制系数
    si5xx_reg_write (dev, CWGsPReg, 0x30);
    si5xx_reg_write (dev, ModGsPReg, 0x0e);         //调制系数
    si5xx_reg_write (dev, AutoTestReg, 0x00);       //AmpRcv为1，接收内部信号处理过程是非线性的
    si5xx_reg_write (dev, TxASKReg, 0x00);          // typeC
    si5xx_reg_write (dev, TypeBReg, 0x00);
    si5xx_reg_write (dev, Fel1Reg, 0x00);           // Fel1Reg 同步字节 B2 4D
    si5xx_reg_write (dev, MfRxReg, 0x11);    //ParityDisable校验关闭，信号的频谱下至212kHz
    si5xx_reg_write (dev, DemodReg, 0x41);           //PLL相关
    si5xx_reg_write (dev, TxModeReg, 0x92);          //Tx Framing C
    si5xx_reg_write (dev, RxModeReg, 0x92);          //Rx framing C

    si5xx_reg_write (dev, RxThresholdReg, 0x65); //55 65      // 高四位->最小信号强度，低三位->冲突最小信号强度,最大0xF7,最小0x44
    si5xx_pcdantenna_on(dev);
  }
  else if (type == 'D')                     //PN512_ISO14443_B
  {
    si5xx_reg_write (dev, TxASKReg, 0x00);
    si5xx_reg_write (dev, ControlReg, 0x10);       //设为发起者
    si5xx_reg_write (dev, TxModeReg, 0x03);        //
    si5xx_reg_write (dev, RxModeReg, 0x0B);        //忽视一个少于4位的无效数据
    si5xx_reg_write (dev, TypeBReg, 0x03);       //

    si5xx_reg_write (dev, DemodReg, 0x4d);         //
    si5xx_reg_write (dev, GsNReg, 0xFF);
    si5xx_reg_write (dev, CWGsPReg, 0x3F);         //
    si5xx_reg_write (dev, ModGsPReg, 0x04);      //07
    si5xx_reg_write (dev, RxThresholdReg, 0x55);     //
    si5xx_reg_write (dev, RFCfgReg, 0x68);

    si5xx_pcdantenna_on(dev);
  }
  else if (type == 'E')                     //PN512_NFCIP_1
  {
    //    si522a_reg_write(, 0x37);
    //      WriteRawRC(, 0x10);       //设为发起者
    //      si522a_reg_write(, 0x92);       //
    //    si522a_reg_write(, 0x96);       //
    //
    //      si522a_reg_write(, 0x41);         //
    //      si522a_reg_write(, 0xFF);
    //      si522a_reg_write(, 0x3F);         //
    //      si522a_reg_write(, 0x07);
    //      WriteRawRC(, 0x55);     //
    //    si522a_reg_write(, 0x59);
  }
  else if (type == 'F')                     //PN512_Felica
  {
    si5xx_reg_clear_bitmask (dev, Status2Reg, 0x08);
    si5xx_reg_write (dev, ControlReg, 0x10);       //设为发起者
    si5xx_reg_write (dev, CommandReg, 0x30);
    si5xx_reg_write (dev, CommandReg, 0x20);
    dev->delay_ms(0.5);

    si5xx_reg_write (dev, ModeReg, 0x38);          //TXwait,RAwait;SiGIN为高
    si5xx_reg_write (dev, TxModeReg, 0xa2);        //非106Kbit时置首位，选择速率为212Kbit;模式选择Felica
    si5xx_reg_write (dev, RxModeReg, 0xa6);              //非106Kbit时置首位，选择速率为212Kbit;模式选择Felica
    si5xx_reg_write (dev, TxControlReg, 0x83);     //首位对应TX2反相，后两位对应打开天线83
    si5xx_reg_write (dev, TxASKReg, 0x00);         //也即复位值，与天线自动开关有关，（这位必须要写，否则会置位
    //      si522a_reg_write(TxSelReg, 0x10);       //也即复位值，天线的输入由内部产生
    //      si522a_reg_write(RxSelReg, 0x84);         //也即复位值，调制信号来自内部模拟部分；RXwait部分
    si5xx_reg_write (dev, RxThresholdReg, 0x55);   //接受信号的临界值
    si5xx_reg_write (dev, DemodReg, 0x41);         //选择最好的通信信道；PLL时间相关
    //    si522a_reg_write(FelNFC1Reg, 0x00);       //也即复位值，crc长度；定义接收包的最小长度，（此时不忽略任何一个值）
    //    si522a_reg_write(MfRxReg, 0x11);          //校验位当成数据来处理；高通角频率设置为低于212KHz
    //    si522a_reg_write(TypeBReg, 0x00);         //也即复位值，
    //    si522a_reg_write(GsNOffReg, 0x88);        //也即复位值，
    //    si522a_reg_write(ModWidthReg, 0x26);        //也即复位值
    si5xx_reg_write (dev, RFCfgReg, 0x69);         //RXGain
    si5xx_reg_write (dev, GsNReg, 0xFF);           //88
    si5xx_reg_write (dev, CWGsPReg, 0x3F);         //30
    si5xx_reg_write (dev, ModGsPReg, 0x04);        //调制系数到10%ASK

    si5xx_pcdantenna_on(dev);
  }
  else if (type == 'G')                     //PN512_TEST
  {
    //    si522a_reg_write(, 0x37);
    //      si522a_reg_write(, 0x10);       //设为发起者
    //      si522a_reg_write(, 0x92);       //
    //    si522a_reg_write(, 0x96);       //
    //
    //      si522a_reg_write(, 0x41);         //
    //      si522a_reg_write(, 0xFF);
    //      si522a_reg_write(, 0x3F);         //
    //      si522a_reg_write(, 0x07);
    //      WriteRawRC(, 0x55);     //
    //    WriteRawRC(, 0x59);
  }
  else if (type == 'H')                     //PN512_P2P_Initiator_Active
  {

    si5xx_reg_write (dev, ControlReg, 0x10);       //设为发起者
    si5xx_reg_write (dev, ModeReg, 0x38);        //enable CRC
    //    si522a_reg_write(TxASKReg, 0x40);       //force 100ASK
    si5xx_reg_write (dev, DemodReg, 0x41);       //Set PLL

    //      si522a_reg_write(GsNReg, 0xFF);         //
    si5xx_reg_write (dev, 0x27, 0xff);           //负载调制的寄存器********此处用于编码
    si5xx_reg_write (dev, 0x28, 0x3f);           //负载调制的寄存器********此处用于编码
    si5xx_reg_write (dev, ModGsPReg, 0x04);        //调制系数到10%ASK

    si5xx_reg_write (dev, TxModeReg, 0xa1);      //disable 212kbps/A
    si5xx_reg_write (dev, RxModeReg, 0xa1);        //disable 212kbps/A
    si5xx_reg_write (dev, RxSelReg, 0x89);       //Rxwait
    si5xx_reg_write (dev, TxControlReg, 0x83);       //Tx
    //    WriteRawRC(BitFramingReg, 0x07);    //7bits Tx
    si5xx_reg_write (dev, MfRxReg, 0x10);        //disable p
    //      RFON
    //    COMMAND, TRANSCEIVE
    //    FIFODATA, wfifo
    //    BITFRAMING, data statt
    si5xx_pcdantenna_on(dev);
  }
  else if (type == 'I')                     //PN512_P2P_Target_Active
  {
    si5xx_reg_write (dev, ControlReg, 0x00);       //设为target,也即默认值
    si5xx_reg_write (dev, ModeReg, 0x38);        //enable CRC
    //    si522a_reg_write(TxASKReg, 0x40);       //force 100ASK
    si5xx_reg_write (dev, DemodReg, 0x41);       //Set PLL

    //      WriteRawRC(GsNReg, 0xFF);         //
    si5xx_reg_write (dev, 0x27, 0xff);           //负载调制的寄存器********此处用于编码
    si5xx_reg_write (dev, 0x28, 0x3f);
    si5xx_reg_write (dev, ModGsPReg, 0x04);        //调制系数到10%ASK

    si5xx_reg_write (dev, TxModeReg, 0xa1);      //disable 106kbps/A
    si5xx_reg_write (dev, RxModeReg, 0xa1);        //disable 106kbps/A
    si5xx_reg_write (dev, RxSelReg, 0x89);       //Rxwait
    si5xx_reg_write (dev, TxControlReg, 0x83);       //Tx
    //    WriteRawRC(BitFramingReg, 0x07);        //7bits Tx
    si5xx_reg_write (dev, MfRxReg, 0x10);        //disable p

    si5xx_pcdantenna_off (dev);

  }
  else if (type == 'J')                     //PN512_P2P_Target_Passive
  {
    //复位
    si5xx_reg_write (dev, ModeReg, 0x39);                 //enable CRC
    si5xx_reg_write (dev, TxModeReg, 0x00);         //disable 106kbps/A
    si5xx_reg_write (dev, RxModeReg, 0x00);        //disable 106kbps/A
    si5xx_reg_write (dev, MfRxReg, 0x10);           //disable p 接受和发送的校验位被关闭

    si5xx_reg_write (dev, DivIrqReg, 0x08);         //modeirq
    si5xx_reg_write (dev, ComIEnReg, 0xe1);      //rxirq, txirq, tirq
    si5xx_reg_write (dev, FelNFC2Reg, 0x80);        //waitforselected

    //    si522a_reg_write(GsNOffReg, 0x);      //负载调制的系数
    si5xx_reg_write (dev, TxControlReg, 0x04);      //waitforselected

    si5xx_pcdantenna_on(dev);
  }
  else if (type == 'K')                         //PN512_Operation_MIfareA
  {
    //检波配置
    si5xx_reg_write (dev, CommandReg, 0x0f);
    si5xx_reg_write (dev, ModeReg, 0x39);             //enable CRC
    si5xx_reg_write (dev, RxThresholdReg, 0x55);     //定义最小信号长度
    si5xx_reg_write (dev, RFCfgReg, 0x59);                  //RXGain:38dB&RX上检测电平大小
    si5xx_reg_write (dev, DemodReg, 0x4d);            //Set PLL

    si5xx_reg_write (dev, TxModeReg, 0x00);           //106kbps/A
    si5xx_reg_write (dev, RxModeReg, 0x00);           //106kbps/A
    si5xx_reg_write (dev, MfRxReg, 0x10);             //disable p

    si5xx_reg_write (dev, DivIEnReg, 0x08);           //Allows the mode interrupt request
    si5xx_reg_write (dev, ComIEnReg, 0xe1);            //rxirq, txirq, tirq

    si5xx_reg_write (dev, FelNFC2Reg, 0x80);        //waitforselected
    si5xx_reg_write (dev, MfTxReg, 0x62);              //txwait 9bits

    //被检测到，返回2字节的卡类型号
    si5xx_pcdantenna_off (dev);
  }
  else if (type == 'L')                             //PN512_Operation_Felica
  {
    si5xx_reg_write (dev, CommandReg, 0x0f);
    si5xx_reg_write (dev, ModeReg, 0x38);              //enable CRC
    si5xx_reg_write (dev, RxThresholdReg, 0x55);       //定义最小信号长度
    si5xx_reg_write (dev, RFCfgReg, 0x59);             //RXGain:38dB&RX上检测电平大小
    si5xx_reg_write (dev, DemodReg, 0x4d);             //Set PLL

    //    si522a_reg_write(TxModeReg, 0xa2);      //212kbps/A**************
    //    si522a_reg_write(RxModeReg, 0xa6);      //212kbps/A**************
    si5xx_reg_write (dev, MfRxReg, 0x10);        //disable p

    si5xx_reg_write (dev, 0x23, 0xe8);           //负载调制必须的两个寄存器
    si5xx_reg_write (dev, 0x27, 0xe8);           //负载调制的寄存器********

    si5xx_reg_write (dev, DivIEnReg, 0x08);      //Allows the mode interrupt request
    //    WriteRawRC(ComIEnReg, 0xe1);
    //rxirq, txirq, tirq

    si5xx_reg_write (dev, FelNFC2Reg, 0x80);       //waitforselected
    si5xx_reg_write (dev, MfTxReg, 0x62);                //txwait 9bits

    si5xx_pcdantenna_on(dev);
  }
  dev->delay_ms (2);
}

//
int8_t si5xx_pcd_typea_getuid (si5xx_dev *dev)
{
  uint8_t UID[12]={0};
  uint8_t SAK = 0;
  uint8_t ATQA[2] = {0};
  uint8_t UID_complate1 = 0;
  uint8_t UID_complate2 = 0;
  uint8_t UID_complate3 = 0;
  NRF_LOG_INFO("Test_Si522A_GetUID");
  //I_SI522A_IO_Write (dev, RFCfgReg, RFCfgReg_Val); //复位接收增益

  //寻卡  连续读三次，如果都是失败就是有问题
  if (si522a_pcd_request ( dev, PICC_REQALL, ATQA) != MI_OK)  ////寻天线区内全部卡
  {
    si522a_reg_write (dev, RFCfgReg, 0x48);
    if (si522a_pcd_request ( dev, PICC_REQALL, ATQA) != MI_OK)
    {
      si522a_reg_write (dev, RFCfgReg, 0x58);
      if (si522a_pcd_request ( dev, PICC_REQALL, ATQA) != MI_OK)
      {
        dev->err_massage = PCD_REQUEST;
        return MI_ERR;
      }
      else
      {
        dev->card_massage.ATQA[0] = ATQA[0];
        dev->card_massage.ATQA[1] = ATQA[1];
      }
    }
    else
    {
       dev->card_massage.ATQA[0] = ATQA[0];
       dev->card_massage.ATQA[1] = ATQA[1];
    }
  }
  else
  {  
      dev->card_massage.ATQA[0] = ATQA[0];
      dev->card_massage.ATQA[1] = ATQA[1];
  }

//UID长度=4
  //Anticoll 冲突检测 level1
  if (si522a_pcd_anticoll (dev, UID, PICC_ANTICOLL1) != MI_OK)
  {
    dev->err_massage = PCD_ANTICOLL;
    return MI_ERR;
  }
  else
  {
    if (si522a_pcd_select1 (dev, UID, &SAK) != MI_OK)
    {
      dev->err_massage = PCD_SELECT;
      return MI_ERR;
    }
    else
    {
      dev->card_massage.SAK = SAK;
      if (SAK & 0x04)
      {
        UID_complate1 = 0;

        //UID长度=7
        if (UID_complate1 == 0)
        {
          //Anticoll 冲突检测 level2
          if (si522a_pcd_anticoll (dev, UID + 4, PICC_ANTICOLL2) != MI_OK)
          {
            dev->err_massage = PCD_ANTICOLL;
            return MI_ERR;
          }
          else
          {
            if (si522a_pcd_select2 (dev, UID + 4, &SAK) != MI_OK)
            {
              dev->err_massage = PCD_SELECT;
              return MI_ERR;
            }
            else
            {
              dev->card_massage.SAK = SAK;
              if (SAK & 0x04)
              {
                UID_complate2 = 0;

                //UID长度=10
                if (UID_complate2 == 0)
                {
                  //Anticoll 冲突检测 level3
                  if (si522a_pcd_anticoll (dev, UID + 8, PICC_ANTICOLL3) != MI_OK)
                  {
                    dev->err_massage = PCD_ANTICOLL;
                    return MI_ERR;
                  }
                  else
                  {
                    if (si522a_pcd_select3 (dev, UID + 8, &SAK) != MI_OK)
                    {
                      dev->err_massage = PCD_ANTICOLL;
                      return MI_ERR;
                    }
                    else
                    {
                      dev->card_massage.SAK = SAK;
                      if (SAK & 0x04)
                      {
                        UID_complate3 = 0;
                      }
                      else
                      {
                         UID_complate3 = 1;
                         memcpy (dev->card_massage.UID, UID, 12);
                      }
                    }
                  }
                }
              }
              else
              {
                UID_complate2 = 1;
              }
            }
          }
        }
      }
      else
      {
        UID_complate1 = 1;
        memcpy (dev->card_massage.UID, UID, 12);
      }
    }
  }

  dev->delay_ms (10);
  return 0;
}

//得到卡号
/*
 通过RC522和M1卡通讯（数据的双向传输）
 寻卡
 防冲突
 用RC522计算CRC16（循环冗余校验）
 选定卡片
 校验卡片密码
 在M1卡的指定块地址写入指定数据
 读取M1卡的指定块地址的数据
 让卡片进入休眠模式
 */
int8_t PCD_SIxxA_TypeA_rw_block (si5xx_dev *dev)
{
  uint8_t ATQA[2];    //卡的类型
  uint8_t UID[12];   //出厂默认卡号
  uint8_t SAK = 0;   //..
  uint8_t CardReadBuf[16] =
  { 0 };  //从卡的读取的数据0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xBA

  uint8_t card_data_buf[16] =
  { 0x02, 0x02, 0x02, 0x02 };
  //对扇区2 块3 写入的数据，密钥A 为FFFFFFFFF　操作权限为FF 07 80 69 密钥B 为 0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xBA
  //08778F00 控制字. A不可被读出,B不可被读出 通过B可以写密码A，密钥B 通过密钥A/B控制字A可读可写
  //控制字为 0  1   1  Never  KeyB  KeyA/B  KeyB  Never KeyB      使用密钥B　使用该控制字08778F00
  uint8_t CardWriteBuf[16] =
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x08, 0x77, 0x8F, 0x00, 0xFF, 0xAA,
      0xBB, 0xCC, 0xDD, 0xBA }; //写入卡的数据
  //unsigned char DefaultKeyABuf[10] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  //密钥A
  uint8_t DefaultKeyA_Buf[6] =
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }; //密钥，， 默认都是FFFFFFFFFF

  //密钥B
  uint8_t DefaultKeyB_Buf[6] =
  { 0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xBA }; //密钥，， 默认都是FFFFFFFFFF

  //1. request 寻卡
  if (si5xx_pcd_request ( dev, PICC_REQIDL, ATQA) != MI_OK)  //寻天线区内未进入休眠状态的卡，返回卡片类型 2字节
  {
    dev->err_massage = PCD_REQUEST;
    return MI_ERR;
  }
  else
  {
    dev->card_massage.ATQA[0] = ATQA[0];
    dev->card_massage.ATQA[1] = ATQA[1];
  }

  //2. Anticoll 冲突检测
  if (si5xx_pcd_anticoll (dev, UID, PICC_ANTICOLL1) != MI_OK)
  {
    dev->err_massage = PCD_ANTICOLL;
    return MI_ERR;
  }
  else
  {
    memcpy (dev->card_massage.UID, UID, 12);
  }

  //3. Select 选卡
  if (si5xx_pcd_select1 (dev, UID, &SAK) != MI_OK)
  {
    dev->err_massage = PCD_SELECT;
    return MI_ERR;
  }
  else
  {
    dev->card_massage.SAK = SAK;
  }
  /*先通过A的密钥，然后修改B的密钥*/
  if (si5xx_pcd_authstate ( dev, PICC_AUTHENT1A, 7, DefaultKeyB_Buf, UID) != MI_OK)
  {
    dev->err_massage = PCD_AUTHSTATE;
    return MI_ERR;
  }
  //5. 读BLOCK原始数据 读出
  if (si5xx_pcd_read (dev, 8, CardReadBuf) != MI_OK)
  {
    dev->err_massage = PCD_READ_BLOCK;
    return MI_ERR;
  }
  else
  {
    for (unsigned char i = 0; i < 16; i++)
    {
      dev->card_massage.read_block_number = 8;
   memcpy (dev->card_massage.read_block_date, CardReadBuf, 16);
    }
  }

  //产生随机数  导致丢失了扇区2 .。密钥丢了
//  for(unsigned char i=0;i<16;i++)
//    CardWriteBuf[i] = rand();

  //写BLOCK 写入新的数据
//  if( PcdWrite( 11, CardWriteBuf ) != MI_OK )
//  {
//    printf("\r\nPcdWrite:fail");
//    return 1;
//  }
//  else
//  {
//    printf("\r\nPcdWrite:ok  ");
//    for(unsigned char i=0;i<16;i++)
//    {
//      printf(" %02x",CardWriteBuf[i]);
//    }
//  }
//
//  if( PcdWrite( 8, card_data_buf ) != MI_OK )
//  {
//    printf("\r\nPcdWrite:fail");
//    return 1;
//  }
//  else
//  {
//    printf("\r\nPcdWrite:ok  ");
//    for(unsigned char i=0;i<16;i++)
//    {
//      printf(" %02x",card_data_buf[i]);
//    }
//  }
//
  //读BLOCK 读出新写入的数据
  if (si5xx_pcd_read (dev, 11, CardReadBuf) != MI_OK)
  {
    dev->err_massage = PCD_READ_BLOCK;
    return MI_ERR;
  }
  else
  {
    dev->card_massage.read_block_number = 11;
    memcpy (dev->card_massage.read_block_date, CardReadBuf, 16);
  }
  return MI_OK;
}

//***********************************//修改新增内容

/*
 * 函数名：PcdReset
 * 描述  ：复位RC522
 * 输入  ：无
 * 返回  : 无
 * 调用  ：外部调用
 */
void si5xx_reset (si5xx_dev *dev)
{
  dev->delay_ms(0.05);
 #if 1
 //  SET_NFC_RST;    //RST拉高
   dev->reset_low;    //RST拉低
   dev->delay_ms(10);

 //  delay_ms(2);
   dev->reset_high();    //RST拉高
   dev->delay_ms(2);
 #endif
   dev->delay_ms(0.5);  //需要与客户确认delayus函数延时准确。

   si5xx_reg_write(dev, CommandReg, 0x0f); //向CommandReg 写入 0x0f 作用是使Si522复位
   dev->delay_ms(1);
   si5xx_reg_write(dev, ModeReg, 0x3D);   //和Mifare卡通讯，CRC初始值0x6363

   si5xx_reg_write(dev, TReloadRegL, 30);  //重装定时器值低位
   si5xx_reg_write(dev, TReloadRegH, 0); //重装定时器值高位
   si5xx_reg_write(dev, TModeReg, 0x8D); //跟随协议启动和停止
   si5xx_reg_write(dev, TPrescalerReg, 0x3E);  //6.78/3390=0.002Mhz,(1/2)*30=15ms产生中断

   si5xx_reg_write(dev, TxASKReg, 0x40);  //必须要
}
void si5xx_reg_clear_bitmask (si5xx_dev *dev, uint8_t reg, uint8_t mask)
{
  int8_t tmp = 0x00;
  tmp = si5xx_reg_read (dev, reg);
  si5xx_reg_write (dev, reg, tmp & ~mask);  // clear bit mask
}

void si5xx_reg_set_bitmask (si5xx_dev *dev, uint8_t reg, uint8_t mask)
{
  int8_t tmp = 0x00;
  tmp = si5xx_reg_read (dev, reg);
  si5xx_reg_write (dev, reg, tmp | mask);  // set bit mask
}

void si5xx_reg_simodify(si5xx_dev *dev, uint8_t RegAddr, uint8_t ModifyVal, uint8_t MaskByte)
{
  uint8_t RegVal;
  RegVal = si5xx_reg_read(dev, RegAddr);
  if(ModifyVal)
  {
      RegVal |= MaskByte;
  }
  else
  {
      RegVal &= (~MaskByte);
  }
  si5xx_reg_write(dev, RegAddr, RegVal);
}
/*读A类卡数据*/
void si5xx_pcd_m500_config_isotype(si5xx_dev *dev)   //ISO1443A//A:NXP,B:MOTO
{
  si5xx_reg_clear_bitmask(dev, Status2Reg, BIT3);   //清MFCrypto1On
  si5xx_reg_set_bitmask(dev, ComIEnReg, BIT7);      //低电平触发中断
  si5xx_reg_write(dev, ModeReg, 0x3D);              //和Mifare卡通讯，CRC初始值0x6363
  si5xx_reg_write(dev, RxSelReg, 0x86);             //RxWait,延迟RxWait个比特时间后激活接收机
  si5xx_reg_write(dev, RFCfgReg, 0x58);             //接收增益,0x38 - 0x78,最大0x7F
  si5xx_reg_write(dev, TxASKReg, 0x40);             // typeA
  si5xx_reg_write(dev, TxModeReg, 0x00);            //Tx Framing A
  si5xx_reg_write(dev, RxModeReg, 0x00);            //Rx framing A
  si5xx_reg_write(dev, ControlReg, 0x10);

  si5xx_pcdantenna_on(dev);
  dev->delay_ms(0.4);                 //400us
}
void si5xx_pcd_auto_calc(si5xx_dev *dev)
{
  uint8_t temp;
  uint8_t temp_Compare=0;
  uint8_t VCON_TR[8]={ 0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f};//acd灵敏度调节
  uint8_t TR_Compare[4]={ 0x00, 0x00, 0x00, 0x00};
  uint8_t ACDConfigRegC_RealVal = 0x7f;
  uint8_t ACDConfigRegK_RealVal = 0;
  si5xx_reg_write(dev, TxControlReg, 0x83);     //打开天线
  si5xx_reg_set_bitmask(dev, CommandReg, 0x06);     //开启ADC_EXCUTE
  dev->delay_ms(0.2);                             //200us
  //手动获取阈值
//  si522a_reg_write(dev, ACDConfigSelReg, (ACDConfigK << 2) | 0x40);
//  si522a_reg_write(dev, ACDConfigReg, 0x3f);
//  while(1)
//  {
//    si522a_reg_write(dev, ACDConfigSelReg, (ACDConfigG << 2) | 0x40);
//    temp_Compare = si522a_reg_read(dev, ACDConfigReg);
//    NRF_LOG_INFO("test ACDConfigRegG = %x", temp_Compare);
//    dev->delay_ms(500);
//  }
  again:
    NRF_LOG_INFO("again");
  for(int i=7; i>0; i--)
  {
    si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigK << 2) | 0x40);
    si5xx_reg_write(dev, ACDConfigReg, VCON_TR[i]);

    si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigG << 2) | 0x40);
    temp_Compare = si5xx_reg_read(dev, ACDConfigReg);
    for(int m=0;m<100;m++)
    {
      si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigG << 2) | 0x40);
      temp = si5xx_reg_read(dev, ACDConfigReg);
      if(temp == 0)
        break;                                          //处在接近的VCON值附近值，如果偶合出现0值，均有概率误触发，应舍弃该值。
      temp_Compare=(temp_Compare+temp)/2;
      dev->delay_ms(0.1);                               //100us
    }

    if(temp_Compare == 0 || temp_Compare == 0x7f)       //比较当前值和所存值
    {

    }
    else
    {
      if(temp_Compare < dev->dev_register_parameter.set_nocar_field)
      {
        dev->dev_register_parameter.set_nocar_field = temp_Compare;
        dev->dev_register_parameter.set_voltage_gain = VCON_TR[i];
      }
    }
  }
  ACDConfigRegK_RealVal = dev->dev_register_parameter.set_voltage_gain;     //取得最接近的参考电压VCON

  if ((dev->dev_register_parameter.set_voltage_gain == 0x0e) | (dev->dev_register_parameter.set_voltage_gain == 0x0f))
  {
      NRF_LOG_INFO (
          "Attention!!! The RX partial resistance ratio is too small,please turn down 5.1K resistance and try again!!");
      dev->delay_ms(2000);
      //goto again;
  }

  for(int j=0; j<4; j++)
  {
    si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigK << 2) | 0x40);
    si5xx_reg_write(dev, ACDConfigReg, j*32+dev->dev_register_parameter.set_voltage_gain);

    si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigG << 2) | 0x40);
    temp_Compare = si5xx_reg_read(dev, ACDConfigReg);
    for(int n=0;n<100;n++)
    {
      si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigG << 2) | 0x40);
      temp = si5xx_reg_read(dev, ACDConfigReg);
      temp_Compare=(temp_Compare+temp)/2;
      dev->delay_ms(0.1);                           //100us
    }
    TR_Compare[j] = temp_Compare;
  }//再调TR的档位，将采集值填入TR_Compare[]

  for(int z=0; z<4; z++)
  {
    if(TR_Compare[z] == 0x7f)
    {

    }
    else
    {
      dev->dev_register_parameter.set_nocar_field = TR_Compare[z];//最终选择的配置
      dev->dev_register_parameter.set_voltage_gain = 0x0c + z*32+16;
    }
  }//再选出一个非7f大值

  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigK << 2) | 0x40);
  NRF_LOG_INFO("ACDConfigRegK_Val:K = %02x ",dev->dev_register_parameter.set_voltage_gain);
  NRF_LOG_INFO("ACDConfigRegG_Val:C = %02x ",dev->dev_register_parameter.set_nocar_field);
  si5xx_reg_set_bitmask(dev, CommandReg, 0x06);    //关闭ADC_EXCUTE
}
void si5xx_acd_start(si5xx_dev *dev)
{
  si5xx_reg_write(dev, DivIrqReg, 0x60);  //清中断，该处不清中断，进入ACD模式后会异常产生有卡中断。
  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigJ << 2) | 0x40);
  si5xx_reg_write(dev, ACDConfigReg, 0x55); //Clear ACC_IRQ

  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigA << 2) | 0x40);          //设置轮询时间
  si5xx_reg_write(dev, ACDConfigReg, ACDConfigRegA_Val );
  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigB << 2) | 0x40);          //设置相对模式或者绝对模式
  si5xx_reg_write(dev, ACDConfigReg, ACDConfigRegB_Val );
  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigC << 2) | 0x40);          //设置无卡场强值
  si5xx_reg_write(dev, ACDConfigReg, dev->dev_register_parameter.set_nocar_field);
  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigD << 2) | 0x40);          //设置灵敏度，一般建议为4，在调试时，可以适当降低验证该值，验证ACD功能
  si5xx_reg_write(dev, ACDConfigReg, dev->dev_register_parameter.set_check_sensitivity );
  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigH << 2) | 0x40);          //设置看门狗定时器时间
  si5xx_reg_write(dev, ACDConfigReg, ACDConfigRegH_Val );
  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigI << 2) | 0x40);         //设置ARI功能，在天线场强打开前1us产生ARI电平控制触摸芯片Si12T的硬件屏蔽引脚SCT
  si5xx_reg_write(dev, ACDConfigReg, ACDConfigRegI_Val );
  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigK << 2) | 0x40);          //设置ADC的基准电压和放大增益
  si5xx_reg_write(dev, ACDConfigReg, dev->dev_register_parameter.set_voltage_gain);
  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigM << 2) | 0x40);          //设置监测ACD功能是否产生场强，意外产生可能导致读卡芯片复位或者寄存器丢失
  si5xx_reg_write(dev, ACDConfigReg, ACDConfigRegM_Val );
  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigO << 2) | 0x40);          //设置ACD模式下相关功能的标志位传导到IRQ引脚
  si5xx_reg_write(dev, ACDConfigReg, ACDConfigRegO_Val );

//  I_SI522A_IO_Write(dev, ACDConfigSelReg, (ACDConfigL << 2) | 0x40);          //设置ACD模式下相关功能的标志位传导到IRQ引脚
//  I_SI522A_IO_Write(dev, ACDConfigReg, 0x4c);

  si5xx_reg_write(dev, ComIEnReg, ComIEnReg_Val);                           //ComIEnReg，DivIEnReg   设置IRQ选择上升沿或者下降沿
  si5xx_reg_write(dev, DivIEnReg, DivIEnReg_Val);

  si5xx_reg_write(dev, ACDConfigSelReg, (ACDConfigJ << 2) | 0x40);        //设置监测ACD功能下的重要寄存器的配置值，寄存器丢失后会立即产生中断
  si5xx_reg_write(dev, ACDConfigReg, ACDConfigRegJ_Val );                 // 写非0x55的值即开启功能，写0x55清除使能停止功能。
  si5xx_reg_write(dev, CommandReg, 0xb0);                                 //进入ACD
}
int8_t si5xx_irq_irq(si5xx_dev *dev)
{
  unsigned char status_SI522ACD_IRQ;
  unsigned char temp_SI522ACD_IRQ;

  temp_SI522ACD_IRQ = si5xx_reg_read(dev, DivIrqReg);
  if  ( temp_SI522ACD_IRQ & 0x40) //ACD中断
  {
    si5xx_reg_write(dev, DivIrqReg, 0x40);   //Clear ACDIRq

    status_SI522ACD_IRQ =1;
    return status_SI522ACD_IRQ;
  }

  if ( temp_SI522ACD_IRQ & 0x20)  //ACD看门狗中断
  {
    si5xx_reg_write(dev, DivIrqReg, 0x20);   //Clear ACDTIMER_IRQ

    status_SI522ACD_IRQ = 2;
    return status_SI522ACD_IRQ;
  }
  si5xx_irq_clear(dev);

  return status_SI522ACD_IRQ = 0;
}
/*向卡片指定块写数据*/
int8_t si5xx_user_write_block(si5xx_dev* dev)
{
    unsigned char status;
    unsigned char ATQA[2];    //卡的类型
    unsigned char UID[12];   //出厂默认卡号
    unsigned char SAK = 0;   //..
    unsigned char w_addr = dev->card_massage.write_block_number;          //写块的地址
    unsigned char p_addr = dev->card_massage.write_block_number + dev->card_massage.write_block_number%3;   //写扇区密码块地址
    //unsigned char card_data_buf[16] = pData;
    uint8_t CardWriteBuf[16]  = {0};
    memcpy(CardWriteBuf, dev->card_massage.write_block_date, 16);

    if(si5xx_pcd_isblock(w_addr) != MI_OK)            //判断要写的是否为数据块
          return MI_ERR;

    //1. request 寻卡
    if (status = si5xx_pcd_request ( dev, PICC_REQIDL, ATQA) != MI_OK)  //寻天线区内未进入休眠状态的卡，返回卡片类型 2字节
    {
      dev->err_massage = PCD_REQUEST;
      return status;
    }
    else
    {
      dev->card_massage.ATQA[0] = ATQA[0];
      dev->card_massage.ATQA[1] = ATQA[1];
    }

    //2. Anticoll 冲突检测
    if (status = si5xx_pcd_anticoll (dev, UID, PICC_ANTICOLL1) != MI_OK)
    {
      dev->err_massage = PCD_ANTICOLL;
      return status;
    }
    else
    {
      memcpy ( dev->card_massage.UID, UID, 12);
    }

    //3. Select 选卡
    if (status = si5xx_pcd_select1 (dev, UID, &SAK) != MI_OK)
    {
      dev->err_massage = PCD_SELECT;
      return status;
    }
    else
    {
      dev->card_massage.SAK = SAK;
    }
    /*4 先通过A的密钥验证*/
    if (status = si5xx_pcd_authstate ( dev, PICC_AUTHENT1A, p_addr, dev->card_massage.DefaultKeyA_Buf, UID) != MI_OK)
    {
      dev->err_massage = PCD_AUTHSTATE;
      return status;
    }
    //5. 写BLOCK数据
    if (status = si5xx_pcd_write (dev, w_addr, CardWriteBuf) != MI_OK)
    {
      dev->err_massage = PCD_WRITE_BLOCK;
      return status;
    }
    return status;
}

/*读卡片指定块的数据*/
int8_t si5xx_user_read_block(si5xx_dev* dev)
{
    uint8_t status;
    uint8_t ATQA[2];    //卡的类型
    uint8_t UID[12];   //出厂默认卡号
    uint8_t SAK = 0;   //..
    uint8_t r_addr = dev->card_massage.read_block_number;          //读块的地址
    uint8_t p_addr = dev->card_massage.read_block_number + dev->card_massage.read_block_number%3;     //读密码块的地址
    //unsigned char card_data_buf[16] = pData;
    uint8_t CardReadBuf[16] = {0};
    if(si5xx_pcd_isblock(r_addr) != MI_OK)            //判断要读的是否为数据块
      return MI_ERR;

    //1. request 寻卡
    if (status = si5xx_pcd_request ( dev, PICC_REQALL, ATQA) != MI_OK)  //寻天线区内未进入休眠状态的卡，返回卡片类型 2字节
    {
      dev->err_massage = PCD_REQUEST;
      return status;
    }
    else
    {
      dev->card_massage.ATQA[0] = ATQA[0];
      dev->card_massage.ATQA[0] = ATQA[0];
    }

    //2. Anticoll 冲突检测
    if (status = si5xx_pcd_anticoll (dev, UID, PICC_ANTICOLL1) != MI_OK)
    {
      dev->err_massage = PCD_ANTICOLL;
      return status;
    }
    else
      memcpy (dev->card_massage.UID, UID, 12);

    //3. Select 选卡
    if (status = si5xx_pcd_select1 (dev, UID, &SAK) != MI_OK)
    {
      dev->err_massage = PCD_SELECT;
      return status;
    }
    else
      dev->card_massage.SAK = SAK;
    /*4 先通过A的密钥验证*/
    if (status = si5xx_pcd_authstate ( dev, PICC_AUTHENT1A, p_addr, dev->card_massage.DefaultKeyA_Buf, UID) != MI_OK)
    {
      dev->err_massage = PCD_AUTHSTATE;
      return status;
    }
    //5. 读BLOCK数据
    if (status = si5xx_pcd_read (dev, r_addr, CardReadBuf) != MI_OK)
    {
      dev->err_massage = PCD_READ_BLOCK;
      return status;
    }
    else
    {
      memcpy(dev->card_massage.read_block_date, CardReadBuf, 16);
    }
    return status;
}
/**
  * @brief  判断 ucAddr 是否数据块
  * @param  ucAddr，块绝对地址（0-63）
  * @retval 返回值 1:是数据块；0:不是数据块
  */
int8_t si5xx_pcd_isblock(si5xx_dev* dev, uint8_t ucAddr )
{
  if(ucAddr == 0)
  {
    return MI_ERR;
    dev->err_massage = PCD_BLOCK_ADDR;
  }
  /* 如果是数据块(不包含数据块0) */
  if( (ucAddr<64) && (((ucAddr+1)%4) != 0) )
  {
    return MI_OK;
  }
  else
  {
    dev->err_massage = PCD_BLOCK_ADDR;
  }
  return MI_ERR;
}
void si5xx_powerdown (si5xx_dev* dev)
{
  dev->reset_low(); //拉低硬复位
  dev->delay_ms(100);
  dev->reset_high(); //重新上电
  dev->delay_ms(100);
}
void si5xx_get_status(si5xx_dev* dev, si5xx_dev_status *dev_status)
{
  dev_status->spi_connect_status = si5xx_check_spi_connect(dev);
  dev_status->dev_antenna_status = si5xx_check_antenna_status(dev);
}
static bool si5xx_check_spi_connect(si5xx_dev* dev)
{
  uint8_t tmp = si5xx_reg_read (dev, 0x37);
  if (tmp == SI5XX_DEV_VERSION)
    return true;
  else
    return false;
}
static bool si5xx_check_antenna_status(si5xx_dev* dev)
{
  uint8_t i;
  si5xx_reg_write(dev, TxControlReg,si5xx_reg_read(dev, TxControlReg) | 0x03);  //Tx1RFEn=1  Tx2RFEn=1
  i = si5xx_reg_read(dev, TxControlReg);
  si5xx_reg_write(dev, TxControlReg,80);
  if (i == 0x83)
    return true;
  else
    return false;
}
