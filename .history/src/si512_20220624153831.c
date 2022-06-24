/*
 * si512.c
 *
 *  Created on: 2022年6月14日
 *      Author: LIUBING
 */
#include "si512.h"

#define MAXLEN  64

int8_t si512_p2p_rseceive_data(si522_dev *dev, uint8_t *r_data, uint8_t len)   //暂时没有资料
{
  int32_t RXIRQ=0;
  uint8_t CardReadBuf[MAXLEN] = {0};

  si522a_reg_clear_bitmask(dev, Status2Reg,0x08);
//  WriteRawRC(BitFramingReg,0x07);
  si522a_reg_set_bitmask(dev, TxControlReg,0x03);

  si522a_pcdantenna_on (dev);

  si522a_reg_write(dev, BitFramingReg,0x07);                       //清空fifo

  si522a_reg_write(dev, CommandReg, 0x08);            //激活接收电路

  uint8_t irq_reg;
  if(irq_reg = si522a_reg_read(dev, ComIrqReg)&0x20) //检测RXIRQ是否置位
  {
    si522a_reg_write(dev, ComIrqReg,0x20);//清理中断标志位
    RXIRQ=1;
    uint8_t fifo_len = 64;
    int i=0;
    for (i=0; i < fifo_len; i++)  //读空FIFO
    {
      CardReadBuf[i] = si522a_reg_read(dev, FIFODataReg);      //FIFO值
      //NRF_LOG_INFO("i = %d  rx is ok read_data = %x",i, CardReadBuf[i]);
      dev->delay_ms(1);
    }
    memcpy(r_data, CardReadBuf, len);
  }
  else
  {
    si522a_pcdantenna_off (dev);
    return 0;
  }
  si522a_pcdantenna_off (dev);
  return RXIRQ;
}

int8_t si512_p2p_send_data(si522_dev *dev, uint8_t * const w_data, uint8_t len)        //发送数据
{
  uint8_t unLen = len;
  uint32_t TXIRQ = 0;
  si522a_reg_clear_bitmask (dev, Status2Reg, 0x08);

  si522a_reg_write(dev, BitFramingReg,0x07);                       //清空fifo
  uint8_t i;
  si522a_reg_set_bitmask (dev, TxControlReg, 0x03);     //打开天线
  for(i = 0; i < unLen; i++)
  {
    si522a_reg_write (dev, FIFODataReg, w_data[i]);     //数据写到FIFO里
    NRF_LOG_INFO("i = %d     write_data = %x",i, w_data[i]);
  }
  si522a_reg_write (dev, CommandReg, 0x04);     //使能Transmit
  si522a_reg_set_bitmask(dev, 0x0d, 0x80);     //开始发送

  if (si522a_reg_read (dev, ComIrqReg) & 0x40)    //监测TxIRq
  {
    NRF_LOG_INFO (" TxIRq is set");
    TXIRQ = 1;
  }
  else
  {
    NRF_LOG_INFO ("\r\n waiting");
  }
  si522a_pcdantenna_off (dev);
  return TXIRQ;
}

