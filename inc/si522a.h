#include "stdio.h"
#include "stdint.h"
#include "nrf_gpio.h"
#include "custom_board.h"
#include "nrf_log.h"
#include "si522aDef.h"

#include "main.h"

#define SI5XX_DEV_VERSION     0x82

typedef struct
{
  bool spi_connect_status;                //spi连接状态
  bool dev_antenna_status;                //设备天线驱动状态
} si5xx_dev_status;

typedef struct
{
  void (*readData) (uint8_t* pData, uint16_t Size);
  void (*writeData) (uint8_t* pData, uint16_t Size);
  void (*write_read_data)(const uint8_t *tx_data, uint8_t *rx_data, uint16_t len);
  void (*cs_low)(void);
  void (*cs_high)(void);
  void (*reset_low)(void);
  void (*reset_high)(void);
  void (*delay_ms) (uint32_t ms);
  si5xx_dev_status dev_status;
} si522_dev;

void get_si5xx_status(si522_dev* dev, si5xx_dev_status *dev_status);
bool check_spi_connect(si522_dev* dev);
bool check_dev_antenna_status(si522_dev* dev);

void si522a_reg_write(si522_dev* dev, uint8_t addr, uint8_t value);
uint8_t si522a_reg_read(si522_dev* dev, uint8_t addr);
void si522a_reg_clear_bitmask(si522_dev* dev, uint8_t reg, uint8_t mask);
void si522a_reg_set_bitmask(si522_dev* dev, uint8_t reg, uint8_t mask);
void si522a_reg_simodify(si522_dev* dev, uint8_t RegAddr, uint8_t ModifyVal, uint8_t MaskByte);

void si522a_pcdantenna_on(si522_dev* dev);
void si522a_pcdantenna_off(si522_dev* dev);
void si522a_pcd_config_isotype(si522_dev *dev, uint8_t type);                  //PCD模式读A类开寄存器配置
void si522a_caluate_crc(si522_dev* dev, uint8_t *pIndata,uint8_t len,uint8_t *pOutData);
/*与卡数据交换*/
int8_t si522a_com_mf522(si522_dev* dev, uint8_t Command, uint8_t *pInData, uint8_t InLenByte,uint8_t *pOutData, uint8_t *pOutLenBit);

int8_t si522a_pcd_request(si522_dev* dev, uint8_t req_code, uint8_t * pTagType ); //寻卡

int8_t si522a_pcd_select (si522_dev* dev, uint8_t * pSnr, uint8_t *sak);          //选卡
int8_t si522a_pcd_select1 (si522_dev* dev, uint8_t * pSnr, uint8_t *sak);
int8_t si522a_pcd_select2 (si522_dev* dev, uint8_t * pSnr, uint8_t *sak);
int8_t si522a_pcd_select3 (si522_dev* dev, uint8_t * pSnr, uint8_t *sak);
/*防冲突*/
int8_t si522a_pcd_anticoll (si522_dev *dev, uint8_t *pSnr, uint8_t anticollision_level);

/*让卡片进入休眠模式*/
int8_t si522a_pcd_halt(si522_dev* dev);
/*密码校验*/
int8_t si522a_pcd_authstate(si522_dev* dev, uint8_t auth_mode,uint8_t addr,uint8_t *pKey,uint8_t *pSnr);
/*向卡里写数据*/
int8_t si522a_pcd_write (si522_dev* dev, uint8_t ucAddr, uint8_t * pData );
/*读卡里数据*/
int8_t si522a_pcd_read (si522_dev* dev, uint8_t ucAddr, uint8_t * pData );

//***********************************//修改新增内容

void si522a_pcd_typea_init(si522_dev* dev);   //设置SI522A的工作方式初始化
int8_t si522a_pcd_typea_getuid(si522_dev* dev);   //读A卡UID
int8_t PCD_SI522A_TypeA_rw_block(si522_dev* dev);   //读A卡扇区

int8_t si522a_pcd_isblock( uint8_t ucAddr );           //判断是否为数据块
int8_t si522a_user_write_block(si522_dev* dev, uint8_t ucAddr, uint8_t * pData);
int8_t si522a_user_read_block(si522_dev* dev, uint8_t ucAddr, uint8_t * pData);
//***********************************//修改新增内容

void si522_reset(si522_dev* dev); //初始化
void si5xx_powerdown (si522_dev* dev);
//acd模式
void si522a_pcd_m500_config_isotype(si522_dev *dev);
void si522a_pcd_auto_calc(si522_dev *dev);
void si522a_acd_start(si522_dev *dev);
void si522a_irq_clear(si522_dev *dev);                       //清中断
int8_t si522a_irq(si522_dev* dev);
