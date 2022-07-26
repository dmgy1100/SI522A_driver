#include "stdio.h"
#include "stdint.h"

#include "si522aDef.h"
#define REGISTER_PARAMETER_DEFAULT    {0x54, 0x04, 0x3f, 0x58}
#define SI5XX_DEV_VERSION     0x82

typedef struct
{
  bool spi_connect_status;                //spi连接状态
  bool dev_antenna_status;                //设备天线驱动状态
} si5xx_dev_status;
/*si5xx的部分寄存器可变参数的默认值*/
#define REGISTER_PARAMETER_DEFAULT    {0x54, 0x04, 0x3f, 0x58}

typedef struct
{
  bool spi_connect_status;
  bool dev_antenna_status;
} si5xx_dev_status;

typedef struct
{
  uint8_t set_nocar_field;
  uint8_t set_check_sensitivity;
  uint8_t set_voltage_gain;
  uint8_t set_receive_gain;
} si5xx_register_parameter;

#if SI512
  typedef enum err_masg{
    P2P_ERR_DEFAULT = 0x00,
    P2P_RECIVE,
    P2P_SEND
  }si512_err_massage;
#endif
typedef enum err_massage{
  ERR_DEFAULT = 0x00,
  PCD_REQUEST ,
  PCD_ANTICOLL,
  PCD_SELECT,
  PCD_AUTHSTATE,
  PCD_READ_BLOCK,
  PCD_WRITE_BLOCK
}si5xx_err_massage;

typedef struct card_massage
{
  uint8_t ATQA[2];    //卡的类型
  uint8_t UID[12];   //出厂默认卡号
  uint8_t SAK;
  uint8_t read_block_number;            //读的块号
  uint8_t write_block_number;           //写的块号
  uint8_t DefaultKeyA_Buf[6];           //密钥A
  uint8_t DefaultKeyB_Buf[6];           //密钥B
  uint8_t read_block_date[16];          //读块区数据
  uint8_t write_block_date[16];         //写块区数据
}read_card_massage;
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
  si5xx_err_massage err_massage;
  si5xx_dev_status dev_status;
  si5xx_register_parameter dev_register_parameter;
  read_card_massage card_massage;
#ifdef SI512
  si512_err_massage p2p_err_massage;
#endif
} si5xx_dev;

void get_si5xx_status(si522_dev* dev, si5xx_dev_status *dev_status);
static bool check_spi_connect(si522_dev* dev);
static bool check_dev_antenna_status(si522_dev* dev);

void si5xx_reg_write(si5xx_dev* dev, uint8_t addr, uint8_t value);
uint8_t si5xx_reg_read(si5xx_dev* dev, uint8_t addr);
void si52xx_reg_clear_bitmask(si5xx_dev* dev, uint8_t reg, uint8_t mask);
void si5xx_reg_set_bitmask(si5xx_dev* dev, uint8_t reg, uint8_t mask);
void si5xx_reg_simodify(si5xx_dev* dev, uint8_t RegAddr, uint8_t ModifyVal, uint8_t MaskByte);

void si5xx_pcdantenna_on(si5xx_dev* dev);
void si5xx_pcdantenna_off(si5xx_dev* dev);
void si5xx_pcd_config_isotype(si5xx_dev *dev, uint8_t type);                  //PCD模式读A类开寄存器配置
static void si5xx_caluate_crc(si5xx_dev* dev, uint8_t *pIndata,uint8_t len,uint8_t *pOutData);
/*与卡数据交换*/
int8_t si5xx_com_mf522(si5xx_dev* dev, uint8_t Command, uint8_t *pInData, uint8_t InLenByte,uint8_t *pOutData, uint8_t *pOutLenBit);

int8_t si5xx_pcd_request(si5xx_dev* dev, uint8_t req_code, uint8_t * pTagType ); //寻卡

int8_t si5xx_pcd_select (si5xx_dev* dev, uint8_t * pSnr, uint8_t *sak);          //选卡
int8_t si5xx_pcd_select1 (si5xx_dev* dev, uint8_t * pSnr, uint8_t *sak);
int8_t si5xx_pcd_select2 (si5xx_dev* dev, uint8_t * pSnr, uint8_t *sak);
int8_t si5xx_pcd_select3 (si5xx_dev* dev, uint8_t * pSnr, uint8_t *sak);
/*防冲突*/
int8_t si5xx_pcd_anticoll (si5xx_dev *dev, uint8_t *pSnr, uint8_t anticollision_level);

/*让卡片进入休眠模式*/
int8_t si5xx_pcd_halt(si5xx_dev* dev);
/*密码校验*/
int8_t si5xx_pcd_authstate(si5xx_dev* dev, uint8_t auth_mode,uint8_t addr,uint8_t *pKey,uint8_t *pSnr);
/*向卡里写数据*/
int8_t si5xx_pcd_write (si5xx_dev* dev, uint8_t ucAddr, uint8_t * pData );
/*读卡里数据*/
int8_t si5xx_pcd_read (si5xx_dev* dev, uint8_t ucAddr, uint8_t * pData );

//***********************************//修改新增内容

void si5xx_pcd_typea_init(si5xx_dev* dev);   //设置SI522A的工作方式初始化
int8_t si5xx_pcd_typea_getuid(si5xx_dev* dev);   //读A卡UID
int8_t PCD_SI522A_TypeA_rw_block(si5xx_dev* dev);   //读A卡扇区

int8_t si5xx_pcd_isblock( uint8_t ucAddr );           //判断是否为数据块
int8_t si5xx_user_write_block(si5xx_dev* dev);
int8_t si5xx_user_read_block(si5xx_dev* dev);
//***********************************//修改新增内容

void si5xx_reset(si5xx_dev* dev); //初始化
void si5xx_powerdown (si5xx_dev* dev);
//acd模式
void sixx_pcd_m500_config_isotype(si5xx_dev *dev);
void si5xx_pcd_auto_calc(si5xx_dev *dev);
void si5xx_acd_start(si5xx_dev *dev);
void si5xx_irq_clear(si5xx_dev *dev);                       //清中断
int8_t si5xx_irq(si5xx_dev* dev);
