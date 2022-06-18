# SI522A_driver
is a si522a demo
在应用层创建设备结构体并初始化
typedef struct SI522_DEV
{
void (*readData) (uint8_t* pData, uint16_t Size); //spi读操作函数
void (*writeData) (uint8_t* pData, uint16_t Size); //spi写操作函数
void (*write_read_data)(const uint8_t *tx_data, uint8_t *rx_data, uint16_t len); //spi读写操作函数
void (*cs_low)(void); //芯片片选置低电平
void (*cs_high)(void); //芯片片选置高电平
void (*reset_low)(void); //芯片复位引脚置低电平
void (*reset_high)(void); //芯片复位引脚置高电平
void (*delay_ms) (uint32_t ms); //芯片延时函数
} si522a_dev;
1.相关函数定义
1.初始化相关函数
void si522a_reset(si522_dev* dev) //芯片复位
void si522a_pcdantenna_On(si522_dev* dev) //打开天线
void si522a_pcdantenna_off(si522_dev* dev) //关闭天线
void si522a_pcd_typea_init(si522_dev* dev)//Reader模式的初始化

2.寄存器操作相关函数
/*
参数1：芯片结构体地址
参数2：寄存器地址
参数3：写入的数据
*/
void si522a_reg_write(si522_dev* dev, uint8_t addr, uint8_t value) //写寄存器函数
参数1：芯片结构体地址
参数2：寄存器地址
*/
uint8_t si522a_reg_read(si522_dev* dev, uint8_t addr) //读寄存器函数
/*
参数1：芯片结构体地址
参数2：寄存器地址
参数3：置位值
*/
void si522a_reg_clear_bitmask(si522_dev* dev, uint8_t reg, uint8_t mask) //寄存器值相应为置0
/*
参数1：芯片结构体地址
参数2：寄存器地址
参数3：置位值
*/
void si522a_reg_set_bitmask(si522_dev* dev, uint8_t reg, uint8_t mask) //寄存器值相应位值1
参数1：芯片结构体地址
参数2：寄存器地址
参数3：选择置0或置1
参数4：置位值
*/
void si522a_reg_simodify(si522_dev* dev,  RegAddr, uint8_t ModifyVal, uint8_t MaskByte) //将寄存器值相应位置1或0
PCD模式相关函数

/*
参数1：芯片结构体地址
参数2：读卡类型，传‘A’或‘B’
*/
void si522a_pcd_config_isotype(si522_dev *dev, uint8_t type) //PCD模式读A类或B类开寄存器配置
/*
参数1：芯片结构体地址
参数2：通信命令
参数3：发送数据
参数4：发送数据长度
参数5：接收数据
参数6：接收数据长度
*/
uint8_t si522a_com_mf522(si522_dev *dev, uint8_t Command, uint8_t *pInData,
uint8_tr InLenByte, uint8_t *pOutData,
uint8_t *pOutLenBit) //与卡通信
参数1：芯片结构体地址
参数2：校验数据输入
参数3：校验数据长度
参数4：校验完成输出
*/
void si522a_caluate_crc(si522_dev* dev, uint8_t *pIndata,uint8_t len,uint8_t *pOutData) //CRC校验
/*
参数1：芯片结构体地址
参数2：寻卡方式（输入）
参数3：卡片类型（输出）
*/
int8_t si522a_pcd_request(si522_dev* dev, uint8_t req_code, uint8_t * pTagType ) //寻卡
/*
参数1：芯片结构体地址
参数2：卡片UID（输出）
参数3：防冲突命令（输入）
*/
int8_t si522a_pcd_anticoll(si522_dev *dev, uint8_t *pSnr, uint8_t anticollision_level) //防冲突
/*
参数1：芯片结构体地址
参数2：卡片UID（输入）
参数3：卡片sak（输出）
*/
int8_t si522a_pcd_select(si522_dev* dev, uint8_t * pSnr, uint8_t *sak); //选卡
int8_t si522a_pcd_select1(si522_dev* dev, uint8_t * pSnr, uint8_t *sak);
int8_t si522a_pcd_select2(si522_dev* dev, uint8_t * pSnr, uint8_t *sak);
int8_t si522a_pcd_select3(si522_dev* dev, uint8_t * pSnr, uint8_t *sak);
/*
参数1：芯片结构体地址
参数2：密码验证模式，验证A密钥或者验证B密钥
参数3：块地址
参数4：密码
参数5：卡片UID
*/
int8_t si522a_pcd_authstate(si522_dev* dev, uint8_t auth_mode,uint8_t addr,uint8_t *pKey,uint8_t *pSnr) //密码校验
/*
参数1：芯片结构体地址
参数2：块地址
参数3：写入数据
*/
int8_t si522a_pcd_write (si522_dev* dev, uint8_t ucAddr, uint8_t * pData ) //向卡里指定块写数据
/*
参数1：芯片结构体地址
参数2：块地址
参数3：读到的数据（输出）
*/
int8_t si522a_pcd_read(si522_dev* dev, uint8_t ucAddr, uint8_t * pData ) //读卡指定块的数据

3.ACD模式相关函数
void si522a_pcd_auto_calc(si522_dev *dev) //计算ADC参考电压和场强增益
void si522a_pcd_m500_config_isotype(si522_dev *dev) //读A类卡寄存器配置
void si522a_acd_start(si522_dev *dev) //进入低功耗自动寻卡模式，检测到卡自动退出
int8_t si522a_irq(si522_dev* dev) //判断产生的是什么中断
void si522a_irq_clear(si522_dev *dev); //清中断
4.可直接对块进行读写的函数
/*
参数1：芯片结构体地址
参数2：块地址
参数3：写入数据
*/
int8_t si522a_user_write_block(si522_dev* dev, uint8_t ucAddr, uint8_t * pData); //写卡指定块的函数，o~64,卡有16个扇区，每个扇区4个块，块3为密码块，不可存储数据
/*
参数1：芯片结构体地址
参数2：块地址
参数3：读出的数据
*/
int8_t si522a_user_read_block(si522_dev* dev, uint8_t ucAddr, uint8_t * pData); //读卡指定块的函数，o~64,卡有16个扇区，每个扇区4个块，块3为密码块，不可存储数据
5.卡操作函数
int8_t si522a_pcd_isblock( uint8_t ucAddr ) //判断要写的是否为数据块
int8_t PcdHalt(si522_dev* dev) //使卡进入休眠模式
6.读卡测试函数
/*
参数1：芯片结构体地址
参数2：开片UID（输出）
*/
int8_t si522a_pcd_typea_getuid(si522a_dev* dev, uint8_t *pOut)                  //获取卡片UID


2.使用案例（nrf52840）
void main(void *arg)
{
    /*si522a片选引脚和复位引脚设置和初始化*/
    nrf_gpio_cfg_output(SI522A_CSN);
    nrf_gpio_pin_set(SI522A_CSN);
    nrf_gpio_cfg_output(SI522A_RST);
    nrf_gpio_pin_set(SI522A_RST);
    vTaskDelay(10);
    nrf_gpio_pin_set(SI522A_RST);
    // IRQ 中断
    nrfx_gpiote_in_config_t dio1_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    dio1_config.pull = GPIO_PIN_CNF_PULL_Pullup;
    APP_ERROR_CHECK(nrfx_gpiote_in_init(SI522A_IRQ, &dio1_config, nrfx_adc_irq_handler));
    nrfx_gpiote_in_event_enable(SI522A_IRQ, true);
    spi3_init();             // spi初始化
    struct SI522_DEV si522a; //创建设备结构体
 
    si522a_pcd_typea_init(&si522a); // Reader模式的初始化
    si522a_pcd_auto_calc(&si522a);      //自动获取0F_K寄存器和0F_C的阈值
    si522a_acd_start(&si522a);         //初始化ACD配置寄存器，并且进入ACD模式，检测到卡时自动退出ACD模式
    PCD_IRQ_flagA = 0;
    while (true)
    {
        if (1 == PCD_IRQ_flagA) // IRQ pin脚中断检测有中断信号产生
        {
            switch (PCD_IRQ(&si522a))
            {
            case 0:
                NRF_LOG_INFO("other irq");
                break;
            case 1:                          // acd中断
                unsigned char read_data[16]; //存储读出的块的数据
                unsigned char write_data[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                                0x08, 0x09, 0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5}; //要写入的数据，1块有16字节
                si522a_user_write_block(&si522a, 22, write_data);
                //块函数
                si522a_user_read_block(&si522a, 22, read_data);
                //读块函数
                break;
            case 2: // acd看门狗中断
                break;
            }
            si522a_pcd_typea_init(&si522a); // Reader模式的初始化
            si522a_pcd_auto_calc(&si522a);         //初始化ACD配置寄存器，并且进入ACD模式
            PCD_IRQ_flagA = 0;
        }
        else
        {
            vTaskDelay(500);
        }
    }
}


3.普通读写卡的操作步骤
1.寻卡
int8_t PcdRequest(si522_dev* dev, uint8_t req_code, uint8_t * pTagType )
2.防冲突
int8_t PcdAnticoll (si522_dev *dev, uint8_t *pSnr, uint8_t anticollision_level) //防冲突
3.选卡，用其中一个函数也可实现
int8_t PcdSelect (si522_dev* dev, uint8_t * pSnr, uint8_t *sak); //选卡
int8_t PcdSelect1 (si522_dev* dev, uint8_t * pSnr, uint8_t *sak);
int8_t PcdSelect2 (si522_dev* dev, uint8_t * pSnr, uint8_t *sak);
int8_t PcdSelect3 (si522_dev* dev, uint8_t * pSnr, uint8_t *sak);
4.校验卡密码
int8_t PcdAuthState(si522_dev* dev, uint8_t auth_mode,uint8_t addr,uint8_t *pKey,uint8_t *pSnr) //密码校验
5.读写块
int8_t PcdRead (si522_dev* dev, uint8_t ucAddr, uint8_t * pData ) //读卡指定块的数据
int8_t PcdWrite (si522_dev* dev, uint8_t ucAddr, uint8_t * pData ) //向卡里指定块写数据
4.直接读写块
int8_t I_si522a_write_block(si522_dev* dev, uint8_t ucAddr, uint8_t * pData); //写卡指定块的函数，o~64,卡有16个扇区，每个扇区4个块，块3为密码块，不可存储数据,返回值为写状态
int8_t I_si522a_read_block(si522_dev* dev, uint8_t ucAddr, uint8_t * pData); //读卡指定块的函数，o~64,卡有16个扇区，每个扇区4个块，块3为密码块，不可存储数据，返回值为写状态
5.SI512双向通信模式
1.SI512与SI522A芯片ACD模式和PCD模式驱动代码兼容，增加了模拟卡双向通信功能
1.发送与接收函数（输入输出均为16进制数）

/*
参数1：设备结构体
参数2：发送数据
参数3：发送数据长度
*/
int8_t si512_p2p_send_data(si522_dev *dev, uint8_t *w_data, uint8_t len)            //发送数据（最多可以传19字节，超出会出错，建议传16字节）
/*
参数1：设备结构体
参数2：接收数据（输出）
参数3：接收数据长度
*/
int8_t si512_p2p_rseceive_data(si522_dev *dev, uint8_t *r_data，uint8_t len)                     //接收数据
2.应用示例

void main (void *arg)
{
/*配置和初始化片选和复位引脚*/
  nrf_gpio_cfg_output(SI522A_CSN);
  nrf_gpio_pin_set(SI522A_CSN);
  nrf_gpio_cfg_output(SI522A_RST);
  nrf_gpio_pin_set(SI522A_RST);
  vTaskDelay(10);
  nrf_gpio_pin_set(SI522A_RST);
  // IRQ 中断配置和初始化
  nrfx_gpiote_in_config_t dio1_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  dio1_config.pull = GPIO_PIN_CNF_PULL_Pullup;
  APP_ERROR_CHECK(nrfx_gpiote_in_init(SI522A_IRQ, &dio1_config, nrfx_adc_irq_handler));
  nrfx_gpiote_in_event_enable(SI522A_IRQ, true);
 
  vTaskDelay(10);
  spi3_init();              //spi初始化
#if 1
  si522a_pcd_config_isotype(&si522a, 'H' );             //PN512_P2P_Initiator_Active
#else
  si522a_pcd_config_isotype(&si522a, 'I');                  //PN512_P2P_Target_Active
#endif
  while(true)
  {
#if 1   //作为数据发送方
    uint8_t send_buf[10] = {0xaa,0xbb,0xcc,0xdd,0xee,0x07,0x08,0x09,0x0a};
    si512_p2p_send_data(&si522a, send_buf, 10);
#else   //作为数据接收方
    uint8_t read_buf[64];
    int8_t status = si512_p2p_rseceive_data(&si522a, read_buf， 10);
    vTaskDelay (500);
#endif
  }
