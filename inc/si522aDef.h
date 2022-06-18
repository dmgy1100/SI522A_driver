#define  RFCfgReg_Val  0x68
#define  DivIEnReg_Val  0xC0
#define  ComIEnReg_Val  0x80
#define  IRQMODE  0x01
#define  EXTI_Trigger_Mode  0x0C
#define  ACDConfigRegA_Val  0x02
#define  ACDConfigRegB_Val  0xA8
#define  ACDConfigRegD_Val  0x08
#define  ACDConfigRegH_Val  0xff
#define  ACDConfigRegI_Val  0x00
#define  ACDConfigRegJ_Val  0x44                //0x44
#define  ACDConfigRegM_Val  0x01
#define  ACDConfigRegO_Val  0x0a


//********************************************//
//MF522寄存器定义
//********************************************//
//Page 0: Command and status
#define    RFU00                0x00
#define    CommandReg           0x01
#define    ComIEnReg            0x02
#define    DivIEnReg            0x03
#define    ComIrqReg            0x04
#define    DivIrqReg            0x05
#define    ErrorReg             0x06
#define    Status1Reg           0x07
#define    Status2Reg           0x08
#define    FIFODataReg          0x09
#define    FIFOLevelReg         0x0A
#define    WaterLevelReg        0x0B
#define    ControlReg           0x0C
#define    BitFramingReg        0x0D
#define    CollReg              0x0E
#define    ACDConfigReg         0x0F

//Page 1: Command
#define    RFU10                0x10
#define    ModeReg              0x11
#define    TxModeReg            0x12
#define    RxModeReg            0x13
#define    TxControlReg         0x14
#define    TxASKReg             0x15
#define    TxSelReg             0x16
#define    RxSelReg             0x17
#define    RxThresholdReg       0x18
#define    DemodReg             0x19
#define    Fel1Reg              0x1A
#define    FelNFC2Reg           0x1B
#define    MfTxReg              0x1C
#define    MfRxReg              0x1D
#define    TypeBReg             0x1E
#define    SerialSpeedReg       0x1F

// Page 2: Configuration
#define    ACDConfigSelReg      0x20
#define    CRCResultRegH        0x21
#define    CRCResultRegL        0x22
#define    RFU23                0x23
#define    ModWidthReg          0x24
#define    RFU25                0x25
#define    RFCfgReg             0x26
#define    GsNReg               0x27
#define    CWGsPReg             0x28
#define    ModGsPReg            0x29
#define    TModeReg             0x2A
#define    TPrescalerReg        0x2B
#define    TReloadRegH          0x2C
#define    TReloadRegL          0x2D
#define    TCounterValueRegH    0x2E
#define    TCounterValueRegL    0x2F

//Page 3: Test Register
#define    RFU30                0x30
#define    TestSel1Reg          0x31
#define    TestSel2Reg          0x32
#define    TestPinEnReg         0x33
#define    TestPinValueReg      0x34
#define    TestBusReg           0x35
#define    AutoTestReg          0x36
#define    VersionReg           0x37
#define    AnalogTestReg        0x38
#define    TestDAC1Reg          0x39
#define    TestDAC2Reg          0x3A
#define    TestADCReg           0x3B
#define    RFU3C                0x3C
#define    RFU3D                0x3D
#define    RFU3E                0x3E
#define    RFU3F                0x3F



/////////////////////////////////////////////////////////////////////
//MF522命令字
/////////////////////////////////////////////////////////////////////
#define PCD_IDLE              0x00               //取消当前命令
#define PCD_AUTHENT           0x0E               //验证密钥
#define PCD_RECEIVE           0x08               //接收数据
#define PCD_TRANSMIT          0x04               //发送数据
#define PCD_TRANSCEIVE        0x0C               //发送并接收数据
#define PCD_RESETPHASE        0x0F               //复位
#define PCD_CALCCRC           0x03               //CRC计算


/////////////////////////////////////////////////////////////////////
//和MF522通讯时返回的错误代码
/////////////////////////////////////////////////////////////////////
#define   MI_OK                 0
#define   MI_NOTAGERR           1
#define   MI_ERR                2

#define MI_CHK_FAILED                   (2)
#define MI_CRCERR                       (2)
#define MI_CHK_COMPERR                  (2)
#define MI_EMPTY                        (3)
#define MI_AUTHERR                      (4)
#define MI_PARITYERR                    (5)
#define MI_CODEERR                      (6)
#define MI_SERNRERR                     (8)
#define MI_KEYERR                       (9)
#define MI_NOTAUTHERR                   (10)
#define MI_BITCOUNTERR                  (11)
#define MI_BYTECOUNTERR                 (12)
#define MI_IDLE                         (13)
#define MI_TRANSERR                     (14)
#define MI_WRITEERR                     (15)
#define MI_INCRERR                      (16)
#define MI_DECRERR                      (17)
#define MI_READERR                      (18)
#define MI_OVFLERR                      (19)
#define MI_POLLING                      (20)
#define MI_FRAMINGERR                   (21)
#define MI_ACCESSERR                    (22)
#define MI_UNKNOWN_COMMAND              (23)
#define MI_COLLERR                      (24)
#define MI_RESETERR                     (25)
#define MI_INITERR                      (25)
#define MI_INTERFACEERR                 (26)
#define MI_ACCESSTIMEOUT                (27)
#define MI_NOBITWISEANTICOLL            (28)
#define MI_QUIT                         (30)
#define MI_INTEGRITY_ERR                (35)
#define MI_RECBUF_OVERFLOW              (50)
#define MI_SENDBYTENR                   (51)
#define MI_SENDBUF_OVERFLOW             (53)
#define MI_BAUDRATE_NOT_SUPPORTED       (54)
#define MI_SAME_BAUDRATE_REQUIRED       (55)
#define MI_WRONG_PARAMETER_VALUE        (60)
#define MI_BREAK                        (99)
#define MI_NY_IMPLEMENTED               (100)
#define MI_NO_MFRC                      (101)
#define MI_MFRC_NOTAUTH                 (102)
#define MI_WRONG_DES_MODE               (103)
#define MI_HOST_AUTH_FAILED             (104)
#define MI_WRONG_LOAD_MODE              (106)
#define MI_WRONG_DESKEY                 (107)
#define MI_MKLOAD_FAILED                (108)
#define MI_FIFOERR                      (109)
#define MI_WRONG_ADDR                   (110)
#define MI_DESKEYLOAD_FAILED            (111)
#define MI_WRONG_SEL_CNT                (114)
#define MI_WRONG_TEST_MODE              (117)
#define MI_TEST_FAILED                  (118)
#define MI_TOC_ERROR                    (119)
#define MI_COMM_ABORT                   (120)
#define MI_INVALID_BASE                 (121)
#define MI_MFRC_RESET                   (122)
#define MI_WRONG_VALUE                  (123)
#define MI_VALERR                       (124)
#define MI_COM_ERR                      (125)
#define PROTOCOL_ERR                    (126)

/////////////////////////////////////////////////////////////////////
//MF522 FIFO长度定义
/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH       64                 //FIFO size=64byte
#define MAXRLEN  18

/////////////////////////////////////////////////////////////////////
//Mifare_One卡片命令字
/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL           0x26               //寻天线区内未进入休眠状态
#define PICC_REQALL           0x52               //寻天线区内全部卡
#define PICC_ANTICOLL1        0x93               //防冲撞
#define PICC_ANTICOLL2        0x95               //防冲撞
#define PICC_ANTICOLL3        0x97               //防冲撞

#define PICC_AUTHENT1A        0x60               //验证A密钥
#define PICC_AUTHENT1B        0x61               //验证B密钥
#define PICC_READ             0x30               //读块
#define PICC_WRITE            0xA0               //写块
#define PICC_DECREMENT        0xC0               //扣款
#define PICC_INCREMENT        0xC1               //充值
#define PICC_RESTORE          0xC2               //调块数据到缓冲区
#define PICC_TRANSFER         0xB0               //保存缓冲区中数据
#define PICC_HALT             0x50               //休眠

/*卡的类型*/
#define Mifare_UltraLight     0x4400
#define Mifare_S50            0x0400
#define Mifare_S70            0x0200
#define Mifare_Pro            0x0800
#define Mifare_DESFire        0x4403

#define ACDConfigA       0x00
#define ACDConfigB       0x01
#define ACDConfigC       0x02
#define ACDConfigD       0x03
#define ACDConfigE       0x04
#define ACDConfigF       0x05
#define ACDConfigG       0x06
#define ACDConfigH       0x07
#define ACDConfigI       0x08
#define ACDConfigJ       0x09
#define ACDConfigK       0x0a
#define ACDConfigL       0x0b
#define ACDConfigM       0x0c
#define ACDConfigN       0x0d
#define ACDConfigO       0x0e
#define ACDConfigP       0x0f
//ACD模式相关定义
#define BIT7  0X80
#define BIT6  0X40
#define BIT5  0X20
#define BIT4  0X10
#define BIT3  0X08
#define BIT2  0X04
#define BIT1  0X02
#define BIT0  0X01


#define ISO14443B_ANTICOLLISION                  0x05
#define ISO14443B_ATTRIB                         0x1D
#define ISO14443B_HLTB                           0x50
#define FSDI 8

