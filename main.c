#include "main.h"   


char MBW;
unsigned char v_flag;     //场中断标志
unsigned char data_aroll [VDx][VDy];
unsigned char data_aroll_char [84][4];  
unsigned char k=0;//FIFO切换控制参数


signed int speed=0;     //速度设定参数

signed int  speed_x;//初始动力
signed int  speed_p = 60;//60;  //PID调速的比例系数
signed int  speed_i;
signed int  speed_now;
uchar speed_set=65;
uchar direction_speed=0;
unsigned char data_Vxy [15];
unsigned int data_line[3]; 
  uchar flg_diudao;
//*****************************************************************************************
unsigned char igonre;
INT8U PaTabel[8] = {0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60};
//*****************************************************************************************
void SpiInit(void);
void CpuInit(void);
void RESET_CC1100(void);
void POWER_UP_RESET_CC1100(void);
void halSpiWriteReg(INT8U addr, INT8U value);
void halSpiWriteBurstReg(INT8U addr, INT8U *buffer, INT8U count);
void halSpiStrobe(INT8U strobe);
INT8U halSpiReadReg(INT8U addr);
void halSpiReadBurstReg(INT8U addr, INT8U *buffer, INT8U count);
INT8U halSpiReadStatus(INT8U addr);
void halRfWriteRfSettings(void);
void halRfSendPacket(INT8U *txBuffer, INT8U size); 
INT8U halRfReceivePacket(INT8U *rxBuffer, INT8U *length); 
//*****************************************************************************************
// CC1100 STROBE, CONTROL AND STATUS REGSITER
#define CCxxx0_IOCFG2       0x00        // GDO2 output pin configuration
#define CCxxx0_IOCFG1       0x01        // GDO1 output pin configuration
#define CCxxx0_IOCFG0       0x02        // GDO0 output pin configuration
#define CCxxx0_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CCxxx0_SYNC1        0x04        // Sync word, high INT8U
#define CCxxx0_SYNC0        0x05        // Sync word, low INT8U
#define CCxxx0_PKTLEN       0x06        // Packet length
#define CCxxx0_PKTCTRL1     0x07        // Packet automation control
#define CCxxx0_PKTCTRL0     0x08        // Packet automation control
#define CCxxx0_ADDR         0x09        // Device address
#define CCxxx0_CHANNR       0x0A        // Channel number
#define CCxxx0_FSCTRL1      0x0B        // Frequency synthesizer control
#define CCxxx0_FSCTRL0      0x0C        // Frequency synthesizer control
#define CCxxx0_FREQ2        0x0D        // Frequency control word, high INT8U
#define CCxxx0_FREQ1        0x0E        // Frequency control word, middle INT8U
#define CCxxx0_FREQ0        0x0F        // Frequency control word, low INT8U
#define CCxxx0_MDMCFG4      0x10        // Modem configuration
#define CCxxx0_MDMCFG3      0x11        // Modem configuration
#define CCxxx0_MDMCFG2      0x12        // Modem configuration
#define CCxxx0_MDMCFG1      0x13        // Modem configuration
#define CCxxx0_MDMCFG0      0x14        // Modem configuration
#define CCxxx0_DEVIATN      0x15        // Modem deviation setting
#define CCxxx0_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CCxxx0_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CCxxx0_BSCFG        0x1A        // Bit Synchronization configuration
#define CCxxx0_AGCCTRL2     0x1B        // AGC control
#define CCxxx0_AGCCTRL1     0x1C        // AGC control
#define CCxxx0_AGCCTRL0     0x1D        // AGC control
#define CCxxx0_WOREVT1      0x1E        // High INT8U Event 0 timeout
#define CCxxx0_WOREVT0      0x1F        // Low INT8U Event 0 timeout
#define CCxxx0_WORCTRL      0x20        // Wake On Radio control
#define CCxxx0_FREND1       0x21        // Front end RX configuration
#define CCxxx0_FREND0       0x22        // Front end TX configuration
#define CCxxx0_FSCAL3       0x23        // Frequency synthesizer calibration
#define CCxxx0_FSCAL2       0x24        // Frequency synthesizer calibration
#define CCxxx0_FSCAL1       0x25        // Frequency synthesizer calibration
#define CCxxx0_FSCAL0       0x26        // Frequency synthesizer calibration
#define CCxxx0_RCCTRL1      0x27        // RC oscillator configuration
#define CCxxx0_RCCTRL0      0x28        // RC oscillator configuration
#define CCxxx0_FSTEST       0x29        // Frequency synthesizer calibration control
#define CCxxx0_PTEST        0x2A        // Production test
#define CCxxx0_AGCTEST      0x2B        // AGC test
#define CCxxx0_TEST2        0x2C        // Various test settings
#define CCxxx0_TEST1        0x2D        // Various test settings
#define CCxxx0_TEST0        0x2E        // Various test settings

// Strobe commands
#define CCxxx0_SRES         0x30        // Reset chip.
#define CCxxx0_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is
                                        // running (for quick RX / TX turnaround).
#define CCxxx0_SXOFF        0x32        // Turn off crystal oscillator.
#define CCxxx0_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).
#define CCxxx0_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.
#define CCxxx0_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        // Only go to TX if channel is clear.
#define CCxxx0_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.
#define CCxxx0_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CCxxx0_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CCxxx0_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CCxxx0_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CCxxx0_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CCxxx0_SWORRST      0x3C        // Reset real time clock.
#define CCxxx0_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                        // INT8Us for simpler software.
#define CCxxx0_PARTNUM      0x30
#define CCxxx0_VERSION      0x31
#define CCxxx0_FREQEST      0x32
#define CCxxx0_LQI          0x33
#define CCxxx0_RSSI         0x34
#define CCxxx0_MARCSTATE    0x35
#define CCxxx0_WORTIME1     0x36
#define CCxxx0_WORTIME0     0x37
#define CCxxx0_PKTSTATUS    0x38
#define CCxxx0_VCO_VC_DAC   0x39
#define CCxxx0_TXBYTES      0x3A
#define CCxxx0_RXBYTES      0x3B

#define CCxxx0_PATABLE      0x3E
#define CCxxx0_TXFIFO       0x3F
#define CCxxx0_RXFIFO       0x3F
// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct S_RF_SETTINGS
{
INT8U FSCTRL2;   //自已加的
    INT8U FSCTRL1;   // Frequency synthesizer control.
    INT8U FSCTRL0;   // Frequency synthesizer control.
    INT8U FREQ2;     // Frequency control word, high INT8U.
    INT8U FREQ1;     // Frequency control word, middle INT8U.
    INT8U FREQ0;     // Frequency control word, low INT8U.
    INT8U MDMCFG4;   // Modem configuration.
    INT8U MDMCFG3;   // Modem configuration.
    INT8U MDMCFG2;   // Modem configuration.
    INT8U MDMCFG1;   // Modem configuration.
    INT8U MDMCFG0;   // Modem configuration.
    INT8U CHANNR;    // Channel number.
    INT8U DEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
    INT8U FREND1;    // Front end RX configuration.
    INT8U FREND0;    // Front end RX configuration.
    INT8U MCSM0;     // Main Radio Control State Machine configuration.
    INT8U FOCCFG;    // Frequency Offset Compensation Configuration.
    INT8U BSCFG;     // Bit synchronization Configuration.
    INT8U AGCCTRL2; // AGC control.
INT8U AGCCTRL1; // AGC control.
    INT8U AGCCTRL0; // AGC control.
    INT8U FSCAL3;    // Frequency synthesizer calibration.
    INT8U FSCAL2;    // Frequency synthesizer calibration.
INT8U FSCAL1;    // Frequency synthesizer calibration.
    INT8U FSCAL0;    // Frequency synthesizer calibration.
    INT8U FSTEST;    // Frequency synthesizer calibration control
    INT8U TEST2;     // Various test settings.
    INT8U TEST1;     // Various test settings.
    INT8U TEST0;     // Various test settings.
    INT8U IOCFG2;    // GDO2 output pin configuration
    INT8U IOCFG0;    // GDO0 output pin configuration
    INT8U PKTCTRL1; // Packet automation control.
    INT8U PKTCTRL0; // Packet automation control.
    INT8U ADDR;      // Device address.
    INT8U PKTLEN;    // Packet length.
} RF_SETTINGS;
//*****************************************************************************************
const RF_SETTINGS rfSettings = 
{
0x00,
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x10,   // FREQ2     Frequency control word, high byte.
    0xA7,   // FREQ1     Frequency control word, middle byte.
    0x62,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x03,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
//*****************************************************************************************
    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end RX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2 AGC control.
    0x00,   // AGCCTRL1 AGC control.
    0xB2,   // AGCCTRL0 AGC control.
//*****************************************************************************************
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x11,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x06,   // IOCFG0D   GDO0 output pin configuration. Refer to SmartRF?Studio User Manual for detailed pseudo register explanation.
//*****************************************************************************************
    0x04,   // PKTCTRL1 Packet automation control.
    0x05,   // PKTCTRL0 Packet automation control.
    0x00,   // ADDR      Device address.
    0x0c    // PKTLEN    Packet length.
};
//*****************************************************************************************
//函数名：delay_CC(unsigned int s)
//输入：时间
//输出：无
//功能描述：普通廷时,内部用
//*****************************************************************************************  
static void delay_CC(unsigned int s)
{
unsigned int i;
for(i=0; i<s; i++);
for(i=0; i<s; i++);
}
//*****************************************************************************************
void halWait(INT16U timeout)
{
unsigned char i; 
    do {
        for(i = 0; i< 10; i++);
    } while (--timeout);
}
//*********************************SPI接口初始化*******************************************
void SpiInit(void)
{
DCSN = 1;
DSCK = 1;
DGDO0 = 0;
DMISO = 0;
DMOSI = 1;
DRXEN=1;
DTXEN=1;
CSN=0;
SCK=0;
CSN=1;
}
//****************************************************************************************
//函数名：CpuInit()
//输入：无
//输出：无
//功能描述：SPI初始化程序
/*****************************************************************************************/
void CpuInit(void)
{
SpiInit();
delay_CC(5000);
}
//*****************************************************************************************
//函数名：SpisendByte(INT8U dat)
//输入：发送的数据
//输出：无
//功能描述：SPI发送一个字节
//*****************************************************************************************
INT8U SpiTxRxByte(INT8U dat)
{
INT8U i,temp;
temp = 0; 
SCK = 0;
for(i=0; i<8; i++)
{
   if(dat & 0x80)
   {
    MOSI = 1;
   }
   else MOSI = 0;
   dat <<= 1;

   SCK = 1; 
   asm("nop");
   asm("nop");

   temp <<= 1;
   if(MISO)temp++; 
   SCK = 0;
   asm("nop");
   asm("nop");
}
return temp;
}
//*****************************************************************************************
//函数名：void RESET_CC1100(void)
//输入：无
//输出：无
//功能描述：复位CC1100
//*****************************************************************************************
void RESET_CC1100(void) 
{
unsigned char i;
CSN = 0; 
while (MISO);
    i = SpiTxRxByte(CCxxx0_SRES);    //写入复位命令
while (MISO); 
    CSN = 1; 
}
//*****************************************************************************************
//函数名：void POWER_UP_RESET_CC1100(void) 
//输入：无
//输出：无
//功能描述：上电复位CC1100
//*****************************************************************************************
void POWER_UP_RESET_CC1100(void) 
{
CSN = 1; 
halWait(1); 
CSN = 0; 
halWait(1); 
CSN = 1; 
halWait(41); 
RESET_CC1100();      //复位CC1100
}
//*****************************************************************************************
//函数名：void halSpiWriteReg(INT8U addr, INT8U value)
//输入：地址和配置字
//输出：无
//功能描述：SPI写寄存器
//*****************************************************************************************
void halSpiWriteReg(INT8U addr, INT8U value) 
{
    CSN = 0;
    while (MISO);
    igonre = SpiTxRxByte(addr);   //写地址
    igonre = SpiTxRxByte(value);   //写入配置
    CSN = 1;
}
//*****************************************************************************************
//函数名：void halSpiWriteBurstReg(INT8U addr, INT8U *buffer, INT8U count)
//输入：地址，写入缓冲区，写入个数
//输出：无
//功能描述：SPI连续写配置寄存器
//*****************************************************************************************
void halSpiWriteBurstReg(INT8U addr, INT8U *buffer, INT8U count) 
{
    INT8U i, temp;
temp = addr | WRITE_BURST;
    CSN = 0;
    while (MISO);
    igonre = SpiTxRxByte(temp);
    for (i = 0; i < count; i++)
   {
        igonre = SpiTxRxByte(buffer[i]);
    }
    CSN = 1;
}
//*****************************************************************************************
//函数名：void halSpiStrobe(INT8U strobe)
//输入：命令
//输出：无
//功能描述：SPI写命令
//*****************************************************************************************
void halSpiStrobe(INT8U strobe) 
{
    CSN = 0;
    while (MISO);
    igonre = SpiTxRxByte(strobe);   //写入命令
    CSN = 1;
}
//*****************************************************************************************
//函数名：INT8U halSpiReadReg(INT8U addr)
//输入：地址
//输出：该寄存器的配置字
//功能描述：SPI读寄存器
//*****************************************************************************************
INT8U halSpiReadReg(INT8U addr) 
{
INT8U temp, value;
    temp = addr|READ_SINGLE;//读寄存器命令
CSN = 0;
while (MISO);
igonre = SpiTxRxByte(temp);
value = SpiTxRxByte(0);
CSN = 1;
return value;
}
//*****************************************************************************************
//函数名：void halSpiReadBurstReg(INT8U addr, INT8U *buffer, INT8U count)
//输入：地址，读出数据后暂存的缓冲区，读出配置个数
//输出：无
//功能描述：SPI连续读配置寄存器
//*****************************************************************************************
void halSpiReadBurstReg(INT8U addr, INT8U *buffer, INT8U count) 
{
    INT8U i,temp;
temp = addr | READ_BURST;   //写入要读的配置寄存器地址和读命令
    CSN = 0;
    while (MISO);
igonre = SpiTxRxByte(temp);   
    for (i = 0; i < count; i++) 
{
        buffer[i] = SpiTxRxByte(0);
    }
    CSN = 1;
}
//*****************************************************************************************
//函数名：INT8U halSpiReadReg(INT8U addr)
//输入：地址
//输出：该状态寄存器当前值
//功能描述：SPI读状态寄存器
//*****************************************************************************************
INT8U halSpiReadStatus(INT8U addr) 
{
    INT8U value,temp;
temp = addr | READ_BURST;   //写入要读的状态寄存器的地址同时写入读命令
    CSN = 0;
    while (MISO);
    igonre = SpiTxRxByte(temp);
value = SpiTxRxByte(0);
CSN = 1;
return value;
}
//*****************************************************************************************
//函数名：void halRfWriteRfSettings(RF_SETTINGS *pRfSettings)
//输入：无
//输出：无
//功能描述：配置CC1100的寄存器
//*****************************************************************************************
void halRfWriteRfSettings(void) 
{

halSpiWriteReg(CCxxx0_FSCTRL0, rfSettings.FSCTRL2);//自已加的
    // Write register settings
    halSpiWriteReg(CCxxx0_FSCTRL1, rfSettings.FSCTRL1);
    halSpiWriteReg(CCxxx0_FSCTRL0, rfSettings.FSCTRL0);
    halSpiWriteReg(CCxxx0_FREQ2,    rfSettings.FREQ2);
    halSpiWriteReg(CCxxx0_FREQ1,    rfSettings.FREQ1);
    halSpiWriteReg(CCxxx0_FREQ0,    rfSettings.FREQ0);
    halSpiWriteReg(CCxxx0_MDMCFG4, rfSettings.MDMCFG4);
    halSpiWriteReg(CCxxx0_MDMCFG3, rfSettings.MDMCFG3);
    halSpiWriteReg(CCxxx0_MDMCFG2, rfSettings.MDMCFG2);
    halSpiWriteReg(CCxxx0_MDMCFG1, rfSettings.MDMCFG1);
    halSpiWriteReg(CCxxx0_MDMCFG0, rfSettings.MDMCFG0);
    halSpiWriteReg(CCxxx0_CHANNR,   rfSettings.CHANNR);
    halSpiWriteReg(CCxxx0_DEVIATN, rfSettings.DEVIATN);
    halSpiWriteReg(CCxxx0_FREND1,   rfSettings.FREND1);
    halSpiWriteReg(CCxxx0_FREND0,   rfSettings.FREND0);
    halSpiWriteReg(CCxxx0_MCSM0 ,   rfSettings.MCSM0 );
    halSpiWriteReg(CCxxx0_FOCCFG,   rfSettings.FOCCFG);
    halSpiWriteReg(CCxxx0_BSCFG,    rfSettings.BSCFG);
    halSpiWriteReg(CCxxx0_AGCCTRL2, rfSettings.AGCCTRL2);
   halSpiWriteReg(CCxxx0_AGCCTRL1, rfSettings.AGCCTRL1);
    halSpiWriteReg(CCxxx0_AGCCTRL0, rfSettings.AGCCTRL0);
    halSpiWriteReg(CCxxx0_FSCAL3,   rfSettings.FSCAL3);
   halSpiWriteReg(CCxxx0_FSCAL2,   rfSettings.FSCAL2);
   halSpiWriteReg(CCxxx0_FSCAL1,   rfSettings.FSCAL1);
    halSpiWriteReg(CCxxx0_FSCAL0,   rfSettings.FSCAL0);
    halSpiWriteReg(CCxxx0_FSTEST,   rfSettings.FSTEST);
    halSpiWriteReg(CCxxx0_TEST2,    rfSettings.TEST2);
    halSpiWriteReg(CCxxx0_TEST1,    rfSettings.TEST1);
    halSpiWriteReg(CCxxx0_TEST0,    rfSettings.TEST0);
    halSpiWriteReg(CCxxx0_IOCFG2,   rfSettings.IOCFG2);
    halSpiWriteReg(CCxxx0_IOCFG0,   rfSettings.IOCFG0);    
    halSpiWriteReg(CCxxx0_PKTCTRL1, rfSettings.PKTCTRL1);
    halSpiWriteReg(CCxxx0_PKTCTRL0, rfSettings.PKTCTRL0);
    halSpiWriteReg(CCxxx0_ADDR,     rfSettings.ADDR);
    halSpiWriteReg(CCxxx0_PKTLEN,   rfSettings.PKTLEN);
}
//*****************************************************************************************
//函数名：void halRfSendPacket(INT8U *txBuffer, INT8U size)
//输入：发送的缓冲区，发送数据个数
//输出：无
//功能描述：CC1100发送一组数据
//*****************************************************************************************

void halRfSendPacket(INT8U *txBuffer, INT8U size) 
{
halSpiWriteReg(CCxxx0_TXFIFO, size);
    halSpiWriteBurstReg(CCxxx0_TXFIFO, txBuffer, size); //写入要发送的数据

    halSpiStrobe(CCxxx0_STX);   //进入发送模式发送数据

    // Wait for GDO0 to be set -> sync transmitted
    while (!GDO0);
    // Wait for GDO0 to be cleared -> end of packet
    while (GDO0);
halSpiStrobe(CCxxx0_SFTX);
}
//*****************************************************************************************
void setRxMode(void)
{
    halSpiStrobe(CCxxx0_SRX);   //进入接收状态
}
//*****************************************************************************************
INT8U halRfReceivePacket(INT8U *rxBuffer, INT8U *length) 
{
    INT8U status[2];
    INT8U packetLength;
INT8U i=(*length)*4; // 具体多少要根据datarate和length来决定
    TXEN=0;RXEN=0;
    halSpiStrobe(CCxxx0_SRX);   //进入接收状态
delay_CC(2);
while (GDO0)
{
   delay_CC(2);
   --i;
   if(i<1)
     TXEN=0;RXEN=0;
     return 0;      
} 
    if ((halSpiReadStatus(CCxxx0_RXBYTES) & BYTES_IN_RXFIFO)) //如果接的字节数不为0
{
        packetLength = halSpiReadReg(CCxxx0_RXFIFO);//读出第一个字节，此字节为该帧数据长度
        if (packetLength <= *length)    //如果所要的有效数据长度小于等于接收到的数据包的长度
   {
            halSpiReadBurstReg(CCxxx0_RXFIFO, rxBuffer, packetLength); //读出所有接收到的数据
            *length = packetLength;     //把接收数据长度的修改为当前数据的长度
            halSpiReadBurstReg(CCxxx0_RXFIFO, status, 2); //读出CRC校验位
    halSpiStrobe(CCxxx0_SFRX);   //清洗接收缓冲区
            TXEN=0;RXEN=0;
            return (status[1] & CRC_OK);    //如果校验成功返回接收成功
        }
   else 
   {
            *length = packetLength;
            halSpiStrobe(CCxxx0_SFRX);   //清洗接收缓冲区
            TXEN=0;RXEN=0;
            return 0;
        }
    } 
else
   TXEN=0;RXEN=0;
   return 0;
}

//*****************************************************************************************



void MCU_init(void); /* Device initialization function declaration */

void main(void) {  
// uchar tmp;
signed int dir;
signed int dir_tmp;
uchar i,j;

INT8U leng =0;
INT8U tf =0 , count0 = 0;
INT8U TxBuf[8]={0};    // 8字节, 如果需要更长的数据包,请正确设置
INT8U RxBuf[8]={0};
                     
 signed int spd=0;
 

  MCU_init(); /* call Device Initialization */  
  EnableInterrupts;   
  PACTL=0x40;//脉冲累加器控制寄存器设置   

  
  LCD_init();
  camera_init(); 
  
 // PWMDTY23=100; //0～100  控制电机 PP3 
  //PWMDTY67=0;
//for(;;);
  
 TOP_prog:                                                            
  PIT0_C=0; //PIT0 disable
  //TIE1_C=0; //PT1  disable
  
/*CC1100 初始化 
 CpuInit();
POWER_UP_RESET_CC1100(); //1100复位
halRfWriteRfSettings();   //寄存其配置
EnableInterrupts;
halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
 */

 
  //TXEN=0;RXEN=0;
  delay(6000);
  
 // TIE1_C=1; //PT1  enable
  DDRB_DDRB0=1;
  direction_speed=0;
  speed=0;
  PIT0_C=1;
  keys_init();
 // tmp=0;
  delay(0xFFFF);
  for(;KEY4==1;){
    delay(0xafff);
    getasimpleroll();
    //VDtoLCD2();
    
   for(j=0;j<3;j++){ 
    for(i=0;i<5;i++){
      LCD_write_num(2,i*15,j,data_Vxy[(j*5)+i]);
    }
   }
    
    /*LCD_write_num(2,30,5,data_line[2]);
    LCD_write_num(2,15,5,data_line[1]);
    LCD_write_num(2,0,5,data_line[0]); */
    //LCD_write_num(2,32,0,(char)(speed%100));
    //LCD_write_num(2,20,0,(char)(speed/100));
    //LCD_write_num(1,0,0,direction_speed);
    if(KEY2!=1){      
      direction_speed=1;
      speed++;
      delay(0xaFFF);
    }
    if(KEY3!=1){
      speed=0;
      direction_speed=0;
    }
      
    if(KEY4!=1){
       //set_direction(-140);
      direction_speed=1;
      speed--;
      delay(0xaFFF);
    }
    if(KEY1!=1){
      //if(direction_speed==1){
     //   direction_speed=2;
     //   continue;
     // }
      //if(direction_speed!=1)direction_speed=1;//setspeed( 0,1);

      
    }
  }
  LCD_clear();
  delay(0xFFFF);
  for(;KEY4==1;){
    delay(0xFfff);
    getasimpleroll();
    if(flg_diudao==0){  //不丢道的情况
      j=0;
      data_line[0]=0;
      for(i=0;i<5;i++){
        if(data_Vxy[i]!=0){
          data_line[0]+=data_Vxy[i];
          j++;
        }
      }
      data_line[0]=data_line[0]/j;
      j=0;
      data_line[1]=0;
      for(i=5;i<10;i++){
        if(data_Vxy[i]!=0){
          data_line[1]+=data_Vxy[i];
          j++;
        }
      }
      data_line[1]=data_line[1]/j;
      j=0;
      data_line[2]=0;
      for(i=10;i<15;i++){
        if(data_Vxy[i]!=0){
          data_line[2]+=data_Vxy[i];
          j++;
        }
      }
      data_line[2]=data_line[2]/j;
      
     /* dir_tmp=data_line[2]-data_line[0];
      
      
     
      dir=3; //dir基数
      if(dir_tmp<0){
        LCD_write_english_string(50,5,"-");
        dir_tmp=(-1)*dir_tmp;
        dir=-3;
      }
      LCD_write_num(2,56,5,dir_tmp); 
            
      for(i=1;i<dir_tmp;i++){
        dir=dir*2;
      }
      
      
      set_direction(dir);
      */
      dir=(data_line[1]-22);
      if(dir>(-5)&&dir<5)set_direction(dir*5);
      else set_direction(dir*8);
      
      
 
      LCD_write_num(2,30,5,data_line[2]);
      LCD_write_num(2,15,5,data_line[1]);
      LCD_write_num(2,0,5,data_line[0]);
      for(j=0;j<3;j++){ 
        for(i=0;i<5;i++){
          LCD_write_num(2,i*15,j,data_Vxy[(j*5)+i]);
        }
      }
    }
    
    if(flg_diudao==1){    //丢道的情况
      LCD_clear();
      set_direction(dir*12);
    }
  }
  LCD_clear();
  delay(0xFFFF);
  
  for(;KEY4==1;){
    if(KEY1!=1){      

      speed_set++;
      delay(0xaFFF);
    }
    if(KEY2!=1){
      speed_set--;
      delay(0xaFFF);
    }   
    if(KEY3!=1){
      speed_set=60;
      delay(0xaFFF);
    } 
    LCD_write_num(3,0,0,speed_set);
    
  }


  LCD_clear();
  delay(0xFFFF);
  PIT0_C=1;
  for(;;){
    getasimpleroll();
    if(flg_diudao==0){  //不丢道的情况
      j=0;
      data_line[0]=0;
      for(i=0;i<5;i++){
        if(data_Vxy[i]!=0){
          data_line[0]+=data_Vxy[i];
          j++;
        }
      }
      data_line[0]=data_line[0]/j;
      j=0;
      data_line[1]=0;
      for(i=5;i<10;i++){
        if(data_Vxy[i]!=0){
          data_line[1]+=data_Vxy[i];
          j++;
        }
      }
      data_line[1]=data_line[1]/j;
      j=0;
      data_line[2]=0;
      for(i=10;i<15;i++){
        if(data_Vxy[i]!=0){
          data_line[2]+=data_Vxy[i];
          j++;
        }
      }
      data_line[2]=data_line[2]/j;

      dir=(data_line[0]-22);
      if(dir>(-7)&&dir<7)set_direction(dir*5);
      else set_direction(dir*7);
      
   
    }
    
    if(flg_diudao==1){    //丢道的情况
      //set_direction(dir*12);
    }
    direction_speed=1;
    speed=speed_set-(speed_set*(dir/50));
  }

  //point=0;
  //goto TOP_prog;
  /*
  for(;;){

     if(direction>  110 )direction=110;
     if(direction<(-110))direction=-110;
     set_direction(direction);
     if(KEY1!=1){
     }
     PITINTE_PINTE0=1;//开启调速中
  setspeed(0,0);
  godirection=0;
  */
  //
 
  /*   setspeed(5000,-1);
       PWM3 = 0; 
      PWM4 = 5000;
     delay(0xafff);
  for(;;){
     TXEN=0;RXEN=1; 
     if(halRfReceivePacket(RxBuf,&leng)){  
         
         set_direction((RxBuf[1]-82)*4);       
         spd=(RxBuf[2]-158)*8;
      if(spd<32&&spd>(-32)){
          speed=0;
          setspeed(0,0);
          PIT0_C=0;
          speed_x=4000;//初始动力
          speed_i=0; 
      } 
      if(spd>=32||spd<=(-32)){
          PIT0_C=1;//开启调速中      
          speed=spd*3; 
                
      }
        
     //// if(){
        // speed=-tmmp;
        // godirection=2;
     // }
     // if((spd>(-18))&&(spd<18)){
       //  PITINTE_PINTE0=0;//关调速中    
        // speed=0;
        // speed_x=2000;
     // }     
      PORTB=~RxBuf[1]; 
      delay(1000);
     }
     
     
     TXEN=0;RXEN=0;       
  }
  */   
}
  

