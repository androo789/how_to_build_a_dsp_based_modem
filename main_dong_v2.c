/****************************************************************************/
/*                                                                          */
/*                                                          */
/*                     用delay函数输出spi，控制da输出正弦波
 * 甚至去掉delay，在一个死循环里面，不断输出spi，                                                 */
/****************************************************************************/
// 注意：DSP ports, Shared RAM, UART0, EDMA, SPI0, MMC/SDs,
//       VPIF, LCDC, SATA, uPP, DDR2/mDDR (bus ports), USB2.0, HPI, PRU
//       这些外设使用的时钟来源为 PLL0_SYSCLK2 默认频率为 CPU 频率的二分之一
//       但是，ECAPs, UART1/2, Timer64P2/3, eHRPWMs,McBSPs, McASP0, SPI1
//       这些外设的时钟来源可以在 PLL0_SYSCLK2 和 PLL1_SYSCLK2 中选择
//       通过修改 System Configuration (SYSCFG) Module
//       寄存器 Chip Configuration 3 Register (CFGCHIP3) 第四位 ASYNC3_CLKSRC
//       配置时钟来源
//       （默认值） 0 来源于 PLL0_SYSCLK2
//                  1 来源于 PLL1_SYSCLK2
//       如果不是为了降低功耗，不建议修改这个值，它会影响所有相关外设的时钟频率




#include "hw_syscfg0_C6748.h"       // 系统配置模块寄存器
#include "gpio.h"                   // 通用输入输出口宏及设备抽象层函数声明
#include "timer.h"                  // 通用输入输出口宏及设备抽象层函数声明
#include "interrupt.h"              // DSP C6748 中断相关应用程序接口函数声明及系统事件号定义
#include "TL6748.h"                 // 创龙 DSP6748 开发板相关声明
#include "hw_types.h"				 // 宏命令
#include "soc_C6748.h"			     // DSP C6748 外设寄存器
#include "psc.h"                    // 电源与睡眠控制宏及设备抽象层函数声明
#include "spi.h"                    // 串行外设接口宏及设备抽象层函数声明
#include "uartStdio.h"              // 串口标准输入输出终端函数声明
#include <string.h>
#include "dongqi.h"
/****************************************************************************/
/*                                                                          */
/*              宏定义                                                      */
/*                                                                          */
/****************************************************************************/
// 软件断点
#define SW_BREAKPOINT     asm(" SWBP 0 ");

#define spishizhong  30000000

/****************************************************************************/
/*                                                                          */
/*              全局变量                                                    */
/*                                                                          */
/****************************************************************************/
int dongflag = 0;

/****************************************************************************/
/*                                                                          */
/*              宏定义                                                      */
/*                                                                          */
/****************************************************************************/
// 字符长度
#define CHAR_LENGTH             8
// XXX 整个程序看不太懂，也不是太明白，怎么下手去查明白他？？！？
/****************************************************************************/
/*                                                                          */
/*              全局变量                                                    */
/*                                                                          */
/****************************************************************************/

unsigned int tx_len;
unsigned int rx_len;
unsigned char tx_data[256];
unsigned char rx_data[256];
unsigned char *p_tx;
unsigned char *p_rx;

// DAC 寄存器组
struct
{
	volatile short               Data;

    struct
    {
    	volatile unsigned char  Addr :3;		// 0-2
    	volatile unsigned char  REG  :3;		// 3-6
    	volatile unsigned char  ZERO :1;		// 7
    	volatile unsigned char  RW   :1;		// 8
    }CFG;
}DACReg;

//unsigned short         sine_wave[100];


/****************************************************************************/
/*                                                                          */
/*              函数声明                                                    */
/*                                                                          */
/****************************************************************************/
// 外设使能配置
void PSCInit(void);

// GPIO 管脚复用配置
void GPIOBankPinMuxSet();
// GPIO 管脚初始化
void GPIOBankPinInit();


void SPIInit(void);
void DACOutput(unsigned short tongdaoashuju);
void SPIDataFormatConfig(unsigned int dataFormat);
int SpiTransfer(void);
void DACInit(void);

void delay(volatile unsigned int delay);

//int k=0;

/****************************************************************************/
/*                                                                          */
/*              主函数                                                      */
/*                                                                          */
/****************************************************************************/
int main(void)
{


	int i;
	for ( i = 0; i < 16; i++)//xxx  根据dac取高4位修改，原始数组
	{
	zhengxianbo16[i] = zhengxianbo16[i]<< 4;
	}
	for ( i = 0; i < 2; i++)
	{
		lingyiceshi[i] = lingyiceshi[i]<< 4;
	}

	// 外设使能配置
	PSCInit();
	
	// GPIO 管脚复用配置
	GPIOBankPinMuxSet();

	// GPIO 管脚初始化  初始化为out模式
	GPIOBankPinInit();

	// SPI 初始化
	SPIInit();

	// DAC 初始化
	DACInit();


		// 拉低 LDAC 管脚
	    GPIOPinWrite(SOC_GPIO_0_REGS, 111, GPIO_PIN_LOW);
		// DAC寄存器 VoutA = 3V
		DACReg.CFG.RW = 0;              // 写
		DACReg.CFG.ZERO = 0;            // 恒为 0
		DACReg.CFG.REG = 0;             // 寄存器选择 DAC 寄存器
		DACReg.CFG.Addr = 4;            // 通道选择   A通道


	// 主循环
	for(;;)
	{

			if(dongflag >= 2) dongflag=0;
		    DACOutput(lingyiceshi[dongflag]);// 输出电压值
		    dongflag++;
//			DACOutput(0xf000);
//		    delay(5000000);
//		    DACOutput(0x0f00);
//		    delay(5000000);
//		    DACOutput(0x00f0);
//		    delay(5000000);
//		    DACOutput(0x000f);
//		    delay(5000000);

	}
}

/****************************************************************************/
/*                                                                          */
/*              PSC 初始化                                                  */
/*                                                                          */
/****************************************************************************/
void PSCInit(void)
{//
	// 使能 GPIO 模块
	// 对相应外设模块的使能也可以在 BootLoader 中完成
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
    // 使能 SPI 模块
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_SPI1, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
}

/****************************************************************************/
/*                                                                          */
/*              GPIO 管脚复用配置                                           */
/*                                                                          */
/****************************************************************************/
void GPIOBankPinMuxSet(void)
{
	// 配置相应的 GPIO 口功能为普通输入输出口
	// 核心板 LED
	GPIOBank6Pin12PinMuxSetup();
	GPIOBank6Pin13PinMuxSetup();

	SPIPinMuxSetup(1);
	SPI1CSPinMuxSetup(2);
	// XXX 参考TL5724-A多通道DA模块规格书和TL138_1808_6748-EVM-A3底板原理图
	// 就可以看到da模块用的是spi1，用的是cs2片选

	volatile unsigned int savePinMux = 0;

	savePinMux = HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) & ~(SYSCFG_PINMUX13_PINMUX13_7_4);
	HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) = ((SYSCFG_PINMUX13_PINMUX13_7_4_GPIO6_14 << SYSCFG_PINMUX13_PINMUX13_7_4_SHIFT) | savePinMux);
	//GPIO6_14配置为普通gpio口

}

/****************************************************************************/
/*                                                                          */
/*              GPIO 管脚初始化                                             */
/*                                                                          */
/****************************************************************************/
void GPIOBankPinInit(void)
{
	// 配置 LED 对应管脚为输出管脚
    // OMAPL138 及 DSP C6748 共有 144 个 GPIO
	// 以下为各组 GPIO BANK 起始管脚对应值
    // 范围 1-144
	// GPIO0[0] 1
    // GPIO1[0] 17
	// GPIO2[0] 33
    // GPIO3[0] 49
	// GPIO4[0] 65
    // GPIO5[0] 81
	// GPIO6[0] 97
	// GPIO7[0] 113
	// GPIO8[0] 129

	// 核心板 LED
    GPIODirModeSet(SOC_GPIO_0_REGS, 109, GPIO_DIR_OUTPUT);  // GPIO6[12]
    GPIODirModeSet(SOC_GPIO_0_REGS, 110, GPIO_DIR_OUTPUT);  // GPIO6[13]
}

/****************************************************************************/
/*                                                                          */
/*              配置 SPI 数据格式                                           */
/*                                                                          */
/****************************************************************************/
void SPIDataFormatConfig(unsigned int dataFormat)
{
    // 配置 SPI 时钟
    SPIConfigClkFormat(SOC_SPI_1_REGS, (SPI_CLK_POL_LOW | SPI_CLK_INPHASE), dataFormat);//SPI_CLK_OUTOFPHASE   SPI_CLK_INPHASE

    // 配置 SPI 发送时 MSB 优先
    SPIShiftMsbFirst(SOC_SPI_1_REGS, dataFormat);

    // 设置字符长度
    SPICharLengthSet(SOC_SPI_1_REGS, CHAR_LENGTH, dataFormat);
}


/****************************************************************************/
/*                                                                          */
/*              SPI 传输    3-25去掉了接收                                                */
/*                                                                          */
/****************************************************************************/
int SpiTransfer(void)
{
    p_tx = &tx_data[0];
//    p_rx = &rx_data[0];

    while(tx_len)
    {
    	tx_len--;

    	SPIDat1Config(SOC_SPI_1_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), (1<<2));

        SPITransmitData1(SOC_SPI_1_REGS, *(p_tx+tx_len));//xxx 我已经明白了怎么去用edma写数据，但是这个前后还要进行“SPIDat1Config”配置操作，是这样吗？

//        while( (HWREG(SOC_SPI_1_REGS + SPI_SPIBUF) & 0x80000000 ) );
//        rx_data[2-tx_len] = SPIDataReceive(SOC_SPI_1_REGS);
    }

    SPIDat1Config(SOC_SPI_1_REGS, (SPI_DATA_FORMAT0), (1<<2));

//    return ((rx_data[0]<<16) | (rx_data[1]<<8) | (rx_data[2]));
    return 0;
}

/****************************************************************************/
/*                                                                          */
/*              SPI 初始化  		 	      	                            */
/*                                                                          */
/****************************************************************************/
void SPIInit(void)
{
	// 复位
    SPIReset(SOC_SPI_1_REGS);
    // 取消复位
    SPIOutOfReset(SOC_SPI_1_REGS);
    // 配置 主！ / 从模式
    SPIModeConfigure(SOC_SPI_1_REGS, SPI_MASTER_MODE);
    // 配置时钟
    SPIClkConfigure(SOC_SPI_1_REGS, 228000000, spishizhong, SPI_DATA_FORMAT0);
	// 使能 SIMO SOMI CLK 引脚
    unsigned int  val = 0x00000E04;
	SPIPinControl(SOC_SPI_1_REGS, 0, 0, &val);
	//设置CS3空闲时为高电平
	SPIDefaultCSSet(SOC_SPI_1_REGS, (1<<2));
	// 配置数据格式
    SPIDataFormatConfig(SPI_DATA_FORMAT0);

	SPIDat1Config(SOC_SPI_1_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), (1<<2));

    // 使能 SPI
    SPIEnable(SOC_SPI_1_REGS);

    // 配置管脚为输出状态
    GPIODirModeSet(SOC_GPIO_0_REGS, 111, GPIO_DIR_OUTPUT);  // GPIO6[14]
}

/****************************************************************************/
/*                                                                          */
/*              DAC 初始化  		 	      	                            */
/*                                                                          */
/****************************************************************************/
void DACInit(void)
{
    // 拉高 LDAC 管脚
    GPIOPinWrite(SOC_GPIO_0_REGS, 111, GPIO_PIN_HIGH);

    // 电源控制4通道上电
	DACReg.CFG.RW = 0;              // 写
	DACReg.CFG.ZERO = 0;            // 恒为 0
	DACReg.CFG.REG = 2;             // 寄存器选择 电源控制寄存器
	DACReg.CFG.Addr = 0;            // 通道选择   无
	DACReg.Data = 0x0001;           // 数据       4 通道上电
	tx_len = 3;
	memcpy(&tx_data, &DACReg, 3);
	SpiTransfer();

    // 输出范围选择 0~10V
    DACReg.CFG.RW = 0;              // 写
    DACReg.CFG.ZERO = 0;            // 恒为 0
    DACReg.CFG.REG = 1;             // 寄存器选择 输出范围寄存器
    DACReg.CFG.Addr = 4;            // 通道选择   所有通道
    DACReg.Data = 0x0001;           // 数据       10V
    tx_len = 3;
	memcpy(&tx_data, &DACReg, 3);
	SpiTransfer();
}

/****************************************************************************/
/*                                                                          */
/*              DAC 输出                                                    */
/*                                                                          */
/****************************************************************************/
void DACOutput(unsigned short tongdaoashuju)
{
//	// 拉低 LDAC 管脚
//    GPIOPinWrite(SOC_GPIO_0_REGS, 111, GPIO_PIN_LOW);
//	// DAC寄存器 VoutA = 3V
//	DACReg.CFG.RW = 0;              // 写
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 0;             // 寄存器选择 DAC 寄存器
//	DACReg.CFG.Addr = 4;            // 通道选择   A通道
//	DACReg.Data = 0x4cc0;           // 数据

	DACReg.Data = tongdaoashuju;           // 数据
	tx_len = 3;//3是不是代表数据长度，这里就是指4cc
	memcpy(&tx_data, &DACReg, 3);
	SpiTransfer();
//	Delay(0xf);
//
//	// DAC寄存器 VoutB = 5V
//	DACReg.CFG.RW = 0;              // 写
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 0;             // 寄存器选择 DAC 寄存器
//	DACReg.CFG.Addr = 1;            // 通道选择   B通道
//	DACReg.Data = 0x7ff0;           // 数据
//	tx_len = 3;
//	memcpy(&tx_data, &DACReg, 3);
//	SpiTransfer();
//	Delay(0xf);
//
//	// DAC寄存器 VoutC = 7V
//	DACReg.CFG.RW = 0;              // 写
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 0;             // 寄存器选择 DAC 寄存器
//	DACReg.CFG.Addr = 2;            // 通道选择   C通道
//	DACReg.Data = 0xb320;           // 数据
//	tx_len = 3;
//	memcpy(&tx_data, &DACReg, 3);
//	SpiTransfer();
//	Delay(0xf);
//
//	// DAC寄存器 VoutD = 10V
//	DACReg.CFG.RW = 0;              // 写
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 0;             // 寄存器选择 DAC 寄存器
//	DACReg.CFG.Addr = 3;            // 通道选择   D通道
//	DACReg.Data = 0xfff0;           // 数据
//	tx_len = 3;
//	memcpy(&tx_data, &DACReg, 3);
//	SpiTransfer();
//	Delay(0xf);
////XXX 输出数据告一段落
//	// 读 DAC 寄存器
//	DACReg.CFG.RW = 1;              // 读
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 0;             // 寄存器选择  DAC 寄存器
//	DACReg.CFG.Addr = 0;            // 通道选择    A 通道
//	DACReg.Data = 0;                // 数据
//	tx_len = 3;
//	memcpy(&tx_data, &DACReg, 3);
//	SpiTransfer();
//	Delay(0xf);
//
//	// 写控制寄存器  NOP  读回通道数据
//	UARTPuts("Read From DAC : \r\n", -1);
//    int ReceiveData = 0;
//	DACReg.CFG.RW = 0;              // 写
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 3;             // 寄存器选择
//	DACReg.CFG.Addr = 0;            // 通道选择   无
//	DACReg.Data = 0;                // 数据
//	tx_len = 3;
//	memcpy(&tx_data, &DACReg, 3);
//	ReceiveData = SpiTransfer();
//	Delay(0xf);
//
//	UARTprintf("CH A = 0x%x\r\n",ReceiveData);
//
//	// 读 DAC 寄存器
//	DACReg.CFG.RW = 1;              // 读
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 0;             // 寄存器选择  DAC 寄存器
//	DACReg.CFG.Addr = 1;            // 通道选择    B 通道
//	DACReg.Data = 0;                // 数据
//	tx_len = 3;
//	memcpy(&tx_data, &DACReg, 3);
//	SpiTransfer();
//	Delay(0xf);
//
//	// 写控制寄存器  NOP  读回B通道数据
//	DACReg.CFG.RW = 0;              // 写
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 3;             // 寄存器选择
//	DACReg.CFG.Addr = 0;            // 通道选择   无
//	DACReg.Data = 0;                // 数据
//	tx_len = 3;
//	memcpy(&tx_data, &DACReg, 3);
//	ReceiveData = SpiTransfer();
//	Delay(0xf);
//
//	UARTprintf("CH B = 0x%x\r\n",ReceiveData);
//
//	// 读 DAC 寄存器
//	DACReg.CFG.RW = 1;              // 读
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 0;             // 寄存器选择  DAC 寄存器
//	DACReg.CFG.Addr = 2;            // 通道选择    C 通道
//	DACReg.Data = 0;                // 数据
//	tx_len = 3;
//	memcpy(&tx_data, &DACReg, 3);
//	SpiTransfer();
//	Delay(0xf);
//
//	// 写控制寄存器  NOP  读回B通道数据
//	DACReg.CFG.RW = 0;              // 写
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 3;             // 寄存器选择
//	DACReg.CFG.Addr = 0;            // 通道选择   无
//	DACReg.Data = 0;                // 数据
//	tx_len = 3;
//	memcpy(&tx_data, &DACReg, 3);
//	ReceiveData = SpiTransfer();
//	Delay(0xf);
//
//	UARTprintf("CH C = 0x%x\r\n",ReceiveData);
//
//	// 读 DAC 寄存器
//	DACReg.CFG.RW = 1;              // 读
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 0;             // 寄存器选择  DAC 寄存器
//	DACReg.CFG.Addr = 3;            // 通道选择    D 通道
//	DACReg.Data = 0;                // 数据
//	tx_len = 3;
//	memcpy(&tx_data, &DACReg, 3);
//	SpiTransfer();
//	Delay(0xf);
//
//	// 写控制寄存器  NOP  读回B通道数据
//	DACReg.CFG.RW = 0;              // 写
//	DACReg.CFG.ZERO = 0;            // 恒为 0
//	DACReg.CFG.REG = 3;             // 寄存器选择
//	DACReg.CFG.Addr = 0;            // 通道选择   无
//	DACReg.Data = 0;                // 数据
//	tx_len = 3;
//	memcpy(&tx_data, &DACReg, 3);
//	ReceiveData = SpiTransfer();
//	Delay(0xf);
//
//	UARTprintf("CH D = 0x%x\r\n",ReceiveData);
}




/****************************************************************************/
/*                                                                          */
/*              指令时钟方法延时                                            */
/*                                                                          */
/****************************************************************************/
void delay(volatile unsigned int delay)
{
    while(delay--);
}

