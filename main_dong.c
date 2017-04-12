/****************************************************************************/
/*                                                                          */
/*              ��spi����ʱ�������spi������da������Ҳ���Ч���ܲ���                                          */
/*                                                                          */
/****************************************************************************/
// ע�⣺DSP ports, Shared RAM, UART0, EDMA, SPI0, MMC/SDs,
//       VPIF, LCDC, SATA, uPP, DDR2/mDDR (bus ports), USB2.0, HPI, PRU
//       ��Щ����ʹ�õ�ʱ����ԴΪ PLL0_SYSCLK2 Ĭ��Ƶ��Ϊ CPU Ƶ�ʵĶ���֮һ
//       ���ǣ�ECAPs, UART1/2, Timer64P2/3, eHRPWMs,McBSPs, McASP0, SPI1
//       ��Щ�����ʱ����Դ������ PLL0_SYSCLK2 �� PLL1_SYSCLK2 ��ѡ��
//       ͨ���޸� System Configuration (SYSCFG) Module
//       �Ĵ��� Chip Configuration 3 Register (CFGCHIP3) ����λ ASYNC3_CLKSRC
//       ����ʱ����Դ
//       ��Ĭ��ֵ�� 0 ��Դ�� PLL0_SYSCLK2
//                  1 ��Դ�� PLL1_SYSCLK2
//       �������Ϊ�˽��͹��ģ��������޸����ֵ������Ӱ��������������ʱ��Ƶ��


#include "hw_syscfg0_C6748.h"       // ϵͳ����ģ��Ĵ���
#include "gpio.h"                   // ͨ����������ں꼰�豸����㺯������
#include "timer.h"                  // ͨ����������ں꼰�豸����㺯������
#include "interrupt.h"              // DSP C6748 �ж����Ӧ�ó���ӿں���������ϵͳ�¼��Ŷ���
#include "TL6748.h"                 // ���� DSP6748 �������������
#include "hw_types.h"				 // ������
#include "soc_C6748.h"			     // DSP C6748 ����Ĵ���
#include "psc.h"                    // ��Դ��˯�߿��ƺ꼰�豸����㺯������
#include "spi.h"                    // ��������ӿں꼰�豸����㺯������
#include "uartStdio.h"              // ���ڱ�׼��������ն˺�������
#include <string.h>
#include "dongqi.h"
/****************************************************************************/
/*                                                                          */
/*              �궨��                                                      */
/*                                                                          */
/****************************************************************************/
// ����ϵ�
#define SW_BREAKPOINT     asm(" SWBP 0 ");

// 64λ ��ʱ�� / ����������
// ��ʱʱ�� 1 ��  ����228000000����
// ��32λ
//#define TMR_PERIOD_LSB32  (0x0D970100)
//#define TMR_PERIOD_LSB32  0//200000hz
static unsigned int   TMR_PERIOD_LSB32=0;
/// XXX 228000000���Ƕ�ʱ��Ŀ��ʱ��,����456Mhz��2��Ƶ��
// ��32λ 0
#define TMR_PERIOD_MSB32  (0)
#define spishizhong  3000000
//���������30M�ģ�daоƬ��dsp��ͨ��Ƶ�ʡ������������ܾͲ�����
#define dingshipinlv 23000
/****************************************************************************/
/*                                                                          */
/*              ȫ�ֱ���                                                    */
/*                                                                          */
/****************************************************************************/
//unsigned char Flag = 0;
int dongflag = 0;

/****************************************************************************/
/*                                                                          */
/*              �궨��                                                      */
/*                                                                          */
/****************************************************************************/
// �ַ�����
#define CHAR_LENGTH             8
// XXX �������򿴲�̫����Ҳ����̫���ף���ô����ȥ����������������
/****************************************************************************/
/*                                                                          */
/*              ȫ�ֱ���                                                    */
/*                                                                          */
/****************************************************************************/
 unsigned int flag = 0;

unsigned int tx_len;
unsigned int rx_len;
unsigned char tx_data[256];
unsigned char rx_data[256];
unsigned char *p_tx;
unsigned char *p_rx;

// DAC �Ĵ�����
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
/*              ��������                                                    */
/*                                                                          */
/****************************************************************************/
// ����ʹ������
void PSCInit(void);

// GPIO �ܽŸ�������
void GPIOBankPinMuxSet();
// GPIO �ܽų�ʼ��
void GPIOBankPinInit();

// ��ʱ�� / ��������ʼ��
void TimerInit(unsigned int period);
// ��ʱ�� / �������жϳ�ʼ��
void TimerInterruptInit(void);

// DSP �жϳ�ʼ��
void InterruptInit(void);
// �жϷ�����
void TimerIsr(void);
//static void Delay(volatile unsigned int delay);


void SPIInit(void);
void DACOutput(unsigned short tongdaoashuju);
void SPIDataFormatConfig(unsigned int dataFormat);
int SpiTransfer(void);
void DACInit(void);

int k=0;

/****************************************************************************/
/*                                                                          */
/*              ������                                                      */
/*                                                                          */
/****************************************************************************/
int main(void)
{
		int i;
		unsigned int SamplingRate;
		for ( i = 0; i < 100; i++)
		{
			zhengxianbo100[i] = zhengxianbo100[i]<< 4;
		}
		for ( i = 0; i < 2; i++)
		{
			lingyiceshi[i] = lingyiceshi[i]<< 4;
		}

	// ����ʹ������
	PSCInit();
	


	// GPIO �ܽŸ�������
	GPIOBankPinMuxSet();

	// GPIO �ܽų�ʼ��  ��ʼ��Ϊoutģʽ
	GPIOBankPinInit();

	// ��ʱ�� / ��������ʼ��
//	TimerInit();
	// ��ʱ�� / ��������ʼ��
	 SamplingRate = dingshipinlv;
	TMR_PERIOD_LSB32 = 228000000/SamplingRate;
	TimerInit(TMR_PERIOD_LSB32);

	// DSP �жϳ�ʼ��
	InterruptInit();



	// SPI ��ʼ��
	SPIInit();

	// DAC ��ʼ��
	DACInit();

	DACReg.CFG.RW = 0;              // д
	DACReg.CFG.ZERO = 0;            // ��Ϊ 0
	DACReg.CFG.REG = 0;             // �Ĵ���ѡ�� DAC �Ĵ���
	DACReg.CFG.Addr = 4;            // ͨ��ѡ��   Aͨ��
//	DACReg.Data = 0x4cc0;           // ����

	//XXX ��ʱ�� / �������жϳ�ʼ��.,��仰�ǲ���Ӧ�÷�����ѭ��������棿
	TimerInterruptInit();

	// ��ѭ��
	for(;;)
	{
//		if(flag == 1){
//			TimerIntDisable(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);
//
//			// �����ѹֵ
////			if(dongflag >= 100) dongflag=0;
////		    DACOutput(zhengxianbo100[dongflag]);
//		    //dongflag++;
//			DACOutput(lingyiceshi[dongflag]);
//		    flag =0;
//		    TimerIntEnable(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);
//		}

	}
}

/****************************************************************************/
/*                                                                          */
/*              PSC ��ʼ��                                                  */
/*                                                                          */
/****************************************************************************/
void PSCInit(void)
{//
	// ʹ�� GPIO ģ��
	// ����Ӧ����ģ���ʹ��Ҳ������ BootLoader �����
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_GPIO, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
    // ʹ�� SPI ģ��
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_SPI1, PSC_POWERDOMAIN_ALWAYS_ON, PSC_MDCTL_NEXT_ENABLE);
}

/****************************************************************************/
/*                                                                          */
/*              GPIO �ܽŸ�������                                           */
/*                                                                          */
/****************************************************************************/
void GPIOBankPinMuxSet(void)
{
	// ������Ӧ�� GPIO �ڹ���Ϊ��ͨ���������
	// ���İ� LED
	GPIOBank6Pin12PinMuxSetup();
	GPIOBank6Pin13PinMuxSetup();

	SPIPinMuxSetup(1);
	SPI1CSPinMuxSetup(2);
	// XXX �ο�TL5724-A��ͨ��DAģ�������TL138_1808_6748-EVM-A3�װ�ԭ��ͼ
	// �Ϳ��Կ���daģ���õ���spi1���õ���cs2Ƭѡ

	volatile unsigned int savePinMux = 0;

	savePinMux = HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) & ~(SYSCFG_PINMUX13_PINMUX13_7_4);
	HWREG(SOC_SYSCFG_0_REGS + SYSCFG0_PINMUX(13)) = ((SYSCFG_PINMUX13_PINMUX13_7_4_GPIO6_14 << SYSCFG_PINMUX13_PINMUX13_7_4_SHIFT) | savePinMux);
	//GPIO6_14����Ϊ��ͨgpio��

}

/****************************************************************************/
/*                                                                          */
/*              GPIO �ܽų�ʼ��                                             */
/*                                                                          */
/****************************************************************************/
void GPIOBankPinInit(void)
{
	// ���� LED ��Ӧ�ܽ�Ϊ����ܽ�
    // OMAPL138 �� DSP C6748 ���� 144 �� GPIO
	// ����Ϊ���� GPIO BANK ��ʼ�ܽŶ�Ӧֵ
    // ��Χ 1-144
	// GPIO0[0] 1
    // GPIO1[0] 17
	// GPIO2[0] 33
    // GPIO3[0] 49
	// GPIO4[0] 65
    // GPIO5[0] 81
	// GPIO6[0] 97
	// GPIO7[0] 113
	// GPIO8[0] 129

	// ���İ� LED
    GPIODirModeSet(SOC_GPIO_0_REGS, 109, GPIO_DIR_OUTPUT);  // GPIO6[12]
    GPIODirModeSet(SOC_GPIO_0_REGS, 110, GPIO_DIR_OUTPUT);  // GPIO6[13]
}

/****************************************************************************/
/*                                                                          */
/*              ���� SPI ���ݸ�ʽ                                           */
/*                                                                          */
/****************************************************************************/
void SPIDataFormatConfig(unsigned int dataFormat)
{
    // ���� SPI ʱ��
    SPIConfigClkFormat(SOC_SPI_1_REGS, (SPI_CLK_POL_LOW | SPI_CLK_INPHASE), dataFormat);//SPI_CLK_OUTOFPHASE   SPI_CLK_INPHASE

    // ���� SPI ����ʱ MSB ����
    SPIShiftMsbFirst(SOC_SPI_1_REGS, dataFormat);

    // �����ַ�����
    SPICharLengthSet(SOC_SPI_1_REGS, CHAR_LENGTH, dataFormat);
}


/****************************************************************************/
/*                                                                          */
/*              SPI ����                                                    */
/*                                                                          */
/****************************************************************************/
int SpiTransfer(void)
{
    p_tx = &tx_data[0];
    p_rx = &rx_data[0];

    while(tx_len)
    {
    	tx_len--;

    	SPIDat1Config(SOC_SPI_1_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), (1<<2));

        SPITransmitData1(SOC_SPI_1_REGS, *(p_tx+tx_len));

        while( (HWREG(SOC_SPI_1_REGS + SPI_SPIBUF) & 0x80000000 ) );
        rx_data[2-tx_len] = SPIDataReceive(SOC_SPI_1_REGS);
    }

    SPIDat1Config(SOC_SPI_1_REGS, (SPI_DATA_FORMAT0), (1<<2));

    return ((rx_data[0]<<16) | (rx_data[1]<<8) | (rx_data[2]));
}

/****************************************************************************/
/*                                                                          */
/*              SPI ��ʼ��  		 	      	                            */
/*                                                                          */
/****************************************************************************/
void SPIInit(void)
{
	// ��λ
    SPIReset(SOC_SPI_1_REGS);
    // ȡ����λ
    SPIOutOfReset(SOC_SPI_1_REGS);
    // ���� ���� / ��ģʽ
    SPIModeConfigure(SOC_SPI_1_REGS, SPI_MASTER_MODE);
    // ����ʱ��
    SPIClkConfigure(SOC_SPI_1_REGS, 228000000, spishizhong, SPI_DATA_FORMAT0);
	// ʹ�� SIMO SOMI CLK ����
    unsigned int  val = 0x00000E04;
	SPIPinControl(SOC_SPI_1_REGS, 0, 0, &val);
	//����CS3����ʱΪ�ߵ�ƽ
	SPIDefaultCSSet(SOC_SPI_1_REGS, (1<<2));
	// �������ݸ�ʽ
    SPIDataFormatConfig(SPI_DATA_FORMAT0);

	SPIDat1Config(SOC_SPI_1_REGS, (SPI_CSHOLD | SPI_DATA_FORMAT0), (1<<2));

    // ʹ�� SPI
    SPIEnable(SOC_SPI_1_REGS);

    // ���ùܽ�Ϊ���״̬
    GPIODirModeSet(SOC_GPIO_0_REGS, 111, GPIO_DIR_OUTPUT);  // GPIO6[14]
}

/****************************************************************************/
/*                                                                          */
/*              DAC ��ʼ��  		 	      	                            */
/*                                                                          */
/****************************************************************************/
void DACInit(void)
{
    // ���� LDAC �ܽ�
    GPIOPinWrite(SOC_GPIO_0_REGS, 111, GPIO_PIN_HIGH);

    // ��Դ����4ͨ���ϵ�
	DACReg.CFG.RW = 0;              // д
	DACReg.CFG.ZERO = 0;            // ��Ϊ 0
	DACReg.CFG.REG = 2;             // �Ĵ���ѡ�� ��Դ���ƼĴ���
	DACReg.CFG.Addr = 0;            // ͨ��ѡ��   ��
	DACReg.Data = 0x0001;           // ����       4 ͨ���ϵ�
	tx_len = 3;
	memcpy(&tx_data, &DACReg, 3);
	SpiTransfer();

    // �����Χѡ�� 0~10V
    DACReg.CFG.RW = 0;              // д
    DACReg.CFG.ZERO = 0;            // ��Ϊ 0
    DACReg.CFG.REG = 1;             // �Ĵ���ѡ�� �����Χ�Ĵ���
    DACReg.CFG.Addr = 4;            // ͨ��ѡ��   ����ͨ��
    DACReg.Data = 0x0001;           // ����       10V
    tx_len = 3;
	memcpy(&tx_data, &DACReg, 3);
	SpiTransfer();
}

/****************************************************************************/
/*                                                                          */
/*              DAC ���                                                    */
/*                                                                          */
/****************************************************************************/
void DACOutput(unsigned short tongdaoashuju)
{
	// ���� LDAC �ܽ�
    GPIOPinWrite(SOC_GPIO_0_REGS, 111, GPIO_PIN_LOW);
	// DAC�Ĵ��� VoutA = 3V


	DACReg.Data = tongdaoashuju;           // ����
	tx_len = 3;//3�ǲ��Ǵ������ݳ��ȣ��������ָ4cc
	memcpy(&tx_data, &DACReg, 3);
	SpiTransfer();

}
/****************************************************************************/
/*                                                                          */
/*              ��ʱ�� / ��������ʼ��                                       */
/*                                                                          */
/****************************************************************************/
/// XXX ��ʱ����Ҫ�� �ط�������
void TimerInit(unsigned int period)
{
    // ���� ��ʱ�� / ������ 2 Ϊ 64 λģʽ
    TimerConfigure(SOC_TMR_2_REGS, TMR_CFG_64BIT_CLK_INT);

    // ��������
    TimerPeriodSet(SOC_TMR_2_REGS, TMR_TIMER12, period);//���õ�32λ
    TimerPeriodSet(SOC_TMR_2_REGS, TMR_TIMER34, TMR_PERIOD_MSB32);//���ø�32λ��ע���32λĬ����0������ò�����23λ�����Ҳ����ʡ��

    // ʹ�� ��ʱ�� / ������ 2
    TimerEnable(SOC_TMR_2_REGS, TMR_TIMER12, TMR_ENABLE_CONT);
}

/****************************************************************************/
/*                                                                          */
/*              ��ʱ�� / �������жϳ�ʼ��                                   */
/*                                                                          */
/****************************************************************************/
void TimerInterruptInit(void)
{
	// ע���жϷ�����
	IntRegister(C674X_MASK_INT4, TimerIsr);

	// ӳ���жϵ� DSP �������ж�
	IntEventMap(C674X_MASK_INT4, SYS_INT_T64P2_TINTALL);

	// ʹ�� DSP �������ж�
	IntEnable(C674X_MASK_INT4);

	// ʹ�� ��ʱ�� / ������ �ж�   ������Ϳ�ʼ��ʱ�����ˣ�����������?
	TimerIntEnable(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);
}

/****************************************************************************/
/*                                                                          */
/*              DSP �жϳ�ʼ��                                              */
/*                                                                          */
/****************************************************************************/
void InterruptInit(void)
{
	// ��ʼ�� DSP �жϿ�����
	IntDSPINTCInit();

	// ʹ�� DSP ȫ���ж�
	IntGlobalEnable();
}

/****************************************************************************/
/*                                                                          */
/*              �жϷ�����                                                */
/*                                                                          */
/****************************************************************************/
void TimerIsr(void)
{
    // ���ö�ʱ�� / �������ж�
    TimerIntDisable(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);

    // ����жϱ�־
    IntEventClear(SYS_INT_T64P2_TINTALL);
    TimerIntStatusClear(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);


    if(flag ==0){
    	DACOutput(lingyiceshi[flag]);
        flag = 1;
    }
    if(flag ==1){
    	DACOutput(lingyiceshi[flag]);
		flag = 0;
	}


    // ʹ�� ��ʱ�� / ������ �ж�
    TimerIntEnable(SOC_TMR_2_REGS, TMR_INT_TMR12_NON_CAPT_MODE);
}



