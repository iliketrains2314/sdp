/**
 * Minimal BLDC Motor Controller - With Direct GPIO2 Interrupt & Sine Wave
 */
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xsysmon.h"
#include "xil_exception.h"

XSysMon S;
XGpio G0, G1, G2, G3;
u16 spd = 0, trq = 0, u = 0, v = 0, w = 0;
u8 trap = 0, dir = 0, bemf = 0, test_bit = 0;
volatile u32 gpio2_in = 0;
volatile u32 gpio2_count = 0;
u16 gpio3_in = 0;
u32 rpm = 0;
u32 current_rpm = 0;


volatile u8 sine_index = 0;
volatile u8 sine_enabled = 0;
volatile u8 sine_index1 = 0;

// Sine wave lookup table (64 samples, 0-65535 range)
const u16 sine_table[64] = {
    2048, 2248, 2447, 2642, 2831, 3013, 3185, 3347,
    3496, 3631, 3751, 3854, 3940, 4007, 4056, 4085,
    4095, 4085, 4056, 4007, 3940, 3854, 3751, 3631,
    3496, 3347, 3185, 3013, 2831, 2642, 2447, 2248,
    2048, 1847, 1648, 1453, 1264, 1082,  910,  748,
     599,  464,  344,  241,  155,   88,   39,   10,
       0,   10,   39,   88,  155,  241,  344,  464,
     599,  748,  910, 1082, 1264, 1453, 1648, 1847
};

void Upd() {
    XGpio_DiscreteWrite(&G0, 1, spd);
    XGpio_DiscreteWrite(&G0, 2, trq);
    XGpio_DiscreteWrite(&G1, 1, trap);
    XGpio_DiscreteWrite(&G1, 2, (test_bit<<3)|(1<<2)|(bemf<<1)|dir);

    // Only write constant if sine wave disabled
    if(sine_enabled) {
		XGpio_DiscreteWrite(&G2, 2, sine_table[sine_index]); //sine_table[sine_index]
		sine_index++;
		if(sine_index >= 64) sine_index = 0;
		sine_index1 = sine_index;
	}
    if(!sine_enabled) {
        XGpio_DiscreteWrite(&G2, 2, current_rpm);
    }
}


void ExternalIsr(void *CallbackRef)
{
	// Read GPIO2 channel 1
	gpio2_in = XGpio_DiscreteRead(&G2, 1);

	// Calculate RPM: rpm = (gpio2_ch1 * 65535) / 6000
	current_rpm = (u16)(uint16_t)((180UL * 100000000UL) / ( gpio2_in));
	//(uint16_t)((60UL * 100000000UL) / gpio2_in);
	// Calculate target RPM from speed setpoint (spd maps 0-65535 to 0-10000 RPM)
	u32 target_rpm = (spd * 10000u) / 65535u;

	// Calculate ±10% tolerance
	u32 rpm_low = target_rpm - (target_rpm / 10);
	u32 rpm_high = target_rpm + (target_rpm / 10);

	// Check if current RPM is within ±10% of target
	if(current_rpm >= rpm_low && current_rpm <= rpm_high) {
		// Within tolerance - call Upd()
		Upd();
		//xil_printf(current_rpm);
	}
}

void Init() {
    XSysMon_Config *c = XSysMon_LookupConfig(XPAR_SYSMON_0_DEVICE_ID);
    XSysMon_CfgInitialize(&S, c, c->BaseAddress);
    XSysMon_SetSequencerMode(&S, XSM_SEQ_MODE_CONTINPASS);
    //XSysMon_SetSeqChEnables(&S, XSM_SEQ_CH_AUX03|XSM_SEQ_CH_AUX10|XSM_SEQ_CH_AUX02);

    XGpio_Initialize(&G0, XPAR_AXI_GPIO_0_DEVICE_ID);
    XGpio_Initialize(&G1, XPAR_AXI_GPIO_1_DEVICE_ID);
    XGpio_Initialize(&G2, XPAR_AXI_GPIO_2_DEVICE_ID);
    XGpio_Initialize(&G3, XPAR_AXI_GPIO_3_DEVICE_ID);

    XGpio_SetDataDirection(&G0, 1, 0);
    XGpio_SetDataDirection(&G0, 2, 0);
    XGpio_SetDataDirection(&G1, 1, 0);
    XGpio_SetDataDirection(&G1, 2, 0);
    XGpio_SetDataDirection(&G2, 1, 1);
    XGpio_SetDataDirection(&G2, 2, 0);
    XGpio_SetDataDirection(&G3, 1, 1);
}

void SetupInterrupts(void)
{
    // Initialize exception handling
    Xil_ExceptionInit();

    // Register ISR for external interrupt (directly connected to MicroBlaze)
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                  (Xil_ExceptionHandler)ExternalIsr,
                                  NULL);

    // Enable MicroBlaze interrupts
    Xil_ExceptionEnable();

    // Note: No GPIO interrupt setup needed - external signal triggers directly
}

/*
void ReadADC() {
    u = XSysMon_GetAdcData(&S, XSM_CH_AUX_MIN+3);
    v = XSysMon_GetAdcData(&S, XSM_CH_AUX_MIN+10);
    w = XSysMon_GetAdcData(&S, XSM_CH_AUX_MIN+2);
}*/

u32 GetNum() {
    u32 n = 0;
    char c;
    while(1) {
        c = inbyte();
        if(c=='\r'||c=='\n') break;
        xil_printf("%c", c);
        if(c>='0'&&c<='9') n = n*10+(c-'0');
    }
    xil_printf("\r\n");
    return n;
}

int main()
{
    init_platform();
    Init();
    SetupInterrupts();
    Upd();

    xil_printf("BLDC\r\n");

    u16 cnt = 0;
    int n;

    while(1) {
        // Periodic GPIO3 read
        if(++cnt >= 1000) {
            gpio3_in = XGpio_DiscreteRead(&G3, 1);
            cnt = 0;
        }

        xil_printf(">");
        char c = inbyte();
        xil_printf("%c\r\n", c);

        if(c=='s') { xil_printf("S:"); spd=(u16)GetNum(); Upd(); }
        else if(c=='t') { xil_printf("T:"); trq=(u16)GetNum(); Upd(); }
        else if(c=='p') { xil_printf("P:"); trap=(u8)GetNum(); if(trap>100)trap=100; Upd(); }
        else if(c=='d') { dir=!dir; xil_printf("D:%d\r\n",dir); Upd(); }
        else if(c=='m') { bemf=!bemf; xil_printf("M:%d\r\n",bemf); Upd(); }
        else if(c=='+'||c=='=') {
            n=(int)spd+100;
            if(n>65535)n=65535;
            spd=(u16)n;
            XGpio_DiscreteWrite(&G0,1,spd);
            xil_printf("S:%d\r\n",spd);
        }
        else if(c=='-') {
            n=(int)spd-100;
            if(n<0)n=0;
            spd=(u16)n;
            XGpio_DiscreteWrite(&G0,1,spd);
            xil_printf("S:%d\r\n",spd);
        }
        else if(c=='i') {
            //ReadADC();
            gpio3_in = XGpio_DiscreteRead(&G3, 1);
            //u32 rpm = 0;
            xil_printf("S:%d T:%d P:%d D:%d M:%d Z:%d\r\n",spd,trq,trap,dir,bemf,test_bit);
            xil_printf("G2:%u (%u RPM) G3:%d IntCnt:%u\r\n",gpio2_in,current_rpm,gpio3_in,gpio2_count);
            xil_printf("U:%X V:%X W:%X\r\n",u,v,w);
        }
        else if(c=='r') {
            // Manual read of GPIO2 to compare
            u32 manual = XGpio_DiscreteRead(&G2, 1);
            u32 rpm_manual = 0;
            u32 rpm_isr = 0;

            // Integer math: RPM = (60 * 10000000) / (14 * period)
            // = 600000000 / (14 * period)
            if(manual > 0) {
                rpm_manual = 600000000UL / (14 * manual);
            }
            if(gpio2_in > 0) {
                rpm_isr = 600000000UL / (14 * gpio2_in);
            }

            xil_printf("Manual G2:%u (%u RPM)  ISR G2:%u (%u RPM)  IntCnt:%u\r\n",
                       manual, rpm_manual, gpio2_in, rpm_isr, gpio2_count);
        }
        else if(c=='z') {
            test_bit = !test_bit;
            Upd();
            xil_printf("Z:%d\r\n", test_bit);
        }
        else if(c=='w') {
            sine_enabled = !sine_enabled;
            if(!sine_enabled) {
                XGpio_DiscreteWrite(&G2, 2, 5000);  // Reset to constant
            }
            xil_printf("Sine:%d\r\n", sine_enabled);
        }
        else if(c=='x') { spd=0; trq=0; Upd(); xil_printf("X\r\n"); }
        else if(c=='q') { spd=0; trq=0; Upd(); break; }
    }

    cleanup_platform();
    return 0;
}
