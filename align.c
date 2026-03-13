/**
 * Minimal BLDC Motor Controller - With Direct GPIO2 Interrupt & Sine Wave
 *
 * UPDATED to use FPGA align_en:
 * - Repurpose the old "bemf" software bit as align_en (bit1 of G1 ch2)
 * - Adds align_rotor() that asserts align_en for ALIGN_TIME_MS while holding
 *   a fixed commutation vector in HDL (ALIGN_STEP)
 * - Triggers align automatically when speed transitions 0 -> nonzero
 *
 * IMPORTANT:
 * - Align only works in OPEN-LOOP TOP path, so keep enable_bemf=0 during align.
 * - Align requires speed > 0 (TOP turns off outputs when speed==0), so align_rotor()
 *   forces a small nonzero speed during align, then restores the commanded speed.
 */

#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xsysmon.h"
#include "xil_exception.h"
#include "sleep.h"     // for usleep

XSysMon S;
XGpio G0, G1, G2, G3;

u16 spd = 0, trq = 0, u = 0, v = 0, w = 0;
u8  trap = 0, dir = 0, bemf = 0, test_bit = 0;

/*
 * Software note:
 *   You said you aren't using BEMF mode from software, so we repurpose the
 *   old 'bemf' bit (bit1 in G1 ch2 control word) as align_en.
 *
 * Control word on G1 channel 2:
 *   bit0 = dir
 *   bit1 = align_en   (was bemf)
 *   bit2 = enable (kept constant 1)
 *   bit3 = test_bit
 */
static volatile u8 align_en = 0;

// Track last commanded speed to detect 0 -> nonzero transitions
static u16 last_spd_cmd = 0;

volatile u32 gpio2_in = 0;        // raw GPIO2 ch1 (latest period sample)
volatile u32 gpio2_count = 0;     // interrupt count
u16 gpio3_in = 0;
u32 rpm = 0;
u32 current_rpm = 0;
u32 scaled = 0;

// ---------------- Midpoint Filter (GPIO2 Channel 1) ----------------
#define GPIO2_MA_LEN  32   // window length

static volatile u32 gpio2_ma_buf[GPIO2_MA_LEN] = {0};
static volatile u32 gpio2_ma_idx = 0;
static volatile u32 gpio2_ma_count = 0;     // number of valid samples collected (<= GPIO2_MA_LEN)
static volatile u32 gpio2_in_avg = 0;       // filtered period (midpoint of min/max)
// ------------------------------------------------------------------

// ---------------- Align parameters ----------------
#define ALIGN_TIME_MS   200      // hold vector for 200 ms
#define ALIGN_SPD       10000      // must be nonzero so TOP outputs are enabled
#define ALIGN_TRQ       20000     // moderate duty (TOP uses torque[15:6])
#define ALIGN_TRAP      0        // square during align (recommended)
// -------------------------------------------------

volatile u8 sine_index = 0;
volatile u8 sine_enabled = 0;
volatile u8 sine_index1 = 0;

// Sine wave lookup table (64 samples, 0-4095 range)
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

static void Upd(void)
{
    XGpio_DiscreteWrite(&G0, 1, spd);
    XGpio_DiscreteWrite(&G0, 2, trq);
    XGpio_DiscreteWrite(&G1, 1, trap);

    // UPDATED: bit1 is align_en (repurposed from bemf)
    XGpio_DiscreteWrite(&G1, 2, (test_bit<<3) | (align_en<<2) | (0<<1) | dir);

    // Only write constant if sine wave disabled
    if (sine_enabled) {
        XGpio_DiscreteWrite(&G2, 2, sine_table[sine_index]);
        sine_index++;
        if (sine_index >= 64) sine_index = 0;
        sine_index1 = sine_index;
    }
    if (!sine_enabled) {
        XGpio_DiscreteWrite(&G2, 2, scaled);
    }
}

// True rotor align: hold commutation vector in FPGA while applying torque for ALIGN_TIME_MS
static void align_rotor(void)
{
    // Save current commanded values
    u16 saved_spd  = spd;
    u16 saved_trq  = trq;
    u8  saved_trap = trap;

    // During align we want a fixed magnetic vector: square drive is simplest
    trap = ALIGN_TRAP;
    trq  = ALIGN_TRQ;

    // Must be nonzero for TOP to output anything; HDL will hold commutation_step during align_en
    spd  = (saved_spd == 0) ? ALIGN_SPD : saved_spd;

    // Assert align
    align_en = 1;
    Upd();

    // Hold
    usleep(ALIGN_TIME_MS * 1000);

    // Release align
    align_en = 0;

    // Restore original requested values
    spd  = saved_spd;
    trq  = saved_trq;
    trap = saved_trap;

    Upd();
}

// Detect start event: speed command goes from 0 -> nonzero
static void handle_start_event(void)
{
    if (last_spd_cmd == 0 && spd > 0) {
        align_rotor();
    }
    last_spd_cmd = spd;
}

/**
 * ISR is triggered by your Verilog "valid" pulse (measurement completed).
 * GPIO2 channel 1 contains the latched 'period' value.
 * We apply a midpoint filter (min/max midpoint) to period, then compute RPM.
 */
void ExternalIsr(void *CallbackRef)
{
    (void)CallbackRef;

    gpio2_count++;

    // Read GPIO2 channel 1 (latched period from bemf_speed_estimator)
    u32 sample = XGpio_DiscreteRead(&G2, 1);

    // Ignore invalid samples to avoid divide-by-zero / poisoning the filter
    if (sample == 0) {
        return;
    }

    gpio2_in = sample;

    // ---------------- Midpoint filter on period ----------------
    gpio2_ma_buf[gpio2_ma_idx] = sample;

    gpio2_ma_idx++;
    if (gpio2_ma_idx >= GPIO2_MA_LEN) gpio2_ma_idx = 0;

    if (gpio2_ma_count < GPIO2_MA_LEN) {
        gpio2_ma_count++;
    }

    // Compute min/max over collected samples, then midpoint
    {
        u32 minv = 0xFFFFFFFFu;
        u32 maxv = 0u;
        u32 i;

        for (i = 0; i < gpio2_ma_count; i++) {
            u32 vv = gpio2_ma_buf[i];
            if (vv < minv) minv = vv;
            if (vv > maxv) maxv = vv;
        }

        gpio2_in_avg = minv + ((maxv - minv) / 2u);
    }
    // ----------------------------------------------------------

    // Compute RPM using FILTERED period
    if (gpio2_in_avg > 0) {
        current_rpm = (u32)((180UL * 100000000UL) / gpio2_in_avg);
        scaled = (u32)(173UL * current_rpm / 20UL - 9050UL);
    } else {
        return;
    }

    // Calculate target RPM from speed setpoint (spd maps 0-65535 to 0-10000 RPM)
    u32 target_rpm = (spd * 10000u) / 65535u;

    // Calculate ±10% tolerance
    u32 tol = target_rpm / 10u;
    u32 rpm_low  = (target_rpm > tol) ? (target_rpm - tol) : 0u;
    u32 rpm_high = target_rpm + tol;

    // Check if current RPM is within ±10% of target
    if (current_rpm >= rpm_low && current_rpm <= rpm_high) {
        Upd();
    }
    Upd(); // (kept your original extra update)
}

static void Init(void)
{
    XSysMon_Config *c = XSysMon_LookupConfig(XPAR_SYSMON_0_DEVICE_ID);
    XSysMon_CfgInitialize(&S, c, c->BaseAddress);
    XSysMon_SetSequencerMode(&S, XSM_SEQ_MODE_CONTINPASS);

    XGpio_Initialize(&G0, XPAR_AXI_GPIO_0_DEVICE_ID);
    XGpio_Initialize(&G1, XPAR_AXI_GPIO_1_DEVICE_ID);
    XGpio_Initialize(&G2, XPAR_AXI_GPIO_2_DEVICE_ID);
    XGpio_Initialize(&G3, XPAR_AXI_GPIO_3_DEVICE_ID);

    XGpio_SetDataDirection(&G0, 1, 0);
    XGpio_SetDataDirection(&G0, 2, 0);
    XGpio_SetDataDirection(&G1, 1, 0);
    XGpio_SetDataDirection(&G1, 2, 0);
    XGpio_SetDataDirection(&G2, 1, 1); // input: period
    XGpio_SetDataDirection(&G2, 2, 0); // output: rpm/sine
    XGpio_SetDataDirection(&G3, 1, 1);

    // Ensure align starts deasserted
    align_en = 0;
}

static void SetupInterrupts(void)
{
    Xil_ExceptionInit();

    // Register ISR for external interrupt (directly connected to MicroBlaze)
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                 (Xil_ExceptionHandler)ExternalIsr,
                                 NULL);

    Xil_ExceptionEnable();
}

static u32 GetNum(void)
{
    u32 n = 0;
    char c;
    while (1) {
        c = inbyte();
        if (c == '\r' || c == '\n') break;
        xil_printf("%c", c);
        if (c >= '0' && c <= '9') n = n * 10 + (c - '0');
    }
    xil_printf("\r\n");
    return n;
}

int main(void)
{
    init_platform();
    Init();
    SetupInterrupts();
    Upd();

    xil_printf("BLDC\r\n");
    xil_printf("Please speed i need this :()\r\n");

    u16 cnt = 0;
    int n;

    while (1) {
        // Periodic GPIO3 read
        if (++cnt >= 1000) {
            gpio3_in = XGpio_DiscreteRead(&G3, 1);
            cnt = 0;
        }

        xil_printf(">");
        char c = inbyte();
        xil_printf("%c\r\n", c);

        if (c == 's') {
            xil_printf("S:");
            spd = (u16)GetNum();
            handle_start_event();  // NEW: auto-align on 0->nonzero
            Upd();
        }
        else if (c == 't') { xil_printf("T:"); trq = (u16)GetNum(); Upd(); }
        else if (c == 'p') { xil_printf("P:"); trap = (u8)GetNum(); if (trap > 100) trap = 100; Upd(); }
        else if (c == 'd') { dir = !dir; xil_printf("D:%d\r\n", dir); Upd(); }

        // 'm' used to toggle bemf; now we map it to manual align toggle for testing (optional)
        else if (c == 'm') {
            align_en = !align_en;
            xil_printf("ALIGN_EN:%d\r\n", align_en);
            Upd();
        }

        else if (c == '+' || c == '=') {
            n = (int)spd + 100;
            if (n > 65535) n = 65535;
            spd = (u16)n;
            handle_start_event();  // NEW: catches 0->nonzero via + key too
            XGpio_DiscreteWrite(&G0, 1, spd);
            xil_printf("S:%d\r\n", spd);
        }
        else if (c == '-') {
            n = (int)spd - 100;
            if (n < 0) n = 0;
            spd = (u16)n;
            // Update last speed tracking too
            last_spd_cmd = spd;
            XGpio_DiscreteWrite(&G0, 1, spd);
            xil_printf("S:%d\r\n", spd);
        }
        else if (c == 'i') {
            gpio3_in = XGpio_DiscreteRead(&G3, 1);

            xil_printf("S:%d T:%d P:%d D:%d ALIGN:%d Z:%d\r\n",
                       spd, trq, trap, dir, align_en, test_bit);

            xil_printf("G2_period:%u avg:%u  RPM:%u SRPM:%u G3:%d  IntCnt:%u\r\n",
                       gpio2_in, gpio2_in_avg, current_rpm, scaled, gpio3_in, gpio2_count);

            xil_printf("U:%X V:%X W:%X\r\n", u, v, w);
        }
        else if (c == 'r') {
            // Manual read of GPIO2 to compare
            u32 manual = XGpio_DiscreteRead(&G2, 1);
            u32 rpm_manual = 0;
            u32 rpm_isr = 0;

            if (manual > 0) {
                rpm_manual = 600000000UL / (14 * manual);
            }
            if (gpio2_in > 0) {
                rpm_isr = 600000000UL / (14 * gpio2_in);
            }

            xil_printf("Manual G2:%u (%u RPM)  ISR G2:%u (%u RPM)  IntCnt:%u\r\n",
                       manual, rpm_manual, gpio2_in, rpm_isr, gpio2_count);
        }
        else if (c == 'z') {
            test_bit = !test_bit;
            Upd();
            xil_printf("Z:%d\r\n", test_bit);
        }
        else if (c == 'w') {
            sine_enabled = !sine_enabled;
            if (!sine_enabled) {
                XGpio_DiscreteWrite(&G2, 2, 5000);  // Reset to constant
            }
            xil_printf("Sine:%d\r\n", sine_enabled);
        }
        else if (c == 'x') { spd = 0; trq = 0; last_spd_cmd = 0; align_en = 0; Upd(); xil_printf("X\r\n"); }
        else if (c == 'q') { spd = 0; trq = 0; last_spd_cmd = 0; align_en = 0; Upd(); break; }
    }

    cleanup_platform();
    return 0;
}
