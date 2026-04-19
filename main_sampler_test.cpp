#include "chu_init.h"
#include "i2c_core.h"
#include "cmps2.h"
#include <cstdlib>
#include "gpio_cores.h"
#include "xadc_core.h"
#include "sseg_core.h"
#include "spi_core.h"
#include "ps2_core.h"
#include "ddfs_core.h"
#include "adsr_core.h"

I2cCore i2c_bus(get_slot_addr(BRIDGE_BASE, S14_CMPS2));
Cmps2Core compass(&i2c_bus);
SsegCore sseg(get_slot_addr(BRIDGE_BASE, S8_SSEG));
PwmCore pwm(get_slot_addr(BRIDGE_BASE, S6_PWM));
GpoCore led(get_slot_addr(BRIDGE_BASE, S2_LED));

// Helper to get a multi-digit number from UART
int get_uart_int() {
    char buf[16];
    int i = 0;
    while (true) {
        if (!uart.rx_fifo_empty()) {
            char c = uart.rx_byte();
            uart.disp(c); // Echo back to terminal
            if (c == '\r' || c == '\n') {
                buf[i] = '\0';
                return atoi(buf);
            }
            if (i < 15) buf[i++] = c;
        }
    }
}

// active-low 7-seg patterns for compass letters
uint8_t seg_char(char c) {
    switch (c) {
        case 'N': return 0xab; // approximate n
        case 'E': return 0x86; // same as hex E
        case 'S': return 0x92; // same pattern as 5
        case 'W': return 0xc1; // approximate W/U-like
        default:  return 0xff; // blank
    }
}

// Set both RGB LEDs at once
// assumed channel map:
// 0=R1, 1=G1, 2=B1
// 3=R2, 4=G2, 5=B2
void set_rgb_both(double r, double g, double b) {
    pwm.set_duty(b, 0);
    pwm.set_duty(g, 1);
    pwm.set_duty(r, 2);

    pwm.set_duty(b, 3);
    pwm.set_duty(g, 4);
    pwm.set_duty(r, 5);
}

void set_color_from_cardinal(const char* card) {
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;

    if (card[0] == 'N' && card[1] == '\0') {
        // N = green
        g = 1.0;
    }
    else if (card[0] == 'S' && card[1] == '\0') {
        // S = red
        r = 1.0;
    }
    else if (card[0] == 'E' && card[1] == '\0') {
        // E = blue
        b = 1.0;
    }
    else if (card[0] == 'W' && card[1] == '\0') {
        // W = yellow
        r = 1.0;
        g = 1.0;
    }
    else if (card[0] == 'N' && card[1] == 'E' && card[2] == '\0') {
        // NE = green + blue
        g = 1.0;
        b = 1.0;
    }
    else if (card[0] == 'S' && card[1] == 'E' && card[2] == '\0') {
        // SE = red + blue
        r = 1.0;
        b = 1.0;
    }
    else if (card[0] == 'S' && card[1] == 'W' && card[2] == '\0') {
        // SW = red + green
        r = 1.0;
        b = 1.0;
    }
    else if (card[0] == 'N' && card[1] == 'W' && card[2] == '\0') {
        // NW = green + blue
        g = 1.0;
        b = 1.0;
    }

    set_rgb_both(r, g, b);
}

void show_octant_led(const char* card) {
    uint32_t ptn = 0;

    if      (card[0] == 'N' && card[1] == '\0') ptn = 1 << 0;
    else if (card[0] == 'N' && card[1] == 'E')  ptn = 1 << 1;
    else if (card[0] == 'E' && card[1] == '\0') ptn = 1 << 2;
    else if (card[0] == 'S' && card[1] == 'E')  ptn = 1 << 3;
    else if (card[0] == 'S' && card[1] == '\0') ptn = 1 << 4;
    else if (card[0] == 'S' && card[1] == 'W')  ptn = 1 << 5;
    else if (card[0] == 'W' && card[1] == '\0') ptn = 1 << 6;
    else if (card[0] == 'N' && card[1] == 'W')  ptn = 1 << 7;

    led.write(ptn);
}

int main() {
    uart.disp("--- MANUAL OFFSET COMPASS ---\n\r");
    compass.init();
    sseg.set_dp(0x00);   // turn off all decimal points
    pwm.set_freq(50);
    set_rgb_both(0.0, 0.0, 0.0);

    uint8_t ptn[8];

    // 1. Magnetic mapping
    uart.disp("Rotate 360... Press any key to lock.\n\r");
    while (uart.rx_fifo_empty()) {
        compass.get_heading(11.202f, nullptr);
        sleep_ms(50);
    }
    uart.rx_byte();
    compass.lock_calibration();

    // 2. Manual Adjustment
    while (1) {
        bool err;
        int current = compass.get_heading(11.202f, &err);
        uart.disp("\rCurrent Heading: "); uart.disp(current);
        uart.disp(" deg. Press 's' to set offset or 'k' to keep.");

        if (!uart.rx_fifo_empty()) {
            char cmd = uart.rx_byte();
            if (cmd == 's') {
                uart.disp("\n\rEnter value to SUBTRACT: ");
                int val = get_uart_int();
                compass.set_manual_offset((float)val);
                uart.disp("\n\rOffset applied.\n\r");
                break; 
            }
            if (cmd == 'k') break;
        }
        sleep_ms(200);
    }

    // 3. Live mode
    while (1) {
        bool err;
        int head = compass.get_heading(11.202f, &err);
        const char* card = compass.get_cardinal(head);

        uart.disp("Heading: "); uart.disp(head);
        uart.disp(" ["); uart.disp(card);
        uart.disp("]\n\r");

        int hundreds = head / 100;
        int tens     = (head / 10) % 10;
        int ones     = head % 10;

        // Clear display
        ptn[0] = 0xff;
        ptn[1] = 0xff;
        ptn[2] = 0xff;
        ptn[3] = 0xff;
        ptn[4] = 0xff;
        ptn[5] = 0xff;
        ptn[6] = 0xff;
        ptn[7] = 0xff;

        // Put heading on right side
        ptn[0] = sseg.h2s(ones);
        ptn[1] = sseg.h2s(tens);
        ptn[2] = sseg.h2s(hundreds);

        // Put cardinal on left side
        if (card[1] != '\0') {
            ptn[6] = seg_char(card[1]);
            ptn[7] = seg_char(card[0]);
        } else {
            ptn[7] = seg_char(card[0]);
        }

        sseg.write_8ptn(ptn);
        sseg.set_dp(0x00);

        // Update both RGB LEDs from compass direction
        set_color_from_cardinal(card);

        show_octant_led(card);

        sleep_ms(250);
    }
}
