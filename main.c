#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#define BAUD_D  9600UL
#define BSEL_D(baud)  ((F_CPU / (16 * baud)) - 1)

#define PWM_FREQ 1000L
#define PER (F_CPU/(2*PWM_FREQ))

#define STATE_FREQ 100L
#define STATE_PER ((F_CPU/8)/STATE_FREQ)

#define RC_FREQ 50L

#define sec_to_tic(sec) sec*F_CPU
#define CENTER_LEN 1.5e-3
#define DEAD_BAND  0.04e-3

#define ON PER
#define OFF 0
#define DELAY (4*PER)/100L

#define LEFT_STICK 0x00
#define RIGHT_STICK 0x01

#define LEFT_MOTOR 0x00
#define RIGHT_MOTOR 0x01

#define FORWARD 1
#define BACKWARD -1

#define AH 0x00
#define AL 0x01
#define BH 0x02
#define BL 0x03

#define TRUE 0x01
#define FALSE 0x00

static volatile uint16_t* drive[4][2] = {
    {&TCC0_CCABUF, &TCC0_CCCBUF},
    {           0,            0},
    {&TCC0_CCBBUF, &TCC0_CCDBUF},
    {           0,            0}};

static int16_t min_pulse_width[2];
static int16_t max_pulse_width[2];

static int8_t left_stick = 0;
static int8_t right_stick = 0;

static volatile uint32_t number_of_rc_commands = 0;
static volatile uint32_t number_of_runs_without_rc_command = 0;
static volatile uint8_t rc_receiver_ready = FALSE;

void
clock_init() {

    /* RC2M->PLL, 4x multiplier */
    OSC.PLLCTRL = OSC_PLLSRC_RC2M_gc | 0x4;

    /* Start PLL */
    OSC.CTRL |= OSC_PLLEN_bm;

    /* Wait for PLL to become stable */
    while (!(OSC.STATUS & OSC_PLLRDY_bm));

    /* Disable ccp "Configuration Change Protection" */
    CCP = CCP_IOREG_gc;

    /* Use PLL output as the system clock */
    CLK.CTRL = CLK_SCLKSEL_PLL_gc;
}

void
rc_init() {

    /* Configure PB0 and PB1 for input */
    PORTB.DIRCLR = PIN0_bm;
    PORTB.DIRCLR = PIN1_bm;

    /* Interrupt on both edges. */
    PORTB.PIN0CTRL = PORT_ISC_BOTHEDGES_gc;
    PORTB.PIN1CTRL = PORT_ISC_BOTHEDGES_gc;

    /* Select PB0 as input to event channel 0,
     *    and PB1 as input to event channel 1. */
    EVSYS.CH0MUX = EVSYS_CHMUX_PORTB_PIN0_gc;
    EVSYS.CH1MUX = EVSYS_CHMUX_PORTB_PIN1_gc;

    /* Pulse width capture, using event channel 0 and 1 */
    TCC1.CTRLD = TC_EVSEL_CH0_gc | TC_EVACT_PW_gc;

    /* Enable Input "Capture or Compare" channel A and B. */
    TCC1.CTRLB |= TC0_CCAEN_bm | TC0_CCBEN_bm;

    /* Set the period of the counter. Quote from datasheet:
     *
     * If the Period register value is set lower than 0x8000, the polarity of
     * the I/O pin edge will be stored in the Most Significant Bit (MSB) of the
     * Capture register after a Capture. If the MSB of the Capture register is
     * zero, a falling edge generated the Capture. If the MSB is one, a rising
     * edge generated the Capture.
     */
    TCC1_PER = 0x7fff;

    /* Start timer by selecting a clock source. No prescaling. */
    TCC1.CTRLA = TC_CLKSEL_DIV1_gc;

    /* Enable CCA and CCB interrupt */
    TCC1.INTCTRLB = TC_CCAINTLVL_LO_gc | TC_CCBINTLVL_LO_gc;
}

void
fsm_init() {

    /* Set the period of the counter */
    TCD0_PER = STATE_PER;

    /* Start timer by selecting a clock source. /8 prescaling. */
    TCD0.CTRLA = TC_CLKSEL_DIV8_gc;

    /* Enable overflow interrupt */
    TCD0.INTCTRLA = TC_OVFINTLVL_HI_gc;
}

void
pwm_init() {

    /* Enable the PWM outputs for the left and right motor
     * Left:  PC0, PC1, PC2 and PC3
     * Right: PC4, PC5, PC6 and PC7
     */
    PORTC.DIR = 0xFF;

    /* Set the TC period */
    TCC0_PER = PER;

    /* Configure the TC for dual slope mode. */
    TCC0.CTRLB = TC_WGMODE_DS_T_gc;

    /* Enable Compare channel A, B, C and D. */
    TCC0.CTRLB |= TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm;

    /* Start timer by selecting a clock source. No prescaler. */
    TCC0.CTRLA = TC_CLKSEL_DIV1_gc;

    /* Enable AwEx Dead Time Insertion */
    AWEXC.CTRL |= AWEX_DTICCAEN_bm | AWEX_DTICCBEN_bm
               |  AWEX_DTICCCEN_bm | AWEX_DTICCDEN_bm;

    /* Set the dead time (both high and low)
     * in number of CPU cycles
     * 240 ticks = 30us */
    AWEXC.DTBOTH = 240;

    /* Override all pins */
    AWEXC.OUTOVEN = 0xff;

    /* Invert all pins */
    PORTC.PIN0CTRL |= PORT_INVEN_bm;
    PORTC.PIN1CTRL |= PORT_INVEN_bm;
    PORTC.PIN2CTRL |= PORT_INVEN_bm;
    PORTC.PIN3CTRL |= PORT_INVEN_bm;
    PORTC.PIN4CTRL |= PORT_INVEN_bm;
    PORTC.PIN5CTRL |= PORT_INVEN_bm;
    PORTC.PIN6CTRL |= PORT_INVEN_bm;
    PORTC.PIN7CTRL |= PORT_INVEN_bm;

    /* Make sure we have a safe start */
    *drive[AH][LEFT_MOTOR] = OFF;
    *drive[BH][LEFT_MOTOR] = OFF;
    *drive[AH][RIGHT_MOTOR] = OFF;
    *drive[BH][RIGHT_MOTOR] = OFF;

}

void
int_init() {

    /* Enable high and low level interrupt */
    PMIC.CTRL |= PMIC_HILVLEN_bm;
    PMIC.CTRL |= PMIC_LOLVLEN_bm;

    /* Enable global interrupt */
    sei();
}

void
button_init() {

    /* Configure PE2 and PE3 for input */
    PORTE.DIRCLR = PIN2_bm;
    PORTE.DIRCLR = PIN3_bm;
}

void
led_init() {

    /* Set Green LED-pin PD4 as output */
    PORTD.DIRSET = PIN4_bm;

    /* Set Red LED-pin PD5 as output */
    PORTD.DIRSET = PIN5_bm;

    /* Invert the LED pins, so the LEDs light when we set them high */
    PORTD.PIN4CTRL |= PORT_INVEN_bm;
    PORTD.PIN5CTRL |= PORT_INVEN_bm;
}

void
uart_init(void) {

    /* Set PD3 (TX) as output */
    PORTD.DIRSET = PIN3_bm;

    /* Set the baud rate */
    uint16_t bsel = BSEL_D(BAUD_D);
    USARTD0.BAUDCTRLB = (bsel >> 8) & 0xFF;
    USARTD0.BAUDCTRLA = (uint8_t) bsel;

    /* Enable low level UART receive interrupt */
    USARTD0.CTRLA = USART_RXCINTLVL_LO_gc;

    /* Enable the UART transmitter and receiver */
    USARTD0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;

    /* Asynchronous USART, no parity, 1 stop bit, 8 data bits */
    USARTD0.CTRLC = USART_CHSIZE0_bm | USART_CHSIZE1_bm;
}

int
main() {

    // FIXME: Save this in eeprom and add calibration button
    min_pulse_width[LEFT_STICK] = -3431L;
    max_pulse_width[LEFT_STICK] =  3379L;

    min_pulse_width[RIGHT_STICK] = -3423L;
    max_pulse_width[RIGHT_STICK] =  3385L;

    clock_init();
    rc_init();
    fsm_init();
    pwm_init();
    int_init();
    button_init();
    led_init();
    uart_init();

    //PORTD.OUTSET = PIN4_bm;
    //PORTD.OUTSET = PIN5_bm;

    for(;;) {
    }
}

void
calibrate_stick(int16_t * pulse_width, int8_t stick) {

    if (*pulse_width < min_pulse_width[stick]) {
        min_pulse_width[stick] = *pulse_width;
    }

    if (*pulse_width > max_pulse_width[stick]) {
        max_pulse_width[stick] = *pulse_width;
    }
}

void
single_sticks() {

    drive_motor(LEFT_MOTOR, left_stick);
    drive_motor(RIGHT_MOTOR, right_stick);
}

void
combo_sticks() {

    int16_t motor_thrust_left;
    int16_t motor_thrust_right;

    if (left_stick == 0 && right_stick != 0) {

        /* Turn on the spot */
        motor_thrust_left  =  right_stick;
        motor_thrust_right = -right_stick;

    } else {

        /* Drive */
        motor_thrust_left  = left_stick + right_stick;
        motor_thrust_right = left_stick - right_stick;
    }

    /* Remove overflows */
    if (motor_thrust_left  >  127) motor_thrust_left  =  127;
    if (motor_thrust_left  < -127) motor_thrust_left  = -127;
    if (motor_thrust_right >  127) motor_thrust_right =  127;
    if (motor_thrust_right < -127) motor_thrust_right = -127;

    drive_motor(LEFT_MOTOR,  (int8_t) motor_thrust_left);
    drive_motor(RIGHT_MOTOR, (int8_t) motor_thrust_right);
}

void
drive_motor(int8_t motor, int8_t motor_thrust ) {

    /* Scale the pwm-value */
    uint16_t pwm = (abs(motor_thrust)*PER)/127L;

    if (motor_thrust > 0) {

        /* Drive forward */
        *drive[AH][motor] = OFF;
        *drive[BH][motor] = pwm;

    } else {

        /* Drive backward */
        *drive[AH][motor] = pwm;
        *drive[BH][motor] = OFF;
    }
}

int8_t
normalize_stick_position(int16_t * pulse_width, uint8_t stick) {

    int16_t stick_position;

    /* Center around 1.5 ms */
    *pulse_width -= sec_to_tic(CENTER_LEN);

    /* Dead band */
    if (abs(*pulse_width) <= sec_to_tic(DEAD_BAND)) {
        stick_position = 0;

    /* Scale to -127..127 */
    } else if (*pulse_width >= 0) {

        stick_position = (*pulse_width * 127L)/max_pulse_width[stick];

    } else {

        stick_position = -(*pulse_width * 127L)/min_pulse_width[stick];
    }

    /* 125..127 -> 127 */
    if (stick_position >= 125) {
        stick_position = 127;
    }

    return (int8_t) stick_position;
}

ISR(TCC1_CCA_vect) {

    number_of_runs_without_rc_command = 0;

    if (rc_receiver_ready == TRUE) {
        left_stick = normalize_stick_position(&TCC1_CCA, LEFT_STICK);

    } else {

        /* Skip the first interrupts since the receiver is not stable then */
        if (++number_of_rc_commands >= 200000) {
            PORTD.OUTSET = PIN4_bm;
            rc_receiver_ready = TRUE;
        }
    }
}

ISR(TCC1_CCB_vect) {
    if (rc_receiver_ready == TRUE) {
        right_stick = normalize_stick_position(&TCC1_CCB, RIGHT_STICK);
    }
}

ISR(TCD0_OVF_vect) {

    /* Timeout if we do not get any rc commands */
    if (++number_of_runs_without_rc_command >= 20) {
        PORTD.OUTCLR = PIN4_bm;
        rc_receiver_ready = FALSE;
        number_of_rc_commands = 0;
    }

    if (rc_receiver_ready == TRUE) {
    //if (PORTE.IN & PIN2_bm)
        //single_sticks();
    //else
        combo_sticks();
    }
}

ISR(USARTD0_RXC_vect) {
    if (rc_receiver_ready == FALSE) {

        uint8_t motor;
        int8_t direction;

        uint8_t command = USARTD0.DATA;

        /* Get the two MSB of command */
        switch (command >> 6) {

            case 0:
                motor = LEFT_MOTOR;
                direction = BACKWARD;
                break;

            case 1:
                motor = LEFT_MOTOR;
                direction = FORWARD;
                break;

            case 2:
                motor = RIGHT_MOTOR;
                direction = BACKWARD;
                break;

            case 3:
                motor = RIGHT_MOTOR;
                direction = FORWARD;
                break;
        }

        /*
         * Drive the left or right motor, with the correct direction and speed.
         *
         * 0x3f = 0b00111111 and masks out the speed from the command.
         * This speed (0..63) is then scaled to (0..127) by multiplying by two and adding one.
         * The direction variable is positiv for forward and negative for backward
         * direction, and thus give the final speed as an int8_t of range -127..127
         *
         */
        drive_motor(motor, direction*(command & 0x3f)*2 + 1 );
    }
}
