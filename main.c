//******************************************************************************
//           10F322
//          ----------
// IR   RA0 |1      6| RA3  SW2
//      Vss |2      5| Vdd
//  LED RA1 |3      4| RA2  SW1
//          ----------
//
//******************************************************************************

//******************************************************************************
// notes
//******************************************************************************
/*

*/


//******************************************************************************
// includes
//******************************************************************************
#include <xc.h>
#include <stdbool.h>
#include <stdint.h>



//******************************************************************************
// defines
//******************************************************************************
//my pin macros
#define _PIN_SET(pt,pn,m)           LAT##pt##pn = m
#define _PIN_TOGGLE(pt,pn)          LAT##pt##pn = ~LAT##pt##pn
#define _PIN_VALUE(pt,pn)           R##pt##pn
#define _PIN_IO(pt,pn,m)            TRIS##pt##pn = m
#define _PIN_PU(pt,pn,m)            WPU##pt##pn = m
//instead of using ANS##pt##pn = m; , use byte method on ANSELx register
//as not all pins analog - this will result in set/clear of a ANSELx bit
//(if bit not implemented, no harm done and will keep PIN_INIT always happy)
#define _PIN_MODE(pt,pn,m)            ANSEL##pt |= m<<pn; ANSEL##pt &= ~((!m)<<pn)
#define _PIN_INIT(pt,pn,m,io,pu,v)    _PIN_SET(pt,pn,v); _PIN_PU(pt,pn,pu); \
                                    _PIN_MODE(pt,pn,m); _PIN_IO(pt,pn,io)
//need to expand pp
#define PIN_SET(pp,m)               _PIN_SET(pp,m)
#define PIN_TOGGLE(pp)              _PIN_TOGGLE(pp)
#define PIN_VALUE(pp)               _PIN_VALUE(pp)
#define IO_IN                       1
#define IO_OUT                      0
#define PIN_IO(pp,m)                _PIN_IO(pp,m)
#define PU_ON                       1
#define PU_OFF                      0
#define PIN_PU(pp,m)                _PIN_PU(pp,m)
#define MODE_ANA                    1
#define MODE_DIG                    0
#define PIN_MODE(pp,m)              _PIN_MODE(pp,m)
#define PIN_INIT(pp,m,io,p,v)       _PIN_INIT(pp,m,io,p,v)

//IOC macros
#define _IOC_NCHANGE(pt,pn)         IOC##pt##Nbits.IOC##pt##N##pn
#define _IOC_FLAG(pt,pn)            IOC##pt##Fbits.IOC##pt##F##pn
#define IOC_NCHANGE(pp)             _IOC_NCHANGE(pp)
#define IOC_FLAG(pp)                _IOC_FLAG(pp)

//pin assignment
#define SWDN_PIN                    A,2
#define SWUP_PIN                    A,3
#define LED_PIN                     A,1
#define IR_PIN                      A,0


//sw1/2 macros
#define SWDN_PRESSED                (PIN_VALUE(SWDN_PIN) == 0)
#define SWDN_NOT_PRESSED            (PIN_VALUE(SWDN_PIN) == 1)
#define SWUP_PRESSED                (PIN_VALUE(SWUP_PIN) == 0)
#define SWUP_NOT_PRESSED            (PIN_VALUE(SWUP_PIN) == 1)

//clock freq
#define _XTAL_FREQ                  8000000

//ir carrier freq
#define IR_FREQ                     37917   //37916.666
#define IR_NEC_PR2                  52      //8000000/4/1/37917=52.74=53=0.47% error
                                            //actual freq = 8000000 / 4 / 53 = 37735hz
                                            //actual period = 26.5us
#define IR_T1                       21      //first half of every bit, seond half of bit0
#define IR_T2                       63      //second half of bit1
#define IR_TS1                      339     //start- on
#define IR_TS2                      170     //start- off
#define IR_TIDLE                    1492    //idle time
#define IR_TIDLER                   3630    //idle time after repeat
#define IR_TFRAME                   4077    //total frame time (108ms)

#define LG_ADDR                     4       //lg remote address
#define LG_CHUP                     0
#define LG_CHDN                     1
#define LG_VOLUP                    2
#define LG_VOLDN                    3
#define LG_ONOFF                    8       //on/off (toggle power)
#define LG_ON                       0xC4    //on only
#define LG_OFF                      0xC5    //off only



//******************************************************************************
// pause ms milliseconds (cpu keeps running)
//******************************************************************************
void pause(uint16_t ms)
{
    for( ; ms; ms-- ) __delay_ms(1);
}

//******************************************************************************
// blink led1 on for ms milliseconds (0-255)
//******************************************************************************
//void blink_led( uint8_t ms )
//{
//    PIN_SET( LED_PIN, 1 );
//    pause( ms );
//    PIN_SET( LED_PIN, 0 );
//}


//******************************************************************************
// set PWM duty - 1=on (25%) 0=off
// ir pin = pwm1  led = pwm2
// led mirrors ir output
//******************************************************************************
void pwm_duty(bool on)
{
    if( on ){ //25% duty
        PWM1DCL = PR2;
        PWM1DCH = PR2 >> 2;
        PWM2DCL = PR2;
        PWM2DCH = PR2 >> 2;
    }
    else{ //0% duty
        PWM1DCL = 0;
        PWM1DCH = 0;
        PWM2DCL = 0;
        PWM2DCH = 0;
    }
}

//******************************************************************************
// init ir pwm and led pwm
// timer2 stops in sleep, so pwm will be off when in sleep
//******************************************************************************
void pwm_init(uint8_t pr2_val)
{
    //    uint8_t i = 0;
    //    uint32_t x = _XTAL_FREQ >> 2; // fosc/4/prescale(1)
    //    for( ; x > freq; i++, x -= freq ) //manual 'divide' (keep out divide code)
    //    if( (x >> 1) > freq) i++; //round up
    //    PR2 = i;
    //PR2 = (uint8_t)((_XTAL_FREQ >> 2) / freq) - 1;
    PR2 = pr2_val;    //timer2 period  _XTAL_FREQ / 4 / prescale / freq
    //duty cycle 0%
    //output remains low with 0% duty
    pwm_duty(0);
    PWM1CONbits.PWM1EN = 1; //ir
    PWM1CONbits.PWM1OE = 1;
    PWM2CONbits.PWM2EN = 1; //led
    PWM2CONbits.PWM2OE = 1;
    TMR2IF = 0; //clear overflow flag
    TMR2 = 0; //clear timer
    TMR2ON = 1; //timer2 on
    //pwm now started - output remains low
}

//******************************************************************************
// turn off ir pwm
//******************************************************************************
void ir_disable()
{
    //PWM1CONbits.PWM1EN = 0; //pwm off
    //PWM1CONbits.PWM1OE = 0; //now using LAT
    pwm_duty(0); //make sure next pwm is 0% duty (off)
    while( !TMR2IF ); //wait for previous pwm
    TMR2ON = 0; //timer2 off
    TMR2IF = 0; //clear overflow flag
}

//******************************************************************************
// nec pulse on (b==1) or off (b==0) for c1 cycles (25% duty)
// will always leave output off when done (at next cycle)
// (which means when this function called if b=0- no need to set duty)
// if 2 pulse on's in a row, need to call this before timer2 overflow
// (should not be a problem)
//******************************************************************************
void ir_pulse(uint16_t c1, bool b)
{
    if(b){ //on
        //25% duty (will be loaded at next overflow)
        pwm_duty(1);
    }
    for( ; c1; c1--, TMR2IF = 0 ) while( !TMR2IF );
    //0% duty - output off (will be loaded at next overflow)
    //any other value loaded before tmr2if will override 0
    pwm_duty(0);
}

//******************************************************************************
// nec ir frame end pulse (pass 1 to complete frame, 0 to invalidate)
// nec end pulse - bit 0 pulse
// nec idle
//******************************************************************************
void ir_frame_nec_end(bool y)
{
    if( y ) ir_pulse( IR_T1, 1 ); //end pulse
    ir_pulse( IR_TIDLE, 0 ); //idle
    ir_disable();
}

//******************************************************************************
// nec ir frame (without end pulse)
// start pulse, address, naddress, data, ndata, end pulse, idle
// nec start pulse - 9ms on, 4.5ms off
// nec bit 0 pulse - 562.5us on 562.5us off - total 1.125ms
// nec bit 1 pulse - 562.5us on 1.675ms off - total 2.225ms
//******************************************************************************
void ir_frame_nec(uint8_t a, uint8_t d)
{
    union { uint8_t dat4[4]; uint32_t dat32; } u;
    u.dat4[0] = a; u.dat4[1] = ~a; u.dat4[2] = d; u.dat4[3] = ~d;
    pwm_init( IR_NEC_PR2 );
    ir_pulse( IR_TS1, 1 ); //start 9ms on
    ir_pulse( IR_TS2, 0 ); //4.5ms off
    for( uint8_t i = 32; i; i--, u.dat32 >>= 1 ){ //data
        ir_pulse( IR_T1, 1 ); //on time always same
        ir_pulse( u.dat32 & 1 ? IR_T2 : IR_T1, 0 ); //off time
    }
    //ir_pulse( IR_T1, 1 ); //end pulse
    //ir_pulse( IR_TIDLE, 0 ); //idle
    //ir_disable();
}

//******************************************************************************
// send 'on' command
//******************************************************************************
lg_on(void)
{
    ir_frame_nec( LG_ADDR, LG_ON );
    ir_frame_nec_end( 1 );
}


//******************************************************************************
// main
//******************************************************************************
void main(void)
{
    //osc
    OSCCONbits.IRCF = 0b110; //8MHz

    //pins
    OPTION_REGbits.nWPUEN = 0; //enable weak pullups
    PIN_INIT( LED_PIN, MODE_DIG, IO_OUT, PU_OFF, 0);
    PIN_INIT( IR_PIN, MODE_DIG, IO_OUT, PU_OFF, 0);
    PIN_INIT( SWDN_PIN, MODE_DIG, IO_IN, PU_ON, 0);
    PIN_PU( SWUP_PIN, PU_ON ); //ra3/mclr in only

    //main loop
    for( ;; ){

        //wait for release of all switches
        while( SWUP_PRESSED || SWDN_PRESSED );
        pause( 40 ); //debounce

        //wait for sw1/2 wakeup from sleep
        IOC_NCHANGE( SWDN_PIN ) = 1; //enable irq on neg change
        IOC_NCHANGE( SWUP_PIN ) = 1; //enable irq on neg change
        IOC_FLAG( SWDN_PIN ) = 0;    //clear irq flag
        IOC_FLAG( SWUP_PIN ) = 0;    //clear irq flag
        INTCONbits.IOCIE = 1;       //enable irq on change
        SLEEP();                    //sw press will resume
        INTCONbits.IOCIE = 0;       //disable irq on change

        //a switch was pressed, always send 'on'
        //buys some time and any sw press will send 'on'
        lg_on();

        //108ms    elapsed - should be plenty of time for sw debounce

        //check for short press of up or down using only IOC flags
        //in case short press less than 108ms
        //up only
        if( 1 == IOC_FLAG( SWUP_PIN ) && 0 == IOC_FLAG( SWDN_PIN ) )
        {
            ir_frame_nec( LG_ADDR, LG_CHUP ); //start to send chup
            ir_frame_nec_end( SWUP_NOT_PRESSED ); //if released, complete frame
            if( SWUP_NOT_PRESSED ) continue; //released, start over
        }
        //down only
        if( 0 == IOC_FLAG( SWUP_PIN ) && 1 == IOC_FLAG( SWDN_PIN ) )
        {
            ir_frame_nec( LG_ADDR, LG_CHDN ); //start to send chdn
            ir_frame_nec_end( SWDN_NOT_PRESSED ); //if released, complete frame
            if( SWDN_NOT_PRESSED ) continue; //released, start over

        }

        //216ms elapsed

        //send 'on' again
        lg_on();

        //324ms elapsed

        //one or both switches still pressed

        //check for long press up
        if( SWUP_PRESSED && SWDN_NOT_PRESSED ){
            while( SWUP_PRESSED )
            {
                ir_frame_nec( LG_ADDR, LG_VOLUP );
                ir_frame_nec_end( 1 );
                lg_on();
            }
            continue; //released, start over
        }
        //check for long press down
        if( SWDN_PRESSED && SWUP_NOT_PRESSED ){
            while( SWDN_PRESSED ){
                ir_frame_nec( LG_ADDR, LG_VOLDN );
                ir_frame_nec_end( 1 );
                lg_on();
            }
            continue; //released, start over
        }

        //check for long press both
        while( SWDN_PRESSED && SWUP_PRESSED )
        {
            ir_frame_nec( LG_ADDR, LG_OFF );
            ir_frame_nec_end( 1 );
        }



//        for( uint8_t i = 0; SWDN_PRESSED || SWUP_PRESSED;  ){
//            //turn on while sw1 pressed < ~1sec
//            //harmless if already on (on-only command)
//            if( i < 10 ) ir_frame_nec( LG_ADDR, LG_ON );
//            //long press > ~2sec send off command
//            else if( i > 20 ) ir_frame_nec( LG_ADDR, LG_OFF );
//            //nothing done, pause frame time
//            else pause( 100 );
//
//            if( i < 255 ) i++;
//        }
//
//        pause( 100 );

    }

}


#if 0

//ir_frame_nec( LG_ADDR, LG_OFF );
#define LG_CHUP                    0
#define LG_CHDN                    1
#define LG_VOLUP                2
#define LG_VOLDN                3

SW_UP_PRESSED
SW_UP_NOT_PRESSED
SW_DN_PRESSED
SW_DN_NOT_PRESSED

void ir_frame_nec_end(bool y)
{
    if( y ) ir_pulse( IR_T1, 1 ); //end pulse
    ir_pulse( IR_TIDLE, 0 ); //idle
    ir_disable();
}
void ir_frame_nec(uint8_t a, uint8_t d)
{
    uint8_t i = 32;
    union { uint8_t dat4[4]; uint32_t dat32; } u;
    u.dat4[0] = a; u.dat4[1] = ~a; u.dat4[2] = d; u.dat4[3] = ~d;
    blink_led( 5 );
    ir_init( IR_NEC_PR2 );
    ir_pulse( IR_TS1, 1 ); //start 9ms on
    ir_pulse( IR_TS10, 0 ); //4.5ms off
    for( ; i; i--, u.dat32 >>= 1 ){ //data
        ir_pulse( IR_T1, 1 ); //on time always same
        ir_pulse( u.dat32 & 1 ? IR_T11 : IR_T1, 0 ); //off time
    }
}


for( ; ; )
{
    //wait for release of all switches
    while( SW_UP_PRESSED || SW_DN_PRESSED );
    pause( 20 ) //debounce

    //wait for a switch press
    wait_sw();

    //a switch was pressed, send 'on'
    ir_frame_nec( LG_ADDR, LG_ON );
    ir_frame_nec_end( 1 );

    //108ms

    //check for short press of up or down
    if( SW_UP_PRESSED && SW_DN_NOT_PRESSED ){
        ir_frame( LG_ADDR, LG_CHUP );
        ir_frame_nec_end( SW_UP_NOT_PRESSED ); //if released, complete frame
        if( SW_UP_NOT_PRESSED ) continue;
    }
    if( SW_DN_PRESSED && SW_UP_NOT_PRESSED ){
        ir_frame( LG_ADDR, LG_CHDN );
        ir_frame_nec_end( SW_DN_NOT_PRESSED ); //if released, complete frame
        if( SW_DN_NOT_PRESSED ) continue;
    }

    //216ms

    //send 'on' again
    ir_frame_nec( LG_ADDR, LG_ON );
    ir_frame_nec_end( 1 );

    //324ms

    //check for long press up
    if( SW_UP_PRESSED && SW_DN_NOT_PRESSED ){
        while( SW_UP_PRESSED ) ir_frame( LG_ADDR, LG_VOLUP );
        continue;
    }
    //check for long press down
    if( SW_DN_PRESSED && SW_UP_NOT_PRESSED ){
        while( SW_DN_PRESSED ) ir_frame( LG_ADDR, LG_VOLDN );
        continue;
    }

    //check for long press both
    while( SW_DN_PRESSED && SW_UP_PRESSED ) ir_frame( LG_ADDR, LG_OFF );



}



#endif
