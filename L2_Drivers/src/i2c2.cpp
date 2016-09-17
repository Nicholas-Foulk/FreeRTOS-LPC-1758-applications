/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

#include "I2C2.hpp"
#include "LPC17xx.h"



/**
 * IRQ Handler needs to be enclosed in extern "C" because this is C++ file, and
 * we don't want C++ to "mangle" this function name.
 * This ISR Function need needs to be named precisely to override "WEAK" ISR
 * handler defined at startup.cpp
 */
extern "C"
{
    void I2C2_IRQHandler()
    {
        I2C2::getInstance().handleInterrupt();
    }
}


//void I2C2::SlaveInit(uint8_t SlaveADDR, uint8_t *buff1,uint32_t sizzbuff1)
////uint8_t I2C2::SlaveInit(uint8_t SlaveADDR, uint8_t *buff1[256])
//
//{
//    //i2cStateMachine();
//    //my I2C2 code?
//      uint32_t clockspeed;
//      uint8_t buff;
//      clockspeed = sys_get_cpu_clock()/8;
//      LPC_SC->PCONP &= ~(0 << 26);   //pg 63
//
//      LPC_SC->PCLKSEL1 &= ~(3 << 20);
//      LPC_SC->PCLKSEL1 |= (3 << 20);  //pg 57
//
//      LPC_PINCON->PINSEL0 &= ~(3 << 20);
//      LPC_PINCON->PINSEL0 |= (2 << 20);
//      LPC_PINCON->PINSEL0 &= ~(3 << 22);
//      LPC_PINCON->PINSEL0 |= (2 << 22);
//
//      LPC_PINCON->PINMODE0 &= ~(0xF << 20); // Both pins with Pull-Up Enabled
//      LPC_PINCON->PINMODE0 |=  (0xA << 20); // Disable both pull-up and pull-down
//                                            //These were taken from the original init function of I2C2
//
//      LPC_PINCON->PINMODE_OD0 &= ~(1<<10);
//      LPC_PINCON->PINMODE_OD0 &= ~(1<<11);  //use for open drain collector
//
//      NVIC_EnableIRQ(I2C2_IRQn);  //enable interrupt request handler.
//
//      LPC_I2C2->I2CONCLR = 0x00;
//      LPC_I2C2->I2ADR0 = SlaveADDR;
//      LPC_I2C2->I2STAT = 0xF8;
//
//      //here is SCLL and SCLH
//      clockspeed = sys_get_cpu_clock()/8;  //this is the same a pclk in I2C_Base
//      const uint32_t percent_high = 40;
//      const uint32_t percent_low = (100 - percent_high);
//      //const uint32_t freq_hz = (busRateInKhz > 1000) ? (100 * 1000) : (busRateInKhz * 1000);
//      const uint32_t freq_hz = (38000 > 1000) ? (100 * 1000) : (38000 * 1000);
//      //const uint32_t half_clock_divider = (pclk / freq_hz) / 2;
//      const uint32_t half_clock_divider = (clockspeed/ freq_hz) / 2;
//
//
//      LPC_I2C2->I2SCLH = (half_clock_divider * percent_high) / 100;   //determines the high time of the I2C clock
//      LPC_I2C2->I2SCLL = (half_clock_divider * percent_low ) / 100;   //determines the low time of the I2C clock
//
//
//      LPC_I2C2->I2CONSET = 0x44; //slave enable bits pg 466.
//
//      buff=LPC_I2C2->I2DATA_BUFFER;
//
//      sizzbuff1--;
//
//      if(1 == sizzbuff1)
//      { // Only 1 more byte remaining
//          LPC_I2C2->I2CONCLR = (1<<2); //im not sure this right
//      }
//      else
//      {
//          LPC_I2C2->I2CONSET = (1<<2); // ACK next byte --> Next state: dataAvailableAckSent(back to this state)
//      }
//
//}
//void I2C2::SlaveReg(uint8_t slaveADDR)
//{
//     uint32_t clockspeed;
//     uint8_t buff;
//     //LPC_I2C2->I2CONSET = 0x00;   //assuming these are all 8 bits
////     LPC_I2C2->I2STAT = 0xF8;
////     LPC_I2C2->I2DAT = 0x00;
////     LPC_I2C2->I2ADR0 = 0x11;   //slave address
////
//     LPC_I2C2->I2CONCLR = 0x00;
//
//     clockspeed = sys_get_cpu_clock()/8;  //this is the same a pclk in I2C_Base
//     const uint32_t percent_high = 40;
//     const uint32_t percent_low = (100 - percent_high);
//     //const uint32_t freq_hz = (busRateInKhz > 1000) ? (100 * 1000) : (busRateInKhz * 1000);
//     const uint32_t freq_hz = (38000 > 1000) ? (100 * 1000) : (38000 * 1000);
//     //const uint32_t half_clock_divider = (pclk / freq_hz) / 2;
//     const uint32_t half_clock_divider = (clockspeed/ freq_hz) / 2;
//
//
//     LPC_I2C2->I2SCLH = (half_clock_divider * percent_high) / 100;   //determines the high time of the I2C clock
//     LPC_I2C2->I2SCLL = (half_clock_divider * percent_low ) / 100;   //determines the low time of the I2C clock
//
//     //LPC_I2C2->I2CONCLR = 0x00;
//     //LPC_I2C2->I2DATA_BUFFER = 0x00;
//
//     LPC_I2C2->I2STAT = 0xF8;
//     //LPC_I2C2->I2DAT = 0x00;
//     LPC_I2C2->I2ADR0 = slaveADDR;   //slave address
//     //LPC_I2C2->I2ADR0 = 0x11;   //slave address
//     LPC_I2C2->I2CONSET = 0x44;  //slave enable bits pg 466.
//
//
//     buff=LPC_I2C2->I2DAT;
//
//     LPC_I2C2->I2CONSET = 0x44;
//
//
//}

bool I2C2::init(unsigned int speedInKhz)
{
    /**
     * Before I2C is initialized, check to be sure that the I2C wires are logic "1"
     * means that they are pulled high, otherwise there may be a short circuit.
     *
     * I2C2 is on P0.10, and P0.11
     */
    const uint32_t i2c_pin_mask = ( (1<<10) | (1<<11) );
    const bool i2c_wires_are_pulled_high = (i2c_pin_mask == (LPC_GPIO0->FIOPIN & i2c_pin_mask) );

    LPC_PINCON->PINMODE0 &= ~(0xF << 20); // Both pins with Pull-Up Enabled
    LPC_PINCON->PINMODE0 |=  (0xA << 20); // Disable both pull-up and pull-down

    // Enable Open-drain for I2C2 on pins P0.10 and P0.11
    LPC_PINCON->PINMODE_OD0 |= i2c_pin_mask;

    LPC_PINCON->PINSEL0 &= ~(0xF << 20);  // Clear
    LPC_PINCON->PINSEL0 |=  (0xA << 20);  // Enable I2C Pins: SDA, SCL

    lpc_pclk(pclk_i2c2, clkdiv_8);
    const uint32_t pclk = sys_get_cpu_clock() / 8;

    /**
     * I2C wires should be pulled high for normal operation, so if they are, initialize I2C
     * otherwise disable operations on I2C since I2C has a likely hardware BUS fault such as:
     *  - I2C SDA/SCL with no pull-up
     *  - I2C SDA/SCL shorted to ground
     */
    if (i2c_wires_are_pulled_high) {
        return I2C_Base::init(pclk, speedInKhz);
    }
    else {
        disableOperation();
        return false;
    }
}

I2C2::I2C2() : I2C_Base((LPC_I2C_TypeDef*) LPC_I2C2_BASE)
{

}
