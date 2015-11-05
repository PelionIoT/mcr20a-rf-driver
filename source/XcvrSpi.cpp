/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
* \file XcvrSpi.c
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/*****************************************************************************
*                               INCLUDED HEADERS                            *
*---------------------------------------------------------------------------*
* Add to this section all the headers that this module needs to include.    *
* Note that it is not a good practice to include header files into header   *
* files, so use this section only if there is no other better solution.     *
*---------------------------------------------------------------------------*
*****************************************************************************/

#include "mbed-drivers/mbed.h"

#if defined(TARGET_K64F)
  SPI spi(PTD2, PTD3, PTD1);
#elif defined(TARGET_NUCLEO_F401RE)
  SPI spi(SPI_MOSI, SPI_MISO, SPI_SCK);
#else
  "SPI not defined for this platform"
#endif

/* FREESCALE MCR20A pins */
DigitalOut RF_CS(D10);
DigitalOut RF_RST(D5);          
InterruptIn RF_IRQ (D2);
DigitalIn RF_IRQ_PIN (D2);

//gcapraru 
extern Serial pc;


extern "C" void PHY_InterruptHandler(void);


extern "C" void xcvr_dbg(char* s)
{
    pc.printf(s);    
}

/*****************************************************************************/
/*****************************************************************************/
extern "C" void xcvr_spi_init(uint32_t instance)
{
    (void)instance;
}

/*****************************************************************************/
/*****************************************************************************/
extern "C" void RF_IRQ_Init(void) {
    RF_IRQ.mode(PullUp);
    RF_IRQ.fall(&PHY_InterruptHandler);
}

/*****************************************************************************/
/*****************************************************************************/
extern "C" void RF_IRQ_Enable(void) {
    RF_IRQ.enable_irq();
}

/*****************************************************************************/
/*****************************************************************************/
extern "C" void RF_IRQ_Disable(void) {
    RF_IRQ.disable_irq();
}

/*****************************************************************************/
/*****************************************************************************/
extern "C" uint8_t RF_isIRQ_Pending(void) {
        return !RF_IRQ_PIN.read();
}

/*****************************************************************************/
/*****************************************************************************/
extern "C" void RF_RST_Set(int state) {
    RF_RST = state;
}

/*****************************************************************************/
/*****************************************************************************/
extern "C" void gXcvrAssertCS_d(void)
{
    RF_CS = 0;
}

/*****************************************************************************/
/*****************************************************************************/
extern "C" void gXcvrDeassertCS_d(void)
{
    RF_CS = 1;
}

/*****************************************************************************/
/*****************************************************************************/
extern "C" void xcvr_spi_configure_speed(uint32_t instance, uint32_t freq)
{
    (void)instance;
    spi.frequency(freq);
}

/*****************************************************************************/
/*****************************************************************************/
extern "C" void xcvr_spi_transfer(uint32_t instance,
                         uint8_t * sendBuffer,
                         uint8_t * receiveBuffer,
                         size_t transferByteCount)
{
    (void)instance;
    volatile uint8_t dummy;

    if( !transferByteCount )
        return;

    if( !sendBuffer && !receiveBuffer )
        return;

    while( transferByteCount-- )
    {
        if( sendBuffer )
        {
            dummy = *sendBuffer;
            sendBuffer++;
        }
        else
        {
            dummy = 0xFF;
        }

        dummy = spi.write(dummy);
        
        if( receiveBuffer )
        {
            *receiveBuffer = dummy;
            receiveBuffer++;
        }
    }
}
