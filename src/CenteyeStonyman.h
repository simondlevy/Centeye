/*********************************************************************/
/*********************************************************************/
//	ArduEye_SMH_v1.h
//	ArduEye Library for the Stonyman/Hawksbill Centeye Vision Chips
//	
//	Basic functions to operate Stonyman/Hawksbill with ArduEye boards
//	Supports all Arduino boards that use the ATMega 8/168/328/2560
//	NOTE: ATMega 2560 SPI for external ADC is not supported.
//
//	Working revision started July 9, 2012
//
/*********************************************************************/
/*********************************************************************/

/*
===============================================================================
Copyright (c) 2012 Centeye, Inc. 
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, 
    this list of conditions and the following disclaimer.
    
    Redistributions in binary form must reproduce the above copyright notice, 
    this list of conditions and the following disclaimer in the documentation 
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY CENTEYE, INC. ``AS IS'' AND ANY EXPRESS OR 
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
EVENT SHALL CENTEYE, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are 
those of the authors and should not be interpreted as representing official 
policies, either expressed or implied, of Centeye, Inc.
===============================================================================
*/

#ifndef _ARDUEYE_SMH_H_INCLUDED
#define _ARDUEYE_SMH_H_INCLUDED

#include <stdint.h>

/*********************************************************************/
//SMH System Registers

#define SMH_SYS_COLSEL 0	//select column
#define SMH_SYS_ROWSEL 1	//select row
#define SMH_SYS_VSW 2		//vertical switching
#define SMH_SYS_HSW 3		//horizontal switching
#define SMH_SYS_VREF 4		//voltage reference
#define SMH_SYS_CONFIG 5	//configuration register
#define SMH_SYS_NBIAS 6		//nbias
#define SMH_SYS_AOBIAS 7	//analog out bias

/*********************************************************************/
//default values

// Supply voltage types
// Notation: AVB is A.B volts. e.g. 5V0 is 5V, 3V3 is 3.3V, etc.
#define SMH1_VDD_5V0  1

#define SMH_VREF_5V0 30		//vref for 5 volts
#define SMH_NBIAS_5V0 55	//nbias for 5 volts
#define SMH_AOBIAS_5V0 55	//aobias for 5 volts

#define SMH_GAIN_DEFAULT 0	//no amp gain
#define SMH_SELAMP_DEFAULT 0	//amp bypassed

/*********************************************************************/
// ADC types

// ARDUINO ONBOARD ADC
#define SMH1_ADCTYPE_ONBOARD 0
// MCP3201, Microchip, 12bits, 100ksps
#define SMH1_ADCTYPE_MCP3201 1
// MCP3201, Microchip, 12bits, 100ksps, for ArduEye Bug v1.0
#define SMH1_ADCTYPE_MCP3201_2 2
// MCP3001, Microchip, 10bits, 200ksps
#define SMH1_ADCTYPE_MCP3001 3

/*********************************************************************/

#if defined (__AVR_ATmega8__)||(__AVR_ATmega168__)|  	(__AVR_ATmega168P__)||(__AVR_ATmega328P__)

//uno can handle 10x10 array and FPN mask
//so we set SKIP_PIXELS=8 to downsample by 8
//and START_ROW/COL to 16 to center the resulting
//80x80 raw grid on the Stonyman raw 112x112 array.

#define MAX_ROWS    10    
#define MAX_COLS    10
#define SKIP_PIXELS 8
#define START_ROW   16
#define START_COL   16
#define START_PIXEL 8  

#else

//If not Uno, assume a more modern board like Mega 2560,
//which can handle 48x48 array and FPN mask
//so we set SKIP_PIXELS=2 to downsample by 2
//and START_PIXELS at row/col 8.  This gives us
//a 48x48 array with superpixels of 2x2.  With the
//start row and col at 8, the 96x96 raw grid is 
//centered on the Stonyman 112x112 raw array.

#define MAX_ROWS    16
#define MAX_COLS    16
#define SKIP_PIXELS 4
#define START_ROW   24
#define START_COL   24  
#define START_PIXEL 18

#endif

#define MAX_PIXELS (MAX_ROWS*MAX_COLS)

/*********************************************************************/
//	ArduEyeSMH
/*********************************************************************/


class ArduEyeSMH 
{
    private:

        //indicates whether amplifier is in use	
        char useAmp;

        // input pins
        uint8_t _resp;
        uint8_t _incp;
        uint8_t _resv;
        uint8_t _incv;
        uint8_t _inphi;

        void init_pin(uint8_t pin);

    public:

        // Constructor
        ArduEyeSMH(uint8_t resp, uint8_t incp, uint8_t resv, uint8_t incv, uint8_t inphi=0);

        /*********************************************************************/
        // Initialize the vision chip for image readout

        void begin(short vref=30,
                short nbias=40,
                short aobias=40,
                char selamp=SMH_SELAMP_DEFAULT); 

        /*********************************************************************/
        // Chip Register and Value Manipulation

        //set pointer on chip
        void setPointer(char ptr);

        //set value of current pointer
        void setValue(short val);

        //increment value of current pointer
        void incValue(short val);

        //pulse INPHI to operate amplifier
        void pulseInphi(char delay); 

        //clear all register values
        void clearValues(void);

        //set pointer to register and set value for that register
        void setPointerValue(char ptr,short val);

        //set configuration register on chip
        void setConfig(char gain, char selamp,char cvdda=1);

        //select amp and set amp gain
        void setAmpGain(char gain);

        //set analog input to Arduino for onboard ADC
        void setAnalogInput(char analogInput);

        //set external ADC input
        void setADCInput(char ADCInput,char state);

        //set hsw and vsw registers to bin on-chip
        void setBinning(short hbin,short vbin);

        /*********************************************************************/
        // Bias functions

        //set individual bias values
        void setVREF(short vref);
        void setNBIAS(short nbias);
        void setAOBIAS(short aobias);

        //set biases based on Vdd
        void setBiasesVdd(char vddType);

        //set all bias values
        void setBiases(short vref,short nbias,short aobias);


        /*********************************************************************/
        // Image Functions

        //given an image, returns a fixed-pattern noise mask and mask_base
        void calcMask(short *img, short size, uint8_t *mask, short *mask_base);

        //applies pre-calculated FPN mask to an image
        void applyMask(short *img, short size, uint8_t *mask, short mask_base);

        //gets an image from the vision chip
        void getImage(
                short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                char ADCType,
                char anain);

        //gets a image from the vision chip, sums each row and returns one pixel for the row
        void getImageRowSum(
                short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                char ADCType,
                char anain);

        //gets a image from the vision chip, sums each col and returns one pixel for the col
        void getImageColSum(short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                char ADCType,char anain);

        //takes an image and returns the maximum value row and col
        void findMax(
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                char ADCType,
                char anain,
                uint8_t *max_row, 
                uint8_t *max_col);

        //prints the entire vision chip over serial as a Matlab array
        void chipToMatlab(char whichchip,char ADCType,char anain);

        //prints a section of the vision chip over serial as a Matlab array
        void sectionToMatlab(
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                char ADCType, 
                uint8_t anain);   

};

#endif
