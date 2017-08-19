/*
Stonyman.h Library for the Stonyman Centeye Vision Chip

Basic functions to operate Stonyman/Hawksbill with ArduEye boards
Supports all Arduino boards that use the ATMega 8/168/328/2560
NOTE: ATMega 2560 SPI for external ADC is not supported.

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
*/

#ifndef _STONYMAN_INCLUDED
#define _STONYMAN_INCLUDED


#include <stdint.h>

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

/**
 *	Stonyman
 */
class Stonyman 
{
    public:

        /**
         * Constructor
         */
        Stonyman(uint8_t resp, uint8_t incp, uint8_t resv, uint8_t incv, uint8_t inphi=0);

        /**
        * Initializes the vision chips for normal operation.  Sets vision
        * chip pins to low outputs, clears chip registers, sets biases and
        * config register.  If no parameters are passed in, default values
        * are used.
        */
        void begin( short vref=30, short nbias=40, short aobias=40, bool selamp=false); 

        /*********************************************************************/
        //	setConfig
        //	Sets configuration register
        //	cvdda:  (1) connect vdda, always should be connected
        //	selamp: (0) bypasses amplifier, (1) connects it
        //	gain: amplifier gain 1-7
        //	EXAMPLE 1: To configure the chip to bypass the amplifier:
        //	setConfig(0,0,1);
        //	EXAMPLE 2: To use the amplifier and set the gain to 4:
        //	setConfig(4,1,1);
        /*********************************************************************/
        void setConfig(uint8_t gain, uint8_t selamp,uint8_t cvdda=1);

        /*********************************************************************/
        //	setAmpGain
        //	A friendlier version of setConfig.  If amplifier gain is set to 
        //	zero, amplifier is bypassed.  Otherwise the appropriate amplifier
        //	gain (range 1-7) is set.
        /*********************************************************************/
        void setAmpGain(uint8_t gain);

        /*********************************************************************/
        //	setBinning
        //	Configures binning in the focal plane using the VSW and HSW
        //	system registers. The super pixels are aligned with the top left 
        //	of the image, e.g. "offset downsampling" is not used. This 
        //	function is for the Stonyman chip only. 
        //	VARIABLES:
        //	hbin: set to 1, 2, 4, or 8 to bin horizontally by that amount
        //	vbin: set to 1, 2, 4, or 8 to bin vertically by that amount
        /*********************************************************************/
        void setBinning(short hbin,short vbin);

        /*********************************************************************/
        // Bias functions

        /*********************************************************************/
        //	setVREF
        //	Sets the VREF register value (0-63)
        /*********************************************************************/
        void setVREF(short vref);

        /*********************************************************************/
        //	setNBIAS
        //	Sets the NBIAS register value (0-63)
        /*********************************************************************/
        void setNBIAS(short nbias);

        /*********************************************************************/
        //	setAOBIAS
        //	Sets the AOBIAS register value (0-63)
        /*********************************************************************/
        void setAOBIAS(short aobias);

        /*********************************************************************/
        //	setBiasesVdd
        //	Sets biases based on chip voltage
        /*********************************************************************/
        void setBiasesVdd(uint8_t vddType);

        /*********************************************************************/
        //	setBiases
        //	Sets all three biases
        /*********************************************************************/
        void setBiases(short vref,short nbias,short aobias);


        /*********************************************************************/
        // Image Functions

        /*********************************************************************/
        //	calcMask
        //	Expose the vision chip to uniform texture (such as a white piece
        //	of paper placed over the optics).  Take an image using the 
        //	getImage function.  Pass the short "img" array and the "size"
        //	number of pixels, along with a uint8_t "mask" array to hold
        //	the FPN mask and mask_base for the FPN mask base.  Function will
        //	populate the mask array and mask_base variable with the FPN mask,
        //	which can then be used with the applMask function. 
        /*********************************************************************/
        void calcMask(short *img, short size, uint8_t *mask, short *mask_base);

        /*********************************************************************/
        //	applyMask
        //	given the "mask" and "mask_base" variables calculated in        
        //	calcMask, and a current image, this function will subtract the
        //	mask to provide a calibrated image.
        /*********************************************************************/
        void applyMask(short *img, short size, uint8_t *mask, short mask_base);

        /*********************************************************************/
        //	getImageAnalog
        //	This function acquires a box section of a Stonyman or Hawksbill 
        //  from the analog output, and and saves to image array img.  Note 
        //  that images are read out in 
        //	raster manner (e.g. row wise) and stored as such in a 1D array. 
        //	In this case the pointer img points to the output array. 
        //
        //	VARIABLES: 
        //	img (output): pointer to image array, an array of signed shorts
        //	rowstart: first row to acquire
        //	numrows: number of rows to acquire
        //	rowskip: skipping between rows (useful if binning is used)
        //	colstart: first column to acquire
        //	numcols: number of columns to acquire
        //	colskip: skipping between columns
        //  
        //	
        //	EXAMPLES:
        //	getImageAnalog(img,16,8,1,24,8,1,SMH1_ADCTYPE_ONBOARD,0): 
        //	Grab an 8x8 window of pixels at raw resolution starting at row 
        //	16, column 24, from chip using onboard ADC at input 0
        //	getImage(img,0,14,8,0,14,8,SMH1_ADCTYPE_MCP3201,2): 
        //	Grab entire Stonyman chip when using
        //	8x8 binning. Grab from input 2.
        /*********************************************************************/
        void getImageAnalog(
                short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input);

        void getImageDigital(
                short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input);

        /*********************************************************************/
        //	getImageRowSum
        //	This function acquires a box section of a Stonyman or Hawksbill 
        //	and saves to image array img.  However, each row of the image
        //	is summed and returned as a single value.
        //	Note that images are read out in 
        //	raster manner (e.g. row wise) and stored as such in a 1D array. 
        //	In this case the pointer img points to the output array. 
        //
        //	VARIABLES: 
        //	img (output): pointer to image array, an array of signed shorts
        //	rowstart: first row to acquire
        //	numrows: number of rows to acquire
        //	rowskip: skipping between rows (useful if binning is used)
        //	colstart: first column to acquire
        //	numcols: number of columns to acquire
        //	colskip: skipping between columns
        //	ADCType: which ADC to use, defined ADC_TYPES
        //	ANALOG (0,1,2,3): which analog input to use
        //	
        //	EXAMPLES:
        //	getImage(img,16,8,1,24,8,1,SMH1_ADCTYPE_ONBOARD,0): 
        //	Grab an 8x8 window of pixels at raw resolution starting at row 
        //	16, column 24, from chip using onboard ADC at input 0
        //	getImage(img,0,14,8,0,14,8,SMH1_ADCTYPE_MCP3201,2): 
        //	Grab entire Stonyman chip when using
        //	8x8 binning. Grab from input 2.
        /*********************************************************************/
        void getImageRowSumAnalog(
                short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input);

        void getImageRowSumDigital(
                short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input);

        /*********************************************************************/
        //	getImageColSum
        //	This function acquires a box section of a Stonyman or Hawksbill 
        //	and saves to image array img.  However, each col of the image
        //	is summed and returned as a single value.
        //	Note that images are read out in 
        //	raster manner (e.g. row wise) and stored as such in a 1D array. 
        //	In this case the pointer img points to the output array. 
        //
        //	VARIABLES: 
        //	img (output): pointer to image array, an array of signed shorts
        //	rowstart: first row to acquire
        //	numrows: number of rows to acquire
        //	rowskip: skipping between rows (useful if binning is used)
        //	colstart: first column to acquire
        //	numcols: number of columns to acquire
        //	colskip: skipping between columns
        //	ADCType: which ADC to use, defined ADC_TYPES
        //	ANALOG (0,1,2,3): which analog input to use
        //	
        //	EXAMPLES:
        //	getImage(img,16,8,1,24,8,1,SMH1_ADCTYPE_ONBOARD,0): 
        //	Grab an 8x8 window of pixels at raw resolution starting at row 
        //	16, column 24, from chip using onboard ADC at input 0
        //	getImage(img,0,14,8,0,14,8,SMH1_ADCTYPE_MCP3201,2): 
        //	Grab entire Stonyman chip when using
        //	8x8 binning. Grab from input 2.
        /*********************************************************************/
        void getImageColSumAnalog(short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input);

        void getImageColSumDigital(short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input);

        /*********************************************************************/
        //	findMax
        //	Searches over a block section of a Stonyman or Hawksbill chip
        //	to find the brightest pixel. This function is intended to be used 
        //	for things like finding the location of a pinhole in response to 
        //	a bright light.
        //
        //	VARIABLES: 
        //	rowstart: first row to search
        //	numrows: number of rows to search
        //	rowskip: skipping between rows (useful if binning is used)
        //	colstart: first column to search
        //	numcols: number of columns to search
        //	colskip: skipping between columns
        //	ADCType: which ADC to use, defined ADC_TYPES
        //	ANALOG (0,1,2,3): which analog input to use
        //	rowwinner: (output) pointer to variable to write row of brightest 
        //	pixel
        //	colwinner: (output) pointer to variable to write column of 
        //	brightest pixel
        //
        //	EXAMPLE:
        //	FindMaxSlow(8,104,1,8,104,1,SMH1_ADCTYPE_ONBOARD,0,&rowwinner,
        //	&colwinner): 
        //	Search rows 8...104 and columns 8...104 for brightest pixel, with 
        //	onboard ADC, chip 0
        /*********************************************************************/
        void findMaxAnalog(
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input,
                uint8_t *max_row, 
                uint8_t *max_col);

        void findMaxDigital(
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input,
                uint8_t *max_row, 
                uint8_t *max_col);

        /*********************************************************************/
        //	chipToMatlab
        //	This function dumps the entire contents of a Stonyman or 
        //	Hawksbill chip to the Serial monitor in a form that may be copied 
        //	into Matlab. The image is written be stored in matrix Img. 
        //
        //	VARIABLES: 
        //	ADCType: which ADC to use, defined ADC_TYPES
        //	ANALOG (0,1,2,3): Selects one analog input
        /*********************************************************************/
        void chipToMatlabAnalog(uint8_t input);
        void chipToMatlabDigital(uint8_t input);

        /*********************************************************************/
        //	sectionToMatlab
        //	This function dumps a box section of a Stonyman or Hawksbill 
        //	to the Serial monitor in a form that may be copied into Matlab. 
        //	The image is written to be stored in matrix Img. 
        //
        //	VARIABLES: 
        //	rowstart: first row to acquire
        //	numrows: number of rows to acquire
        //	rowskip: skipping between rows (useful if binning is used)
        //	colstart: first column to acquire
        //	numcols: number of columns to acquire
        //	colskip: skipping between columns
        //	ADCType: which ADC to use, defined ADC_TYPES
        //	ANALOG (0,1,2,3): which analog input to use
        //
        //	EXAMPLES:
        //	sectionToMatlab(16,8,1,24,8,1,SMH1_ADCTYPE_ONBOARD,0): 
        //	Grab an 8x8 window of pixels at raw resolution starting at row 
        //	16, column 24, from onboard ADC at chip 0
        //	sectionToMatlab(0,14,8,0,14,8,SMH1_ADCTYPE_ONBOARD,2): 
        //	Grab entire Stonyman chip when using 8x8 binning. Grab from input 
        //	2.
        /*********************************************************************/
        void sectionToMatlabAnalog(
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input);   

        void sectionToMatlabDigital(
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input);   
    private:

        //indicates whether amplifier is in use	
        bool use_amp;

        // input pins
        uint8_t _resp;
        uint8_t _incp;
        uint8_t _resv;
        uint8_t _incv;
        uint8_t _inphi;

        void get_image(
                short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input,
                bool use_digital);

        void get_image_row_sum(
                short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input, 
                bool use_digital);

        void get_image_col_sum(
                short *img, 
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input, 
                bool use_digital);

        void find_max(
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input,
                uint8_t *max_row, 
                uint8_t *max_col,
                bool use_digital);

        void chip_to_matlab(uint8_t input, bool use_digital);

        void section_to_matlab(
                uint8_t rowstart, 
                uint8_t numrows, 
                uint8_t rowskip, 
                uint8_t colstart, 
                uint8_t numcols, 
                uint8_t colskip, 
                uint8_t input, 
                bool use_digital);

        static void init_pin(uint8_t pin);

        /*********************************************************************/
        // Chip Register and Value Manipulation

        // Sets pointer on chip
        void set_pointer(uint8_t ptr);

        // Sets value of current register
        void set_value(short val);

        //	Sets the pointer system register to the desired value.  Value is
        //	not reset so the current value must be taken into account.
        void inc_value(short val);

        //	Operates the amplifier.  Sets inphi pin high, delays to allow
        //	value time to settle, and then brings it low.
        void pulse_inphi(uint8_t delay); 

        //	Resets the value of all registers to zero
        void clear_values(void);

        //	Sets the pointer to a register and sets the value of that        
        //	register
        void set_pointer_value(uint8_t ptr,short val);
};

#endif
