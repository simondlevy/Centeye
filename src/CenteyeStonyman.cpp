#include "CenteyeStonyman.h"
#include <SPI.h>	//SPI required for external ADC

/*********************************************************************/
//	begin
//	Initializes the vision chips for normal operation.  Sets vision
//	chip pins to low outputs, clears chip registers, sets biases and
//	config register.  If no parameters are passed in, default values
//	are used.
/*********************************************************************/

ArduEyeSMH::ArduEyeSMH(uint8_t resp, uint8_t incp, uint8_t resv, uint8_t incv, uint8_t inphi)
{
    _resp = resp;
    _incp = incp;
    _resv = resv;
    _incv = incv;
    _inphi = inphi;
}

void ArduEyeSMH::init_pin(uint8_t pin)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void ArduEyeSMH::begin(short vref,short nbias,short aobias, char selamp)
{
    //set all digital pins to output
    init_pin(_resp);
    init_pin(_incp);
    init_pin(_resv);
    init_pin(_incv);

    //clear all chip register values
    clearValues();

    //set up biases
    setBiases(vref,nbias,aobias);

    //turn chip on with config value
    setPointerValue(SMH_SYS_CONFIG,16);

    //if amp is used, set useAmp variable
    if(selamp==1)
        useAmp=1;
    else
        useAmp=0;

}

/*********************************************************************/
//	setPointer
//	Sets the pointer system register to the desired value
/*********************************************************************/

static void pulse(int pin)
{
    digitalWrite(pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(pin, LOW);
}

void ArduEyeSMH::setPointer(char ptr)
{
    // clear pointer
    pulse(_resp);

    // increment pointer to desired value
    for (short i=0; i!=ptr; ++i) 
        pulse(_incp);
}

/*********************************************************************/
//	setValue
//	Sets the value of the current register
/*********************************************************************/

void ArduEyeSMH::setValue(short val) 
{
    // clear pointer
    pulse(_resv);

    // increment pointer
    for (short i=0; i!=val; ++i) 
        pulse(_incv);
}

/*********************************************************************/
//	incValue
//	Sets the pointer system register to the desired value.  Value is
//	not reset so the current value must be taken into account
/*********************************************************************/

void ArduEyeSMH::incValue(short val) 
{
    for (short i=0; i<val; ++i) //increment pointer
        pulse(_incv);
}

/*********************************************************************/
//	pulseInphi
//	Operates the amplifier.  Sets inphi pin high, delays to allow
//	value time to settle, and then brings it low.
/*********************************************************************/

void ArduEyeSMH::pulseInphi(char delay) 
{
    (void)delay;
    pulse(_inphi);
}

/*********************************************************************/
//	setPointerValue
//	Sets the pointer to a register and sets the value of that        
//	register
/*********************************************************************/

void ArduEyeSMH::setPointerValue(char ptr,short val)
{
    setPointer(ptr);	//set pointer to register
    setValue(val);	//set value of that register
}

/*********************************************************************/
//	clearValues
//	Resets the value of all registers to zero
/*********************************************************************/

void ArduEyeSMH::clearValues(void)
{
    for (char i=0; i!=8; ++i)
        setPointerValue(i,0);	//set each register to zero
}

/*********************************************************************/
//	setVREF
//	Sets the VREF register value (0-63)
/*********************************************************************/

void  ArduEyeSMH::setVREF(short vref)
{
    setPointerValue(SMH_SYS_VREF,vref);
}

/*********************************************************************/
//	setNBIAS
//	Sets the NBIAS register value (0-63)
/*********************************************************************/

void  ArduEyeSMH::setNBIAS(short nbias)
{
    setPointerValue(SMH_SYS_NBIAS,nbias);
}

/*********************************************************************/
//	setAOBIAS
//	Sets the AOBIAS register value (0-63)
/*********************************************************************/

void  ArduEyeSMH::setAOBIAS(short aobias)
{
    setPointerValue(SMH_SYS_AOBIAS,aobias);
}

/*********************************************************************/
//	setBiasesVdd
//	Sets biases based on chip voltage
/*********************************************************************/

void ArduEyeSMH::setBiasesVdd(char vddType)
{

    // determine biases. Only one option for now.
    switch (vddType) 
    {
        case SMH1_VDD_5V0:	//chip receives 5V
        default:
            setPointerValue(SMH_SYS_NBIAS,SMH_NBIAS_5V0);
            setPointerValue(SMH_SYS_AOBIAS,SMH_AOBIAS_5V0);
            setPointerValue(SMH_SYS_VREF,SMH_VREF_5V0);
            break;
    }
}

/*********************************************************************/
//	setBiases
//	Sets all three biases
/*********************************************************************/

void ArduEyeSMH::setBiases(short vref,short nbias,short aobias)
{
    setPointerValue(SMH_SYS_NBIAS,nbias);
    setPointerValue(SMH_SYS_AOBIAS,aobias);
    setPointerValue(SMH_SYS_VREF,vref);
}

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

void ArduEyeSMH::setConfig(char gain, char selamp, char cvdda) 
{
    short config=gain+(selamp*8)+(cvdda*16);	//form register value

    if(selamp==1)	//if selamp is 1, set useAmp variable to 1
        useAmp=1;
    else 
        useAmp=0;

    // Note that config will have the following form binary form:
    // 000csggg where c=cvdda, s=selamp, ggg=gain (3 bits)
    // Note that there is no overflow detection in the input values.
    setPointerValue(SMH_SYS_CONFIG,config);
}

/*********************************************************************/
//	setAmpGain
//	A friendlier version of setConfig.  If amplifier gain is set to 
//	zero, amplifier is bypassed.  Otherwise the appropriate amplifier
//	gain (range 1-7) is set.
/*********************************************************************/

void ArduEyeSMH::setAmpGain(char gain)
{
    short config;

    if((gain>0)&&(gain<8))	//if gain is a proper value, connect amp
    {
        config=gain+8+16;	//gain+(selamp=1)+(cvdda=1)
        useAmp=1;	//using amplifier
    }
    else	//if gain is zero or outside range, bypass amp
    {
        config=16;	//(cvdda=1)
        useAmp=0;	//bypassing amplifier
    }

    setPointerValue(SMH_SYS_CONFIG,config);	//set config register
}

/*********************************************************************/
//	setAnalogInput
//	Sets the analog pin for one vision chip to be an input.
//	This is for the Arduino onboard ADC, not an external ADC
/*********************************************************************/

void ArduEyeSMH::setAnalogInput(char analogInput)
{
    (void)analogInput;
}

/*********************************************************************/
//	setADCInput
//	Sets the analog pin to be a digital output and select a chip
//	to connect to the external ADC.  The state can be used to
//	deselect a particular chip as well.
/*********************************************************************/

void ArduEyeSMH::setADCInput(char ADCInput,char state)
{
    (void)ADCInput;
    (void)state;

}

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

void ArduEyeSMH::setBinning(short hbin,short vbin)
{
    short hsw,vsw;

    switch (hbin) //horizontal binning
    {
        case 2:		//downsample by 2
            hsw = 0xAA;
            break;
        case 4:		//downsample by 4
            hsw = 0xEE;
            break;
        case 8:		//downsample by 8
            hsw = 0xFE;
            break;
        default:	//no binning
            hsw = 0x00;
    }

    switch (vbin) 	//vertical binning
    {
        case 2:		//downsample by 2
            vsw = 0xAA;
            break;
        case 4:		//downsample by 4
            vsw = 0xEE;
            break;
        case 8:		//downsample by 8
            vsw = 0xFE;
            break;
        default:	//no binning
            vsw = 0x00;
    }

    //set switching registers
    setPointerValue(SMH_SYS_HSW,hsw);
    setPointerValue(SMH_SYS_VSW,vsw);
}

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

void ArduEyeSMH::calcMask(short *img, short size, uint8_t *mask,short *mask_base)
{
    *mask_base = 10000; // e.g. "high"

    for (int i=0; i<size; ++i)
        if (img[i]<(*mask_base))	//find the min value for mask_base
            *mask_base = img[i];

    // generate calibration mask
    for (int i=0; i<size; ++i)
        mask[i] = img[i] - *mask_base;	//subtract min value for mask
}

/*********************************************************************/
//	applyMask
//	given the "mask" and "mask_base" variables calculated in        
//	calcMask, and a current image, this function will subtract the
//	mask to provide a calibrated image.
/*********************************************************************/

void ArduEyeSMH::applyMask(short *img, short size, uint8_t *mask, short mask_base)
{
    // Subtract calibration mask
    for (int i=0; i<size;++i) 
    {
        img[i] -= mask_base+mask[i];  //subtract FPN mask
        img[i]=-img[i];          //negate image so it displays properly
    }
}

/*********************************************************************/
//	getImage
//	This function acquires a box section of a Stonyman or Hawksbill 
//	and saves to image array img.  Note that images are read out in 
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

void ArduEyeSMH::getImage(short *img, 
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        char ADCType,char ANALOG) 
{
    (void)ADCType;

    short *pimg = img; // pointer to output image array
    short val;
    uint8_t row,col;

    // Go to first row
    setPointerValue(SMH_SYS_ROWSEL,rowstart);

    // Loop through all rows
    for (row=0; row<numrows; ++row) {

        // Go to first column
        setPointerValue(SMH_SYS_COLSEL,colstart);

        // Loop through all columns
        for (col=0; col<numcols; ++col) {

            // settling delay
            delayMicroseconds(1);

            // pulse amplifier if needed
            if (useAmp) 
                pulseInphi(2);

            // get data value
            delayMicroseconds(1);

            val = analogRead(ANALOG); // acquire pixel

            *pimg = val; // store pixel
            pimg++; // advance pointer
            incValue(colskip); // go to next column
        }
        setPointer(SMH_SYS_ROWSEL);
        incValue(rowskip); // go to next row
    }
}

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

void ArduEyeSMH::getImageRowSum(short *img, uint8_t rowstart, uint8_t numrows, uint8_t rowskip, uint8_t colstart, uint8_t numcols, uint8_t colskip, char ADCType,char ANALOG) 
{
    short *pimg = img; // pointer to output image array
    short val,total=0;
    uint8_t row,col;

    // Go to first row
    setPointerValue(SMH_SYS_ROWSEL,rowstart);

    // Loop through all rows
    for (row=0; row<numrows; ++row) {

        // Go to first column
        setPointerValue(SMH_SYS_COLSEL,colstart);

        total=0;

        // Loop through all columns
        for (col=0; col<numcols; ++col) {

            // settling delay
            delayMicroseconds(1);

            // pulse amplifier if needed
            if (useAmp) 
                pulseInphi(2);

            // get data value
            delayMicroseconds(1);

            val = analogRead(ANALOG); // acquire pixel

            total+=val;	//sum values along row
            incValue(colskip); // go to next column
        }

        *pimg = total>>4; // store pixel divide to avoid overflow
        pimg++; // advance pointer

        setPointer(SMH_SYS_ROWSEL);
        incValue(rowskip); // go to next row
    }

    if((ADCType!=SMH1_ADCTYPE_ONBOARD)&&(ADCType!=SMH1_ADCTYPE_MCP3201_2))
        setADCInput(ANALOG,0); // disable chip

}

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

void ArduEyeSMH::getImageColSum(short *img, uint8_t rowstart, uint8_t numrows, uint8_t rowskip, uint8_t colstart, uint8_t numcols, uint8_t colskip, char ADCType,char ANALOG) 
{
    short *pimg = img; // pointer to output image array
    short val,total=0;
    uint8_t row,col;

    // Go to first col
    setPointerValue(SMH_SYS_COLSEL,colstart);

    // Loop through all cols
    for (col=0; col<numcols; ++col) {

        // Go to first row
        setPointerValue(SMH_SYS_ROWSEL,rowstart);

        total=0;

        // Loop through all rows
        for (row=0; row<numrows; ++row) {

            // settling delay
            delayMicroseconds(1);

            // pulse amplifier if needed
            if (useAmp) 
                pulseInphi(2);

            // get data value
            delayMicroseconds(1);

            val = analogRead(ANALOG); // acquire pixel

            total+=val;	//sum value along column
            incValue(rowskip); // go to next row
        }

        *pimg = total>>4; // store pixel
        pimg++; // advance pointer

        setPointer(SMH_SYS_COLSEL);
        incValue(colskip); // go to next col
    }

    if((ADCType!=SMH1_ADCTYPE_ONBOARD)&&(ADCType!=SMH1_ADCTYPE_MCP3201_2))
        setADCInput(ANALOG,0); // disable chip

}


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

void ArduEyeSMH::findMax(
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols,
        uint8_t colskip, 
        char ADCType,
        char ANALOG,
        unsigned 
        char *max_row, 
        uint8_t *max_col)
{
    unsigned short maxval=5000,minval=0,val;
    uint8_t row,col,bestrow=0,bestcol=0;

    // Go to first row
    setPointerValue(SMH_SYS_ROWSEL,rowstart);

    // Loop through all rows
    for (row=0; row<numrows; ++row) {

        // Go to first column
        setPointerValue(SMH_SYS_COLSEL,colstart);

        // Loop through all columns
        for (col=0; col<numcols; ++col) {

            // settling delay
            delayMicroseconds(1);

            // pulse amplifier if needed
            if (useAmp) 
                pulseInphi(2);

            // get data value
            delayMicroseconds(1);

            val = analogRead(ANALOG); // acquire pixel

            if(useAmp)	//amplifier is inverted
            {
                if (val>minval) 	//find max val (bright)
                {
                    bestrow=row;
                    bestcol=col;
                    minval=val;
                }
            }
            else		//unamplified
            {
                if (val<maxval) 	//find min val (bright)
                {
                    bestrow=row;
                    bestcol=col;
                    maxval=val;
                }
            }

            incValue(colskip); // go to next column
        }
        setPointer(SMH_SYS_ROWSEL);
        incValue(rowskip); // go to next row
    }

    if(ADCType!=SMH1_ADCTYPE_ONBOARD)
        setADCInput(ANALOG,0); // disable chip

    // Optionally we can comment out these next three items
    //Serial.print("bestrow = "); Serial.println((short)bestrow);
    //Serial.print("bestcol = "); Serial.println((short)bestcol);
    //Serial.print("maxval = "); Serial.println((short)maxval);

    *max_row = bestrow;
    *max_col = bestcol;
}

/*********************************************************************/
//	chipToMatlab
//	This function dumps the entire contents of a Stonyman or 
//	Hawksbill chip to the Serial monitor in a form that may be copied 
//	into Matlab. The image is written be stored in matrix Img. 
//
//	VARIABLES: 
//	whichchip(0 or 1): 0 for Stonyman, 1 for Hawksbill
//	ADCType: which ADC to use, defined ADC_TYPES
//	ANALOG (0,1,2,3): Selects one analog input
/*********************************************************************/

void ArduEyeSMH::chipToMatlab(char whichchip,char ADCType, char ANALOG) 
{
    uint8_t row,col,rows,cols;
    unsigned short val;

    if (whichchip==1) {
        rows=cols=136;	//hawksbill
    }	else {
        rows=cols=112;	//stonyman
    }	

    Serial.println("Img = [");
    setPointerValue(SMH_SYS_ROWSEL,0); // set row = 0
    for (row=0; row<rows; ++row) {
        setPointerValue(SMH_SYS_COLSEL,0); // set column = 0
        for (col=0; col<cols; ++col) {
            // settling delay
            delayMicroseconds(1);
            // pulse amplifier if needed
            if (useAmp) 
                pulseInphi(2);

            // get data value
            delayMicroseconds(1);

            val = analogRead(ANALOG); // acquire pixel

            // increment column
            incValue(1);
            Serial.print(val);
            Serial.print(" ");
        }
        setPointer(SMH_SYS_ROWSEL); // point to row
        incValue(1); // increment row
        Serial.println(" ");
    }
    Serial.println("];");
    if(ADCType!=SMH1_ADCTYPE_ONBOARD)
        setADCInput(ANALOG,0); // disable chip

}

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

void ArduEyeSMH::sectionToMatlab(
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        char ADCType, 
        uint8_t ANALOG) 
{
    // XXX should support SPI 
    (void)ADCType;

    short val;
    uint8_t row,col;

    Serial.println("Img = [");
    setPointerValue(SMH_SYS_ROWSEL,rowstart);

    for (row=0; row<numrows; row++) {

        setPointerValue(SMH_SYS_COLSEL,colstart);

        for (col=0; col<numcols; col++) {
            // settling delay
            delayMicroseconds(1);

            // pulse amplifier if needed
            if (useAmp) 
                pulseInphi(2);

            delayMicroseconds(1);

            val = analogRead(ANALOG); // acquire pixel

            incValue(colskip);
            Serial.print(val);
            Serial.print(" ");
        }
        setPointer(SMH_SYS_ROWSEL);
        incValue(rowskip); // go to next row
        Serial.println(" ");
    }
    Serial.println("];");

}
