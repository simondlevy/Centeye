#include "CenteyeStonyman.h"
#include <SPI.h>	//SPI required for external ADC

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

void ArduEyeSMH::setValue(short val) 
{
    // clear pointer
    pulse(_resv);

    // increment pointer
    for (short i=0; i!=val; ++i) 
        pulse(_incv);
}

void ArduEyeSMH::incValue(short val) 
{
    for (short i=0; i<val; ++i) //increment pointer
        pulse(_incv);
}

void ArduEyeSMH::pulseInphi(char delay) 
{
    (void)delay;
    pulse(_inphi);
}

void ArduEyeSMH::setPointerValue(char ptr,short val)
{
    setPointer(ptr);	//set pointer to register
    setValue(val);	//set value of that register
}

void ArduEyeSMH::clearValues(void)
{
    for (char i=0; i!=8; ++i)
        setPointerValue(i,0);	//set each register to zero
}

void  ArduEyeSMH::setVREF(short vref)
{
    setPointerValue(SMH_SYS_VREF,vref);
}

void  ArduEyeSMH::setNBIAS(short nbias)
{
    setPointerValue(SMH_SYS_NBIAS,nbias);
}

void  ArduEyeSMH::setAOBIAS(short aobias)
{
    setPointerValue(SMH_SYS_AOBIAS,aobias);
}

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

void ArduEyeSMH::setBiases(short vref,short nbias,short aobias)
{
    setPointerValue(SMH_SYS_NBIAS,nbias);
    setPointerValue(SMH_SYS_AOBIAS,aobias);
    setPointerValue(SMH_SYS_VREF,vref);
}

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

void ArduEyeSMH::applyMask(short *img, short size, uint8_t *mask, short mask_base)
{
    // Subtract calibration mask
    for (int i=0; i<size;++i) 
    {
        img[i] -= mask_base+mask[i];  //subtract FPN mask
        img[i]=-img[i];          //negate image so it displays properly
    }
}

void ArduEyeSMH::getImageAnalog(short *img, 
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input) 
{
    get_image(img, rowstart, numrows, rowskip, colstart, numcols, colskip, input, false);
}

void ArduEyeSMH::getImageDigital(short *img, 
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input) 
{
    get_image(img, rowstart, numrows, rowskip, colstart, numcols, colskip, input, true);
}

void ArduEyeSMH::get_image(
        short *img, 
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input, 
        bool    use_digital) 
{
    // XXX need to support digital (SPI) as well    
    (void)use_digital;

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

            val = analogRead(input); // acquire pixel XXX need to support digital (SPI) as well

            *pimg = val; // store pixel
            pimg++; // advance pointer
            incValue(colskip); // go to next column
        }
        setPointer(SMH_SYS_ROWSEL);
        incValue(rowskip); // go to next row
    }
}

void ArduEyeSMH::getImageRowSumAnalog(
        short *img, 
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input)
{
    get_image_row_sum(img, rowstart, numrows, rowskip, colstart, numcols, colskip, input, false);
}

void ArduEyeSMH::getImageRowSumDigital(
        short *img, 
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input)
{
    get_image_row_sum(img, rowstart, numrows, rowskip, colstart, numcols, colskip, input, true);
}

void ArduEyeSMH::get_image_row_sum(
        short *img, 
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input,
        bool use_digital) 
{
    (void)use_digital;

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

            val = analogRead(input); // acquire pixel

            total+=val;	//sum values along row
            incValue(colskip); // go to next column
        }

        *pimg = total>>4; // store pixel divide to avoid overflow
        pimg++; // advance pointer

        setPointer(SMH_SYS_ROWSEL);
        incValue(rowskip); // go to next row
    }
}

void ArduEyeSMH::getImageColSumAnalog(
        short *img, 
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input) 
{
    get_image_col_sum(img, rowstart, numrows, rowskip, colstart, numcols, colskip, input, false);
}

void ArduEyeSMH::getImageColSumDigital(
        short *img, 
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input) 
{
    get_image_col_sum(img, rowstart, numrows, rowskip, colstart, numcols, colskip, input, true);
}
    
void ArduEyeSMH::get_image_col_sum(
        short *img, 
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input,
        bool use_digital) 
{
    (void)use_digital;

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

            val = analogRead(input); // acquire pixel

            total+=val;	//sum value along column
            incValue(rowskip); // go to next row
        }

        *pimg = total>>4; // store pixel
        pimg++; // advance pointer

        setPointer(SMH_SYS_COLSEL);
        incValue(colskip); // go to next col
    }
}


void ArduEyeSMH::findMaxAnalog(
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols,
        uint8_t colskip, 
        uint8_t input,
        uint8_t *max_row, 
        uint8_t *max_col)
{
    find_max(rowstart, numrows, rowskip, colstart, numcols, colskip, input, max_row, max_col, false);
}

void ArduEyeSMH::findMaxDigital(
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols,
        uint8_t colskip, 
        uint8_t input,
        uint8_t *max_row, 
        uint8_t *max_col)
{
    find_max(rowstart, numrows, rowskip, colstart, numcols, colskip, input, max_row, max_col, true);
}

void ArduEyeSMH::find_max(
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols,
        uint8_t colskip, 
        uint8_t input,
        uint8_t *max_row, 
        uint8_t *max_col,
        bool use_digital)
{
    (void)use_digital;

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

            val = analogRead(input); // acquire pixel

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

    *max_row = bestrow;
    *max_col = bestcol;
}

void ArduEyeSMH::chipToMatlabAnalog(uint8_t input) 
{
    chip_to_matlab(input, false); 
}

void ArduEyeSMH::chipToMatlabDigital(uint8_t input) 
{
    chip_to_matlab(input, true); 
}

void ArduEyeSMH::chip_to_matlab(uint8_t input, bool use_digital) 
{
    (void)use_digital;

    uint8_t row,col;
    unsigned short val;

    Serial.println("Img = [");
    setPointerValue(SMH_SYS_ROWSEL,0); // set row = 0
    for (row=0; row<112; ++row) {
        setPointerValue(SMH_SYS_COLSEL,0); // set column = 0
        for (col=0; col<112; ++col) {
            // settling delay
            delayMicroseconds(1);
            // pulse amplifier if needed
            if (useAmp) 
                pulseInphi(2);

            // get data value
            delayMicroseconds(1);

            val = analogRead(input); // acquire pixel

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
}

void ArduEyeSMH::sectionToMatlabAnalog(
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input) 
{
    section_to_matlab(rowstart, numrows, rowskip, colstart, numcols, colskip, input, false);
}

void ArduEyeSMH::sectionToMatlabDigital(
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input) 
{
    section_to_matlab(rowstart, numrows, rowskip, colstart, numcols, colskip, input, true);
}

void ArduEyeSMH::section_to_matlab(
        uint8_t rowstart, 
        uint8_t numrows, 
        uint8_t rowskip, 
        uint8_t colstart, 
        uint8_t numcols, 
        uint8_t colskip, 
        uint8_t input,
        bool use_digital) 
{
    (void)use_digital;

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

            val = analogRead(input); // acquire pixel

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
