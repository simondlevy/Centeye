/*
   ImageUtils.cpp Basic functions to handle images

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

   Generally images are stored as either signed shorts (16 bits), signed chars (8 bits), or unsigned chars (8 bits). 
   Signed numbers are generally preferred to allow for negative values due to frame differences. 8 bits is adequate
   for some applications in particular when bit depth is not needed and/or speed and memory is a consideration. 

   Although images are generally accepted as two dimensional arrays, for this library they are stored in a
   one dimensional array row-wise. So to store a 4x6 image, we would first declare a variable:

   short A[24]; // 24 = 4 rows * 6 columns

   and then store the first row in the first six elements of A, the second row in the second six elements of A,
   and so on. So A[0] = row 0 column 0, A[1] = row 0 column 1, A[5] = row 0 column 5, A[6] = row 1 column 0,
   A[11] = row 1 column 5, etc. This row-wise format speeds up acquisition of pixels, in particular if pointer
   are used for different arrays, or even for different regions within the same region. Note that it is up to 
   the programmer to ensure that arrays are properly allocated and that indices do not go out of bounds.

   The image functions are then written so that you will need to provide either the number of rows and columns,
   or just the total number of pixels (number of rows * number of columns). This is because some functions
   (like direct frame adding or finding the maximum or minimum values in an image) don't require knowledge
   of the row or column of each pixel, whereas other functions (like printing images or more advanced
   operations) do require the dimensions of the image.

   Hexagonal images are a special case and will be discussed here soon. (More text to be added soon...)

 */

// Support both Arduino and standard C++
#if defined(__arm__) || defined(__avr__)
#include <Arduino.h>
#define PRINTSHORT(n) Serial.print(n)
#define PRINTCHAR(c)  Serial.print(c)
#define PRINTSTR(s)   Serial.print(s) 
#define RANDOM(m)     random(m)
#else
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#define PRINTSHORT(n) printf("%d ", n)
#define PRINTCHAR(c)  printf("%c ", c)
#define PRINTSTR(s)   printf("%s ", s)
#define RANDOM(m)     (random() % m)
#endif

#include "ImageUtils.h"

//========================================================================
// IMAGE BASICS AND MANIPULATION
//========================================================================


// These two variables define an array of characters used for ASCII
// dumps of images to the Arduino serial monitor
char ASCII_DISP_CHARS[16] = "#@$%&x*=o+-~,. ";
char NUM_ASCII_DISP_CHARS = 15;


/*------------------------------------------------------------------------
  ImgShortCopy -- Copies image A to image B.
VARIABLES:
A: copy from image
B: copy to image
numpix: number of pixels e.g. rows * columns
STATUS: UNTESTED
 */
void ImgShortCopy(short *A, short *B, unsigned short numpix) {
    unsigned short pix;
    short *pa,*pb;
    pa=A; pb=B;
    for (pix=0; pix<numpix; ++pix) {
        *pb = *pa;
        pa++; pb++;
    }
}

/*------------------------------------------------------------------------
  ImgShortCopy -- Copies image A to image B. OVERLOADED for CHAR
VARIABLES:
A: copy from image
B: copy to image
numpix: number of pixels e.g. rows * columns
STATUS: UNTESTED
 */
void ImgShortCopy(char *A, char *B, unsigned short numpix) {
    unsigned short pix;
    char *pa,*pb;
    pa=A; pb=B;
    for (pix=0; pix<numpix; ++pix) {
        *pb = *pa;
        pa++; pb++;
    }
}



//========================================================================
// IMAGE DISPLAY AND DUMPING (FOR ARDUINO SERIAL MONITOR)
//========================================================================

/*------------------------------------------------------------------------
  ImgShortDumpAsciiSerial -- Dumps image to the screen in a crude
  ASCII format, with darker characters corresponding to brighter images
VARIABLES:
img: input image
numrows,numcolumns: number of rows and columns in image
mini: predefined minimum- pixel values less than this are considered
dark. Enter 0 to force function to use minimum value.
maxi: predefined maximum- pixel values greater than this are 
considered white. Enter 0 to force the function to use the maximum value
STATUS: UNTESTED
 */
void ImgShortDumpAsciiSerial(short *img, short numrows, short numcolumns, short mini, short maxi) {
    short i,m,n,*pix,delta;

    // if mini==0 then we compute minimum
    if (mini==0) {
        mini = *img;
        for (i=0,pix=img; i<numrows*numcolumns; ++i,++pix) {
            if (*pix<mini)
                mini=*pix;
        }
    }
    // if maxi==0 then we compute maximum
    if (maxi==0) {
        maxi = *img;
        for (i=0,pix=img; i<numrows*numcolumns; ++i,++pix) {
            if (*pix>maxi)
                maxi=*pix;
        }
    }
    // Compute delta value for display
    delta = maxi-mini;
    delta = delta / NUM_ASCII_DISP_CHARS;
    if (delta<1)
        delta=1;

    // Loop through and dump image
    pix=img;
    for (m=0; m<numrows; ++m) {
        for (n=0; n<numcolumns; ++n) {
            // rescale pixel value to 0...9
            i = *pix - mini;
            i = i / delta;
            if (i<0)
                i=0;
            if (i>NUM_ASCII_DISP_CHARS)
                i=NUM_ASCII_DISP_CHARS;
            i = NUM_ASCII_DISP_CHARS+1-i;
            // print
            PRINTCHAR(ASCII_DISP_CHARS[i]);
            // next pixel
            pix++;
        }
        PRINTSTR(" \n");
    }
}

#ifdef __UNUSED
// Below is an older version of this function we are retaining, commented out, in case we need to 
// revive it.
void ImgShortDumpAsciiSerial(short *img, short numrows, short numcolumns, short mini, short maxi) {
    short i,m,n,*pix,delta;

    // if mini==0 then we compute minimum
    if (mini==0) {
        mini = *img;
        for (i=0,pix=img; i<numrows*numcolumns; ++i,++pix) {
            if (*pix<mini)
                mini=*pix;
        }
    }
    // if maxi==0 then we compute maximum
    if (maxi==0) {
        maxi = *img;
        for (i=0,pix=img; i<numrows*numcolumns; ++i,++pix) {
            if (*pix>maxi)
                maxi=*pix;
        }
    }
    // Compute delta value for display
    delta = maxi-mini;
    delta = delta / NUM_ASCII_DISP_CHARS;

    // This portion can be deleted eventually...
    char debugstring[100];
    sprintf(debugstring,"maxi=%d mini=%d delta=%d\n",maxi,mini,delta);
    Serial.print(debugstring);

    // Loop through and dump image
    pix=img;
    for (m=0; m<numrows; ++m) {
        for (n=0; n<numcolumns; ++n) {
            // rescale pixel value to 0...9
            i = *pix - mini;
            i = i / delta;
            if (i<0)
                i=0;
            if (i>9)
                i=9;
            // print
            Serial.print(ASCII_DISP_CHARS[i]);
            // next pixel
            pix++;
        }
        Serial.println(" ");
    }
}
#endif

/*------------------------------------------------------------------------
  ImgShortDumpMatlabSerial -- Dumps image to the screen in a manner
  that may be copied into MATLAB.
VARIABLES:
img: input image
numrows,numcolumns: number of rows and columns in image
STATUS: UNTESTED
 */  
void ImgShortDumpMatlabSerial(short *img, unsigned char numrows, unsigned char numcols) {
    short *pimg = img;
    unsigned char row,col;
    PRINTSTR("Dat = [\n");
    for (row=0; row<numrows; ++row) {
        for (col=0; col<numcols; ++col) {
            PRINTSHORT(*pimg);
            PRINTSTR(" ");
            pimg++;
        }
        PRINTSTR(" \n");
    }
    PRINTSTR("];\n");
}


//========================================================================
// MINIMUM AND MAXIMUM
//========================================================================

/*------------------------------------------------------------------------
  ImgShortFindMinMax -- Finds both the darkest and brightest pixels
  in the image.
VARIABLES:
img: input image
numrows,numcolumns: number of rows and columns in image
mini,maxi: pointers to output variables that will contain the minimum
and maximum pixel values
STATUS: UNTESTED
 */  
void ImgShortFindMinMax(short *img, unsigned char numrows, unsigned char numcols, short *mini, short *maxi) {
    short *pimg=img;
    unsigned char row,col;
    *mini = 0xFFFF; // initialize
    *maxi = 0x0000; // initialize
    for (row=0; row<numrows; ++row)
        for (col=0; col<numcols; ++col) {
            if (*pimg<*mini)
                *mini=*pimg;
            if (*pimg>*maxi)
                *maxi=*pimg;
            pimg++;
        }
}

/*------------------------------------------------------------------------
  ImgShortFindMax -- Finds the minimum or maximum value of an image and
  returns the row and column. Polarity value allows finding of minimum 
  instead of maximum.
VARIABLES:
img: input image
numrows,numcolumns: number of rows and columns in image
polarity: 0=find maximum, 1=find minimum
winrow,wincol: row and column of winning pixel
STATUS: UNTESTED
 */ 
void ImgShortFindMax(short *img, unsigned char numrows, unsigned char numcols, unsigned char polarity, unsigned char *winrow, unsigned char *wincol) {
    short *pimg=img,val,bestval;
    unsigned char row,col;

    bestval=0;

    for (row=0; row<numrows; ++row)
        for (col=0; col<numcols; ++col) {
            val = *pimg;
            if (polarity)
                val = 5000-val;
            if (val>bestval) {
                bestval=val;
                *winrow = row;
                *wincol = col;
            }
            pimg++;
        }
}

/*------------------------------------------------------------------------
  ImgShortMin -- Returns the minimum pixel value in an image.
VARIABLES:
A: input image
numpix: number of pixels e.g. rows * columns
OUTPUT: minimum pixel value
STATUS: UNTESTED
 */
short ImgShortMin(short *A, unsigned short numpix) {
    short *pa = A;
    short minval;
    unsigned short pixnum;

    minval = *A;
    for (pixnum=0; pixnum<numpix; ++pixnum) {
        if (*pa < minval)
            minval = *pa;
        ++pa;
    }
    return minval;
}

/*------------------------------------------------------------------------
  ImgShortMax -- Returns the maximum pixel value in an image.
VARIABLES:
A: input image
numpix: number of pixels e.g. rows * columns
OUTPUT: maximum pixel value
STATUS: UNTESTED
 */
short ImgShortMax(short *A, unsigned short numpix) {
    short *pa = A;
    short maxval;
    unsigned short pixnum;

    maxval = *A;
    for (pixnum=0; pixnum<numpix; ++pixnum) {
        if (*pa > maxval)
            maxval = *pa;
        ++pa;
    }
    return maxval;
}

//========================================================================
// IMAGE ARITHMETIC
//========================================================================

/*------------------------------------------------------------------------
  ImgShortDiff -- Computes pixel-wise difference between two images e.g.
  computes a simple frame difference. Input and output images are all short.
VARIABLES:
A,B: input image
D: output image containing A-B
numpix: number of pixels e.g. rows * columns
STATUS: UNTESTED
 */
void ImgShortDiff(short *A, short *B, short *D, unsigned short numpix) {
    short *pa,*pb,*pd;
    pa=A; pb=B; pd=D;
    unsigned short pixnum;
    for (pixnum=0; pixnum<numpix; pixnum++) {
        *pd = *pa - *pb;
        pa++;
        pb++;
        pd++;
    }
}

/*------------------------------------------------------------------------
  ImgShortHPF -- Implements a time-domain high pass filter. Image I is
  the input image, image L is a time-domain low passed version of I, and
  image H is a high passed version. WORK ON THIS FUNCTION.
VARIABLES:
I: input image
L: low-passed image, shifted left by four bits.
H: high-passed image, unshifted
numpix: number of pixels e.g. rows * columns
shiftalpha: shifting amount used to implement time constant
STATUS: UNTESTED
 */
void ImgShortHPF(short *I, short *L, short *H, short numpix, char shiftalpha) {
    short pix,*pi,*pl,*ph;
    pi=I; pl=L; ph=H;
    for (pix=0; pix<numpix; ++pix) {
        // update lowpass
        short indiff = (*pi << 4) - *pl ;
        indiff = indiff >> shiftalpha;
        *pl += indiff;
        // compute highpass
        *ph = *pi - (*pl >> 4);
        // increment pointers
        pi++; pl++; ph++;
    }
}

//========================================================================
// IMAGE FIXED PATTERN NOISE FUNCTIONS
//========================================================================

/*------------------------------------------------------------------------
  ImgShortAddCharFPN -- Adds an FPN to an image. Allows the FPN to be
  multiplied. Basically this function implements A = A + F * mult
  where F is an image of unsigned chars.
VARIABLES:
A: image of shorts
F: fixed pattern noise image of unsigned chars
numpix: number of pixels e.g. rows * columns
mult: multiplier
STATUS: UNTESTED
 */
void ImgShortAddCharFPN(short *A, unsigned char *F, unsigned short numpix, unsigned char mult) {
    short *pa=A;
    unsigned char *pf=F;
    unsigned short pixnum;

    for (pixnum=0; pixnum<numpix; ++pixnum) {
        *pa += ((short)(*pf))*mult;
        pa++;
        pf++;
    }
}

/*------------------------------------------------------------------------
  ImgCharMakeFPN -- Generates a random fixed pattern noise (FPN) to 
  be used to add still texture to an image. This can be added to high
  passed images so that when there is no motion there is still some 
  texture so that motion sensing algorithms measure zero motion. Yes, this
  is a bit of a kludge... The fixed pattern noise is stored as
  unsigned chars to make room. Parameter "modval" determines the strength
  of the FPN.
VARIABLES:
F: output image
numpix: number of pixels e.g. rows * columns
modval: number of levels in the image, e.g. 2 makes a binary image, 
3 makes an image having values 0,1,2, and so on.
STATUS: UNTESTED
 */
void ImgCharMakeFPN(unsigned char *F, unsigned short numpix, unsigned char modval) {
    unsigned char *pf = F;
    unsigned short pixnum;

    for (pixnum=0; pixnum<numpix; ++pixnum) {
        *pf = (unsigned char)(RANDOM(modval) );
        pf++;
    }
}

//========================================================================
// WINDOWING - EXTRACTING SUBSETS OF IMAGES
//========================================================================

/*------------------------------------------------------------------------
  SubwinShort2D -- Extracts a 2D subwindow from a 2D image of shorts. 
VARIABLES:
I: input image
S: output image
Irows,Icols: number of rows and columns of I
subrow,subcol: row and column of upper left pixel of subwindow
startrow: starting row of window
numrows: number of rows of subwindow
startcol: starting column of window
numcols: number of columns of subwindow
STATUS: UNTESTED
 */
void SubwinShort2D(short *I, short *S, char Irows, char Icols, char startrow, char numrows, char startcol, char numcols) {
    (void)Irows;
    short *pi,*ps;
    char r,c;

    ps = S;
    for (r=0; r<numrows; ++r) {
        pi = I + Icols*(startrow+r) + startcol;
        for (c=0; c<numcols; ++c) {
            *ps = *pi;
            ps++;
            pi++;
        }
    }
}

/*------------------------------------------------------------------------
  SubwinShort2Dto1D -- Extracts a subwindow from a 2D image of shorts
  and then sums rows or columns to form a 1D image. This is a simple way
  to generate 1D images from a 2D image, using a mathematical equivalent
  of on-chip binning. Note that a SUM is used for the resulting pixels,
  not an AVERAGE.
VARIABLES:
I: input image
S: output image
Irows,Icols: number of rows and columns of I
subrow,subcol: row and column of upper left pixel of subwindow
Snumpix: number of pixels in resulting 1D image
Spixlength: length of virtual "rectangles" used to implement super pixels
orientation: 1 means rectangles are horizontal, e.g. window is averaged
row-wise, 2 means rectangles ore vertical.
STATUS: UNTESTED
 */
void SubwinShort2Dto1D(short *I, short *S, char Irows, char Icols, char subrow, char subcol, char Snumpix, char Spixlength, char orientation) {
    (void)Irows;

    // first clear image S
    for (uint8_t r=0; r<Snumpix; ++r) {
        S[r]=0;
    }

    if (orientation==1) { // horizontal pixels
        for (uint8_t r=0; r<Snumpix; ++r) {
            short * pi = I + Icols*(subrow+r) + subcol;
            for (uint8_t c=0; c<Spixlength; ++c) {
                S[r] += *pi;
                pi++;
            }
        }
    } else if (orientation==2) { // vertical pixels
        for (uint8_t r=0; r<Spixlength; ++r) {
            short * pi = I + Icols*(subrow+r) + subcol;
            for (uint8_t c=0; c<Snumpix; ++c) {
                S[c] += *pi;
                pi++;
            }
        }
    }
}

