/*
===============================================================================
ImagesUtils.h Basic functions to handle images

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
/* 
Change log:
January 22, 2012: initial release
February 17, 2012: includes Arduino.h or WProgram.h depending on Arduino IDE version
*/

#ifndef Images_v1_h
#define Images_v1_h

void ImgShortCopy(uint16_t *A, uint16_t *B,  uint16_t numpix);
void ImgShortCopy(uint8_t *A, uint8_t *B,  uint16_t numpix);
void ImgShortDumpAsciiSerial(uint16_t *img, uint16_t numrows, uint16_t numcolumns, uint16_t mini, uint16_t maxi);
void ImgShortDumpMatlabSerial(uint16_t *img,  uint8_t numrows,  uint8_t numcols);
void ImgShortFindMinMax(uint16_t *img,  uint8_t numrows,  uint8_t numcols, uint16_t *mini, uint16_t *maxi);
void ImgShortFindMax(uint16_t *img,  uint8_t numrows,  uint8_t numcols,  uint8_t polarity,  uint8_t *winrow,  uint8_t *wincol);
uint16_t ImgShortMin(uint16_t *A,  uint16_t numpix);
uint16_t ImgShortMax(uint16_t *A,  uint16_t numpix);
void ImgShortDiff(uint16_t *A, uint16_t *B, uint16_t *D,  uint16_t numpix);
void ImgShortHPF(uint16_t *I, uint16_t *L, uint16_t *H, uint16_t numpix, uint8_t shiftalpha);
void ImgShortAddCharFPN(uint16_t *A,  uint8_t *F,  uint16_t numpix,  uint8_t mult);
void ImgCharMakeFPN( uint8_t *F,  uint16_t numpix,  uint8_t modval);
void SubwinShort2D(uint16_t *I, uint16_t *S, uint8_t Irows, uint8_t Icols, uint8_t startrow, uint8_t numrows, uint8_t startcol, uint8_t numcols);
void SubwinShort2Dto1D(uint16_t *I, uint16_t *S, uint8_t Irows, uint8_t Icols, uint8_t subrow, uint8_t subcol, uint8_t Snumpix, uint8_t Spixlength, uint8_t orientation);
	
#endif
