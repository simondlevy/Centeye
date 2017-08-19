/*
OpticalFlow.cpp Functions to calculate optical flow and odometry

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

#include <OpticalFlow.h>
#include <stdint.h>

/*********************************************************************/
//	LPF
//	Send in a new optical flow measurement and the filtered optical
//	flow value will be changed by ((*new_OF)-(*filtered_OF))*alpha
//	alpha should be between 0-1 
/*********************************************************************/

void LPF(int16_t *filtered_OF, int16_t *new_OF, float alpha)
{
    (*filtered_OF)=(*filtered_OF)+((float)(*new_OF)-(*filtered_OF))	*alpha;
}

/*********************************************************************/
//	Accumulate
//	The current optical flow value is added to the accumulation sum
//	only if it crosses a threshold
/*********************************************************************/      
uint16_t Accumulate(int16_t *new_OF, int16_t *acc_OF, uint16_t threshold)
{
    uint16_t reset=0;

    if((*new_OF>threshold)||(*new_OF<-threshold))
    {
        *acc_OF+=*new_OF;
        reset=1;
    }

    return reset;
}


/*********************************************************************/
//	IIA_1D (uint16_t version)
//	This is a one dimensional version of the image interpolation
//	algorithm (IIA) as described by Prof. Mandyam Srinivasan. 
//	curr_img and last_img are input line images. numpix is the number 
//	of pixels in the image. scale is a scaling factor, mostly for the 
//	benefit of the fixed-poing arithmetic performed here. out is the 
//	resulting output optical flow.
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	scale: value of one pixel of motion (for scaling output)
//	out: pointer to integer value for output.
/*********************************************************************/

void IIA_1D(uint16_t *curr_img, uint16_t *last_img, uint8_t numpix, uint16_t scale, uint16_t *out) 
{
    uint16_t *pleft,*pright,*pone,*ptwo;
    long top,bottom;
    uint8_t i;
    int deltat,deltax;

    // Set up pointers
    pleft = curr_img;	//left-shifted image
    pone = curr_img+1;	//center image
    pright = curr_img+2;	//right-shifted image
    ptwo = last_img+1;	//time-shifted image

    top=bottom=0;

    for(i=0; i<numpix-2; ++i) 
    {
        deltat = *ptwo - *pone;    // temporal gradient i.e. pixel change over time
        deltax = *pright - *pleft; // spatial gradient i.e. pixel  change over space

        top += deltat * deltax;		//top summation
        bottom += deltax * deltax;	//bottom summation

        // Increment pointers
        pleft++;
        pone++;
        pright++;
        ptwo++;
    }

    // Compute final output. Note use of "scale" here to multiply 2*top       
    // to a larger number so that it may be meaningfully divided using   
    // fixed point arithmetic
    *out = 2*top*scale/bottom;
}

/*********************************************************************/
//	IIA_1D (uint8_t version)
//	This is a one dimensional version of the image interpolation
//	algorithm (IIA) as described by Prof. Mandyam Srinivasan. 
//	curr_img and last_img are input line images. numpix is the 
//  number of pixels in the image. scale is a scaling factor, mostly 
//  for the benefit of the fixed-poing arithmetic performed here. out 
//  is the resulting output optical flow.
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	scale: value of one pixel of motion (for scaling output)
//	out: pointer to integer value for output.
/*********************************************************************/

void IIA_1D(uint8_t *curr_img, uint8_t *last_img, uint8_t 	numpix, uint16_t scale, uint16_t *out) 
{
    uint8_t *pleft,*pright,*pone,*ptwo;
    long top,bottom;
    int deltat,deltax;
    uint8_t i;

    // Set up pointers
    pleft = curr_img;	//left-shifted image
    pone = curr_img+1;	//center image
    pright = curr_img+2;	//right-shifted image
    ptwo = last_img+1;	//time-shifted image

    top=bottom=0;

    for(i=0; i<numpix-2; ++i) 
    {
        deltat = (int)(*ptwo) - (int)(*pone); // temporal gradient i.e. 	// pixel change over time
        deltax = (int)(*pright) - (int)(*pleft); // spatial gradient i.e. 	//pixel change over space
        top += deltat * deltax;
        bottom += deltax * deltax;

        // Increment pointers
        pleft++;
        pone++;
        pright++;
        ptwo++;
    }

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    *out = 2*top*scale/bottom;
}


/*********************************************************************/
//	IIA_Plus_2D (uint8_t version)
//	This function computes optical flow between two images curr_img //	and last_img using a simplified version of Mandyam Srinivasan's //	image interpolation algorithm. This algorithm assumes that 
//	displacements are generally on the order of one pixel or less. 
//
//	Credit- Thanks to "A.J." on Embedded Eye for optimizing this 
//	function.
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void IIA_Plus_2D(uint8_t *curr_img, uint8_t *last_img, uint16_t rows,uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A=0, BD=0, C=0, E=0, F=0;
    int16_t  F2F1, F4F3, FCF0;

    // set up pointers
    uint8_t *f0 = curr_img + cols + 1; //center image
    uint8_t *f1 = curr_img + cols + 2; //right-shifted image
    uint8_t *f2 = curr_img + cols;     //left-shifted image	
    uint8_t *f3 = curr_img + 2*cols + 1; //down-shifted image	
    uint8_t *f4 = curr_img + 1;		//up-shifted image
    uint8_t *fz = last_img + cols + 1; 	//time-shifted image


    // loop through
    for (uint8_t r=1; r<rows-1; ++r) 
    { 
        for (uint8_t c=1; c<cols-1; ++c) 
        { 
            // compute differentials, then increment pointers 
            F2F1 = (*(f2++) - *(f1++));
            F4F3 = (*(f4++) - *(f3++));
            FCF0 = (*(fz++) - *(f0++));

            // update summations
            A += (F2F1 * F2F1);
            BD += (F4F3 * F2F1);
            C  += (FCF0 * F2F1);                   
            E += (F4F3 * F4F3);
            F  += (FCF0 * F4F3);                                   
        }

        f0+=2;	//move to next row of image
        fz+=2;
        f1+=2;
        f2+=2;
        f3+=2;
        f4+=2;
    }

    int64_t top1=( (int64_t)(C)*E - (int64_t)(F)*BD );
    int64_t top2=( (int64_t)(A)*F - (int64_t)(C)*BD );
    int64_t bottom=( (int64_t)(A)*E - (int64_t)(BD)*BD );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = (2*scale*top1)/bottom;
    int64_t YS = (2*scale*top2)/bottom;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}


/*********************************************************************/
//	IIA_Plus_2D (uint16_t version)
//	This function computes optical flow between two images curr_img 
//	and last_img using a simplified version of Mandyam Srinivasan's 
//	image interpolation algorithm. This algorithm assumes that 
//	displacements are generally on the order of one pixel or less. 
//
//	Credit- Thanks to "A.J." on Embedded Eye for optimizing this 
//	function.
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void IIA_Plus_2D(uint16_t *curr_img, uint16_t *last_img, uint16_t rows,uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A=0, BD=0, C=0, E=0, F=0;
    int16_t  F2F1, F4F3, FCF0;

    // set up pointers
    uint16_t *f0 = curr_img + cols + 1; //center image
    uint16_t *f1 = curr_img + cols + 2; //right-shifted image
    uint16_t *f2 = curr_img + cols;     //left-shifted image	
    uint16_t *f3 = curr_img + 2*cols + 1; //down-shifted image	
    uint16_t *f4 = curr_img + 1;		//up-shifted image
    uint16_t *fz = last_img + cols + 1; 	//time-shifted image


    // loop through
    for (uint8_t r=1; r<rows-1; ++r) 
    { 
        for (uint8_t c=1; c<cols-1; ++c) 
        { 
            // compute differentials, then increment pointers 
            F2F1 = (*(f2++) - *(f1++));
            F4F3 = (*(f4++) - *(f3++));
            FCF0 = (*(fz++) - *(f0++));

            // update summations
            A += (F2F1 * F2F1);
            BD += (F4F3 * F2F1);
            C  += (FCF0 * F2F1);                   
            E += (F4F3 * F4F3);
            F  += (FCF0 * F4F3);                                   
        }

        f0+=2;	//move to next row of image
        fz+=2;
        f1+=2;
        f2+=2;
        f3+=2;
        f4+=2;
    }

    int64_t top1=( (int64_t)(C)*E - (int64_t)(F)*BD );
    int64_t top2=( (int64_t)(A)*F - (int64_t)(C)*BD );
    int64_t bottom=( (int64_t)(A)*E - (int64_t)(BD)*BD );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = (2*scale*top1)/bottom;
    int64_t YS = (2*scale*top2)/bottom;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}

/*********************************************************************/
//	IIA_Square_2D (uint8_t version)
//	This function computes optical flow between two images curr_img //	and last_img using a simplified version of Mandyam Srinivasan's //	image interpolation algorithm. This algorithm assumes that 
//	displacements are generally on the order of one pixel or less. 
//	Instead of the traditional 'plus'
//	image shifts (top, bottom, left, right) here we do a more compact
//	square shift (top right, top left, bottom right, bottom left)
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void IIA_Square_2D(uint8_t *curr_img, uint8_t *last_img, uint16_t rows,uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A=0, BD=0, C=0, E=0, F=0;
    int16_t  F2F1, F4F3, FCF0;

    // set up pointers
    uint8_t *f0 = curr_img;//top left 
    uint8_t *f1 = curr_img + 1; 		//top right
    uint8_t *f2 = curr_img + cols; 	//bottom left
    uint8_t *f3 = curr_img + cols + 1; 	//bottom right
    uint8_t *fz = last_img; 		//top left time-shifted

    // loop through
    for (uint8_t r=0; r<rows-1; ++r) 
    { 
        for (uint8_t c=0; c<cols-1; ++c) 
        { 
            // compute differentials
            F2F1 = ((*(f0) - *(f1)) + (*(f2) - *(f3))) ;
            F4F3 = ((*(f0) - *(f2)) + (*(f1) - *(f3))) ;
            FCF0 = (*(fz) - *(f0));

            //increment pointers
            f0++;
            fz++;
            f1++;
            f2++;
            f3++;

            // update summations
            A += (F2F1 * F2F1);
            BD += (F4F3 * F2F1);
            C  += (FCF0 * F2F1);                   
            E += (F4F3 * F4F3);
            F  += (FCF0 * F4F3);                                   
        }

        //go to next row
        f0++;
        fz++;
        f1++;
        f2++;
        f3++;
    }

    int64_t top1=( (int64_t)(C)*E - (int64_t)(F)*BD );
    int64_t top2=( (int64_t)(A)*F - (int64_t)(C)*BD );
    int64_t bottom=( (int64_t)(A)*E - (int64_t)(BD)*BD );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = (2*scale*top1)/bottom;
    int64_t YS = (2*scale*top2)/bottom;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}

/*********************************************************************/
//	IIA_Square_2D (uint16_t version)
//	This function computes optical flow between two images curr_img 
//	and last_img using a simplified version of Mandyam Srinivasan's 
//	image interpolation algorithm. This algorithm assumes that 
//	displacements are generally on the order of one pixel or less. 
//	Instead of the traditional 'plus'
//	image shifts (top, bottom, left, right) here we do a more compact
//	square shift (top right, top left, bottom right, bottom left)
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void IIA_Square_2D(uint16_t *curr_img,uint16_t *last_img, uint16_t rows,uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A=0, BD=0, C=0, E=0, F=0;
    int16_t  F2F1, F4F3, FCF0;

    // set up pointers
    uint16_t *f0 = curr_img;//top left 
    uint16_t *f1 = curr_img + 1; 		//top right
    uint16_t *f2 = curr_img + cols; 	//bottom left
    uint16_t *f3 = curr_img + cols + 1; 	//bottom right
    uint16_t *fz = last_img; 		//top left time-shifted

    // loop through
    for (uint8_t r=0; r<rows-1; ++r) 
    { 
        for (uint8_t c=0; c<cols-1; ++c) 
        { 
            // compute differentials
            F2F1 = ((*(f0) - *(f1)) + (*(f2) - *(f3))) ;
            F4F3 = ((*(f0) - *(f2)) + (*(f1) - *(f3))) ;
            FCF0 = (*(fz) - *(f0));

            //increment pointers
            f0++;
            fz++;
            f1++;
            f2++;
            f3++;

            // update summations
            A += (F2F1 * F2F1);
            BD += (F4F3 * F2F1);
            C  += (FCF0 * F2F1);                   
            E += (F4F3 * F4F3);
            F  += (FCF0 * F4F3);                                   
        }

        //go to next row
        f0++;
        fz++;
        f1++;
        f2++;
        f3++;
    }

    int64_t top1=( (int64_t)(C)*E - (int64_t)(F)*BD );
    int64_t top2=( (int64_t)(A)*F - (int64_t)(C)*BD );
    int64_t bottom=( (int64_t)(A)*E - (int64_t)(BD)*BD );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = (2*scale*top1)/bottom;
    int64_t YS = (2*scale*top2)/bottom;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}

/*********************************************************************/
//	LK_Plus_2D (uint8_t version)
//	This function computes optical flow between two images curr_img 
//	and last_img using a version of Lucas-Kanade's algorithm
//	This algorithm assumes that displacements are generally on 
//	the order of one pixel or less. 
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void LK_Plus_2D(uint8_t *curr_img, uint8_t *last_img, uint16_t rows,uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;
    int16_t  F2F1, F4F3, FCF0;

    // set up pointers
    uint8_t *f0 = curr_img + cols + 1; //center image
    uint8_t *f1 = curr_img + cols + 2; //right-shifted image
    uint8_t *f2 = curr_img + cols;     //left-shifted image	
    uint8_t *f3 = curr_img + 2*cols + 1; //down-shifted image	
    uint8_t *f4 = curr_img + 1;		//up-shifted image
    uint8_t *fz = last_img + cols + 1; 	//time-shifted image

    // loop through
    for (uint8_t r=1; r<rows-1; ++r) { 
        for (uint8_t c=1; c<cols-1; ++c) { 
            // compute differentials, then increment pointers (post 		// increment)
            F2F1 = (*(f2++) - *(f1++));	//horizontal differential
            F4F3 = (*(f4++) - *(f3++));	//vertical differential
            FCF0 = (*(fz++) - *(f0++));	//time differential

            // update summations
            A11 += (F2F1 * F2F1);
            A12 += (F4F3 * F2F1);
            A22 += (F4F3 * F4F3);
            b1  += (FCF0 * F2F1);                   
            b2  += (FCF0 * F4F3);                                   
        }
        f0+=2;	//move to next row of image
        fz+=2;
        f1+=2;
        f2+=2;
        f3+=2;
        f4+=2;
    }

    //determinant
    int64_t detA = ( (int64_t)(A11)*A22 - (int64_t)(A12)*A12 );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = ( (int64_t)(b1)*A22 - (int64_t)(b2)*A12 ) * scale / 		detA;
    int64_t YS = ( (int64_t)(b2)*A11 - (int64_t)(b1)*A12 ) * scale / 		detA;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}

/*********************************************************************/
//	LK_Plus_2D (uint16_t version)
//	This function computes optical flow between two images curr_img 
//	and last_img using a version of Lucas-Kanade's algorithm
//	This algorithm assumes that displacements are generally on 
//	the order of one pixel or less. 
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void LK_Plus_2D(uint16_t *curr_img, uint16_t *last_img, uint16_t rows,uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;
    int16_t  F2F1, F4F3, FCF0;

    // set up pointers
    uint16_t *f0 = curr_img + cols + 1; //center image
    uint16_t *f1 = curr_img + cols + 2; //right-shifted image
    uint16_t *f2 = curr_img + cols;     //left-shifted image	
    uint16_t *f3 = curr_img + 2*cols + 1; //down-shifted image	
    uint16_t *f4 = curr_img + 1;		//up-shifted image
    uint16_t *fz = last_img + cols + 1; 	//time-shifted image

    // loop through
    for (uint8_t r=1; r<rows-1; ++r) { 
        for (uint8_t c=1; c<cols-1; ++c) { 
            // compute differentials, then increment pointers (post 		// increment)
            F2F1 = (*(f2++) - *(f1++));	//horizontal differential
            F4F3 = (*(f4++) - *(f3++));	//vertical differential
            FCF0 = (*(fz++) - *(f0++));	//time differential

            // update summations
            A11 += (F2F1 * F2F1);
            A12 += (F4F3 * F2F1);
            A22 += (F4F3 * F4F3);
            b1  += (FCF0 * F2F1);                   
            b2  += (FCF0 * F4F3);                                   
        }
        f0+=2;	//move to next row of image
        fz+=2;
        f1+=2;
        f2+=2;
        f3+=2;
        f4+=2;
    }

    //determinant
    int64_t detA = ( (int64_t)(A11)*A22 - (int64_t)(A12)*A12 );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = ( (int64_t)(b1)*A22 - (int64_t)(b2)*A12 ) * scale / detA;
    int64_t YS = ( (int64_t)(b2)*A11 - (int64_t)(b1)*A12 ) * scale / detA;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}

/*********************************************************************/
//	LK_Square_2D (uint8_t version)
//	This function computes optical flow between two images curr_img 
//	and last_img using a version of Lucas-Kanade's algorithm
//	This algorithm assumes that displacements are generally on 
//	the order of one pixel or less. Instead of the traditional 'plus'
//	image shifts (top, bottom, left, right) here we do a more compact
//	square shift (two right minus left, two top minus bottom)
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void LK_Square_2D(uint8_t *curr_img, uint8_t *last_img, uint16_t rows,uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;
    int16_t  F2F1, F4F3, FCF0;

    // set up pointers
    uint8_t *f0 = curr_img;//top left 
    uint8_t *f1 = curr_img + 1; 		//top right
    uint8_t *f2 = curr_img + cols; 	//bottom left
    uint8_t *f3 = curr_img + cols + 1; 	//bottom right
    uint8_t *fz = last_img; 		//top left time-shifted

    // loop through
    for (uint8_t r=0; r<rows-1; ++r) 
    { 
        for (uint8_t c=0; c<cols-1; ++c) 
        { 
            // compute differentials      
            F2F1 = ((*(f0) - *(f1)) + (*(f2) - *(f3))) ;
            F4F3 = ((*(f0) - *(f2)) + (*(f1) - *(f3))) ;
            FCF0 = (*(fz) - *(f0));

            //increment pointers
            f0++;
            fz++;
            f1++;
            f2++;
            f3++;

            // update summations
            A11 += (F2F1 * F2F1);
            A12 += (F4F3 * F2F1);
            A22 += (F4F3 * F4F3);
            b1  += (FCF0 * F2F1);                   
            b2  += (FCF0 * F4F3);                                   
        }

        //go to next row
        f0++;
        fz++;
        f1++;
        f2++;
        f3++;
    }

    //compute determinant
    int64_t detA = ( (int64_t)(A11)*A22 - (int64_t)(A12)*A12 );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = ( (int64_t)(b1)*A22 - (int64_t)(b2)*A12 ) * scale / detA;
    int64_t YS = ( (int64_t)(b2)*A11 - (int64_t)(b1)*A12 ) * scale / detA;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}

/*********************************************************************/
//	LK_Square_2D (uint16_t version)
//	This function computes optical flow between two images curr_img 
//	and last_img using a version of Lucas-Kanade's algorithm
//	This algorithm assumes that displacements are generally on 
//	the order of one pixel or less. Instead of the traditional 'plus'
//	image shifts (top, bottom, left, right) here we do a more compact
//	square shift (top right, top left, bottom right, bottom left)
//
//	VARIABLES:
//	curr_img,last_img: first and second images
//	numpix: number of pixels in line image
//	rows: number of rows in image
//	cols: number of cols in image
//	scale: value of one pixel of motion (for scaling output)
//	ofx: pointer to integer value for X shift.
//	ofy: pointer to integer value for Y shift.
/*********************************************************************/

void LK_Square_2D(uint16_t *curr_img, uint16_t *last_img, uint16_t rows, uint16_t cols, uint16_t scale, int16_t * ofx, int16_t * ofy)
{
    int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;
    int16_t  F2F1, F4F3, FCF0;

    // set up pointers
    uint16_t *f0 = curr_img;//top left 
    uint16_t *f1 = curr_img + 1; 		//top right
    uint16_t *f2 = curr_img + cols; 	//bottom left
    uint16_t *f3 = curr_img + cols + 1; 	//bottom right
    uint16_t *fz = last_img; 		//top left time-shifted


    // loop through
    for (uint8_t r=0; r<rows-1; ++r) 
    { 
        for (uint8_t c=0; c<cols-1; ++c) 
        { 
            // compute differentials
            F2F1 = (*(f0) - *(f1)) + (*(f2) - *(f3))  ;
            F4F3 = (*(f0) - *(f2)) + (*(f1) - *(f3))  ;
            FCF0 = (*(fz) - *(f0));

            //increment pointers
            f0++;
            fz++;
            f1++;
            f2++;
            f3++;

            // update summations
            A11 += (F2F1 * F2F1);
            A12 += (F4F3 * F2F1);
            A22 += (F4F3 * F4F3);
            b1  += (FCF0 * F2F1);                   
            b2  += (FCF0 * F4F3);                                   
        }

        //go to next row
        f0++;
        fz++;
        f1++;
        f2++;
        f3++;
    }

    //compute determinant
    int64_t detA = ( (int64_t)(A11)*A22 - (int64_t)(A12)*A12 );

    // Compute final output. Note use of "scale" here to multiply 2*top   
    // to a larger number so that it may be meaningfully divided using 
    // fixed point arithmetic
    int64_t XS = ( (int64_t)(b1)*A22 - (int64_t)(b2)*A12 ) * scale / detA;
    int64_t YS = ( (int64_t)(b2)*A11 - (int64_t)(b1)*A12 ) * scale / detA;

    (*ofx) = (int16_t)XS;
    (*ofy) = (int16_t)YS;
}
