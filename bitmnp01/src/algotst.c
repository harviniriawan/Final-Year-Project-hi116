/* 
* Copyright( c )1998 and 1999 by the EDN Embedded Microprocessor 
* Benchmark Consortium( EEMBC ), Inc. 
* 
* All Rights Reserved. This is licensed program product and 
* is owned by EEMBC. The Licensee understands and agrees that the 
* Benchmarks licensed by EEMBC hereunder( including methods or concepts 
* utilized therein )contain certain information that is confidential 
* and proprietary which the Licensee expressly agrees to retain in the 
* strictest confidence and to use only in conjunction with the Benchmarks 
* pursuant to the terms of this Agreement. The Licensee further agrees 
* to keep the source code and all related documentation confidential and 
* not to disclose such source code and/or related documentation to any 
* third party. The Licensee and any READER of this code is subject to 
* either the EEMBC Member License Agreement and/or the EEMBC Licensee 
* Agreement. 
* TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, EEMBC DISCLAIMS ALL 
* WARRANTIES, EITHER EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, 
* IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR 
* PURPOSE, WITH REGARD TO THE BENCHMARKS AND THE ACCOMPANYING 
* DOCUMENTATION. LICENSEE ACKNOWLEDGES AND AGREES THAT THERE ARE NO 
* WARRANTIES, GUARANTIES, CONDITIONS, COVENANTS, OR REPRESENTATIONS BY 
* EEMBC AS TO MARKETABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR OTHER 
* ATTRIBUTES, WHETHER EXPRESS OR IMPLIED( IN LAW OR IN FACT ), ORAL OR 
* WRITTEN. 
* 
* Licensee hereby agrees by accessing this source code that all benchmark 
* scores related to this code must be certified by ECL prior to publication 
* in any media, form, distribution, or other means of conveyance of 
* information subject to the terms of the EEMBC Member License Agreement 
* and/or EEMBC Licensee Agreement. 
* 
* Other Copyright Notice( if any ) : 
* For conditions of distribution and use, see the accompanying README file. 
* 
* File : algotst.c
* Subcommittee : EEMBC Automotive/Industrial Subcommittee 
* Algorithm # 8 :  Bit manipulation -- test data functions - bitmnp01
* Author : dt 
* Origin Date : 08/12/98
* Current Internal_Revision : 1.0
* Revision History : 
* 
*   5/14/99 pjt     1.0
*       Initial cleanup
*   7/4/99  arw     1.0
*       Added eembc_dt.h datatypes & additional cleanup
*       "thlib.h" will #include eembc_dt.h for us
* 
* 
* - END OF HEADER - 
*/ 

/*******************************************************************************
    Includes - thlib.h now includes eembc_dt.h                                                                   
*******************************************************************************/

#include "thlib.h"
#include "algo.h"

/*  DECLARATIONS  */

/*
 * The following test data represents numbers to be displayed.
 *
 */    

const varsize inpVariableROM[] = 
{
    1, 

    0, 

    2, 

    0, 

    3, 

    0, 

    4, 

    0, 

    5, 

    0, 

    6, 

    0, 

    7, 

    0, 

    8, 

    0, 

    9, 

    0, 

    0, 

    0, 

    1, 

    0, 

    2, 

    0, 

    3, 

    0, 

    4, 

    0, 

    5, 

    0, 

    6, 

    0, 

    7, 

    0, 

    8, 

    0, 

    9, 

    0, 

    0, 

    0, 

    1, 

    0, 

    2, 

    0, 

    3, 

    0, 

    4, 

    0, 

    5, 

    0, 

    6, 

    0, 

    7, 

    0, 

    8, 

    0, 

    9, 

    0, 

    0, 

    0, 

    1, 

    0, 

    2, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    10, 

    0, 

    1, 

    1, 

    2, 

    1, 

    3, 

    1, 

    4, 

    1, 

    5, 

    1, 

    6, 

    1, 

    7, 

    1, 

    8, 

    1, 

    9, 

    1, 

    0, 

    1, 

    1, 

    1, 

    2, 

    1, 

    3, 

    1, 

    4, 

    1, 

    5, 

    1, 

    6, 

    1, 

    7, 

    1, 

    8, 

    1, 

    9, 

    1, 

    0, 

    1, 

    1, 

    1, 

    2, 

    1, 

    3, 

    1, 

    4, 

    1, 

    5, 

    1, 

    6, 

    1, 

    7, 

    1, 

    8, 

    1, 

    9, 

    1, 

    0, 

    1, 

    1, 

    1, 

    2, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1, 

    10, 

    1

} ; /* End of test values :  inpVariableROM[] */

varsize digitROM[] =
{       

    0xFE, 

    0xFE, 

    0xFE, 

    0xFE, 

    0xFE,    /* Solid character for ERROR */

    0x7C, 

    0xA2, 

    0x92, 

    0x8A, 

    0x7C,    /* Character '0' */

    0x00, 

    0x82, 

    0xFE, 

    0x80, 

    0x00,    /* Character '1' */

    0x84, 

    0xC2, 

    0xA2, 

    0x92, 

    0x8C,    /* Character '2' */

    0x42, 

    0x82, 

    0x8A, 

    0x96, 

    0x62,    /* Character '3' */

    0x30, 

    0x28, 

    0x24, 

    0xFE, 

    0x20,    /* Character '4' */

    0x4E, 

    0x8A, 

    0x8A, 

    0x8A, 

    0x72,    /* Character '5' */

    0x78, 

    0x94, 

    0x92, 

    0x92, 

    0x60,    /* Character '6' */

    0x02, 

    0x02, 

    0xE2, 

    0x1A, 

    0x06,    /* Character '7' */

    0x6C, 

    0x92, 

    0x92, 

    0x92, 

    0x6C,    /* Character '8' */

    0x0C, 

    0x92, 

    0x92, 

    0x52, 

    0x3C,    /* Character '9' */

    0x00, 

    0x00, 

    0x00, 

    0x00, 

    0x00    /* Blank character */

} ; /* End of variable 'charset[]' */

/*******************************************************************************
    Functions                                                                   
*******************************************************************************/

/*
*  Function :  GetTestData
*
*    Builds the table of input data which represents 'inpMode', and 'inputNum' the
*    value to be bit-manipulated.  This is the "real world" data stream which
*    drives the algorithm.
*
*/

n_int
GetTestData( n_void )
{        
    n_int i ;
    n_int j ; 

    inpNumber = (varsize*)
        th_malloc( ( NUM_TESTS + 1 ) * sizeof( varsize ) ) ; 
    inpMode = (int*)
        th_malloc( ( NUM_TESTS + 1 ) * sizeof( n_int ) ) ; 

    if( ( inpNumber == NULL ) || ( inpMode == NULL ) )
    {
        return false  ;
    }

    /*  Copy the test values from ROM to RAM  */    
    for( i = 0, j = 0 ; i < NUM_TESTS ; i++, j++ )
    {
        inpNumber[i] = (varsize)inpVariableROM[j++ ] ; 
        inpMode[i] = (int)inpVariableROM[j] ; 
    }

    return true ; 

} /* End of function 'GetTestData' */

/*
*  Function :  GetInputValues
*
*    On each pass of the table lookup, a value must be input for 'inputNum'.
*    and 'inverted'.  Each time this function is called, the next input value is
*    pulled from the table in RAM.  The table wraps around, so that input data is
*    continuous.     A flag is returned TRUE whenever the table wraps around.
*
*/    

n_int
GetInputValues( n_void )
{        
    inputNum = inpNumber[tableCount] ; 
    inverted = inpMode[tableCount] ; 

    /* If you run out of test data, then start table over */    
    if( ++tableCount < NUM_TESTS )
    {
        return false ; 
    }

    tableCount = 0 ; 
    return true ; 

} /* End of function 'GetInputValues' */




