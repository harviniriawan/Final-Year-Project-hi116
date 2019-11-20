/*
 * algotst.c
 *
 *  Created on: 12 Nov 2019
 *      Author: harvi
 */
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
* Algorithm #13 :  Cache Buster - cacheb01
* Author : dt
* Origin Date :  10/20/98
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
    Includes
*******************************************************************************/

#include "algo.h"

/*  DECLARATIONS  */

/*
* The following test data contains indicies to functions and their
* corresponding arguments. As long as the indicies are all different, then
* the source/destination locations will be at least( ARRAY_SIZE * sizeof(
* varsize ) )apart.  For ARRAY_SIZE=2048 and varsize=LONG, then the source/
* destination arguments will be separated by at least 8192 bytes when the
* difference between arguments is >=2.  But keep in mind that each function
* has mixed up the arguments so that you must inspect each particular function
* to see which argument is used for which purpose.  Three arguments are used
* to index into the 'dataPtrTable' to find out which array is used for
* 'source1', 'source2', and 'dest'.  Then the other three arguments are
* used to index within the source and destination arrays to a particular
* location. The 'func' index picks which function to be performed.
*
*/

const varsize inpTokenROM[] =
{
    0,

    0,

    0,

    0,

    0,

    0,

    0,    /* func, arg1, arg2, arg3, arg4, arg5, arg6 */

    1,

    1,

    1,

    1,

    1,

    1,

    1,    /* First block of tests are just for testing */

    2,

    2,

    2,

    2,

    2,

    2,

    2,

    3,

    3,

    3,

    3,

    3,

    3,

    3,

    4,

    4,

    4,

    4,

    4,

    4,

    4,

    5,

    5,

    5,

    5,

    5,

    5,

    5,

    6,

    6,

    6,

    6,

    6,

    6,

    6,

    7,

    7,

    7,

    7,

    7,

    7,

    7,

    8,

    8,

    8,

    8,

    8,

    8,

    8,

    9,

    9,

    9,

    9,

    9,

    9,

    9,

    10,

    10,

    10,

    10,

    10,

    10,

    10,

    11,

    11,

    11,

    11,

    11,

    11,

    11,

    12,

    12,

    12,

    12,

    12,

    12,

    12,

    13,

    13,

    13,

    13,

    13,

    13,

    13,

    14,

    14,

    14,

    14,

    14,

    14,

    14,

    15,

    15,

    15,

    15,

    15,

    15,

    15,


    0,

    1,

    3,

    5,

    7,

    9,

    11,    /* Here's where we start jumping around */

    7,

    2,

    4,

    6,

    8,

    10,

    12,    /*  not only in ROM between functions, but */

    14,

    3,

    5,

    7,

    9,

    11,

    13,    /*  also in RAM between source/destination */

    6,

    4,

    6,

    8,

    10,

    12,

    14,

    13,

    1,

    5,

    7,

    2,

    4,

    6,

    5,

    15,

    2,

    13,

    4,

    10,

    11

} ; /* End of test values :  inpTokenROM[] */

/*
* The following table contains indicies to select a function to perform.
*
*/

funcPtr funcPtrTable[] =
{
    function1,

    function2,

    function3,

    function5,

    function7,

    function11,

    function13,

    function4,

    function6,

    function8,

    function9,

    function10,

    function12,

    function14,

    function15,

    function16

} ; /* End of table of pointer to functions 'funcPtrTable' */

/*
* The following table contains indicies to select an array in RAM to
* serve as a source or  destination for a function.
*
*/

varsize *dataPtrTable[NUM_ARRAYS] =
{
    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

    NULL,

} ; /* End of table of pointers to data 'dataPtrTable' */

/*******************************************************************************
    Functions
*******************************************************************************/

/*
*  Function :  GetTestData
*
*  Builds the table of input data which represents the sampled
*  input signal.  This is the "real world" data stream which
*  drives the algorithm.
*
*/

n_int
GetTestData( n_void )
{
    n_int i ;

    inputToken = (varsize*)
        th_malloc( ( NUM_TESTS + 1 ) * sizeof( varsize ) ) ;

    if( inputToken == NULL )
    {
        return false ;
    }

    /* Copy the test values from ROM to RAM  */
    for( i = 0 ; i < NUM_TESTS ; i++ )
    {
        inputToken[i] = inpTokenROM[i] ;
    }

    return true ;

} /* End of function 'GetTestData' */

/*
*  Function :  GetInputValues
*
*  On each pass of the table lookup, a value must be input for 'realData'
*  and 'imagData'.  Each time this function is called, the next inputs are
*  pulled from the table in RAM.  The table wraps around, so that data is
*  continuous. A flag is returned TRUE whenever the table wraps around.
*
*/

n_int
GetInputValues( n_void )
{
    func = inputToken[ tableCount++ ] ; /* Get the function selector */
    arg1 = inputToken[ tableCount++ ] ; /* ...and the associated arguments */
    arg2 = inputToken[ tableCount++ ] ;
    arg3 = inputToken[ tableCount++ ] ;
    arg4 = inputToken[ tableCount++ ] ;
    arg5 = inputToken[ tableCount++ ] ;
    arg6 = inputToken[ tableCount++ ] ;

    /* If not the end of the input test data */
    if( tableCount < NUM_TESTS )
    {
        /* ...then we're done... */
        return 0  ;
    }

    /* Else, reset the input test data pointer */
    tableCount = 0 ;
    return  1  ;

} /* End of function 'GetInputValues' */



