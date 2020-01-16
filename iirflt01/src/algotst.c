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
* Algorithm # 7 :  Low-Pass Filter( IIR )and DSP functions
* Author : dt 
* Origin Date : 
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
* 
* - END OF HEADER - 
*/ 

/*******************************************************************************
    Includes                                                                    
*******************************************************************************/

#include "algo.h"

/*  DECLARATIONS  */    

/*
* The following test data represents a real-time input signal
* which the algorithm must low-pass filter using the IIR filter.
*
*/    

const varsize inpSignalROM[] = 
{        
    10000, 

    3900, 

    7500, 

    7400, 

    3400, 

    9200, 

    2800, 

    6100, 

    5700, 

    1500, 

    7000, 

    400, 

    3600, 

    3100, 

    -1100, 

    4200, 

    -2300, 

    800, 

    300, 

    -3800, 

    1600, 

    -4700, 

    -1400, 

    -1700, 

    -5700, 

    000, 

    -6200, 

    -2600, 

    -2600, 

    -6400, 

    -300, 

    -6300, 

    -2500, 

    -2300, 

    -5900, 

    300, 

    -5400, 

    -1400, 

    -1100, 

    -4600, 

    1600, 

    -4000, 

    -100, 

    100, 

    -3300, 

    2800, 

    -3000, 

    800, 

    900, 

    -2700, 

    3300, 

    -2700, 

    900, 

    800, 

    -3000, 

    2800, 

    -3300, 

    100, 

    -100, 

    -4000, 

    1600, 

    -4600, 

    -1100, 

    -1400, 

    -5400, 

    300, 

    -5900, 

    -2300, 

    -2500, 

    -6300, 

    -300, 

    -6400, 

    -2600, 

    -2600, 

    -6200, 

    000, 

    -5700, 

    -1700, 

    -1400, 

    -4700, 

    1600, 

    -3800, 

    300, 

    800, 

    -2300, 

    4200, 

    -1100, 

    3100, 

    3600, 

    400, 

    7000, 

    1500, 

    5700, 

    6100, 

    2800, 

    9200, 

    3400, 

    7400, 

    7500, 

    3900, 

    10000, 

    3900, 

    7500, 

    7400, 

    3400, 

    9200, 

    2800, 

    6100, 

    5700, 

    1500, 

    7000, 

    400, 

    3600, 

    3100, 

    -1100, 

    4200, 

    -2300, 

    800, 

    300, 

    -3800, 

    1600, 

    -4700, 

    -1400, 

    -1700, 

    -5700, 

    000, 

    -6200, 

    -2600, 

    -2600, 

    -6400, 

    -300, 

    -6300, 

    -2500, 

    -2300, 

    -5900, 

    300, 

    -5400, 

    -1400, 

    -1100, 

    -4600, 

    1600, 

    -4000, 

    -100, 

    100, 

    -3300, 

    2800, 

    -3000, 

    800, 

    900, 

    -2700, 

    3300, 

    -2700, 

    900, 

    800, 

    -3000, 

    2800, 

    -3300, 

    100, 

    -100, 

    -4000, 

    1600, 

    -4600, 

    -1100, 

    -1400, 

    -5400, 

    300, 

    -5900, 

    -2300, 

    -2500, 

    -6300, 

    -300, 

    -6400, 

    -2600, 

    -2600, 

    -6200, 

    000, 

    -5700, 

    -1700, 

    -1400, 

    -4700, 

    1600, 

    -3800, 

    300, 

    800, 

    -2300, 

    4200, 

    -1100, 

    3100, 

    3600, 

    400, 

    7000, 

    1500, 

    5700, 

    6100, 

    2800, 

    9200, 

    3400, 

    7400, 

    7500, 

    3900, 

    10000, 

    3900, 

    7500, 

    7400, 

    3400, 

    9200, 

    2800, 

    6100, 

    5700, 

    1500, 

    7000, 

    400, 

    3600, 

    3100, 

    -1100, 

    4200, 

    -2300, 

    800, 

    300, 

    -3800, 

    1600, 

    -4700, 

    -1400, 

    -1700, 

    -5700, 

    000, 

    -6200, 

    -2600, 

    -2600, 

    -6400, 

    -300, 

    -6300, 

    -2500, 

    -2300, 

    -5900, 

    300, 

    -5400, 

    -1400, 

    -1100, 

    -4600, 

    1600, 

    -4000, 

    -100, 

    100, 

    -3300, 

    2800, 

    -3000, 

    800, 

    900, 

    -2700, 

    3300, 

    -2700, 

    900, 

    800, 

    -3000, 

    2800, 

    -3300, 

    100, 

    -100, 

    -4000, 

    1600, 

    -4600, 

    -1100, 

    -1400, 

    -5400, 

    300, 

    -5900, 

    -2300, 

    -2500, 

    -6300, 

    -300, 

    -6400, 

    -2600, 

    -2600, 

    -6200, 

    000, 

    -5700, 

    -1700, 

    -1400, 

    -4700, 

    1600, 

    -3800, 

    300, 

    800, 

    -2300, 

    4200, 

    -1100, 

    3100, 

    3600, 

    400, 

    7000, 

    1500, 

    5700, 

    6100, 

    2800, 

    9200, 

    3400, 

    7400, 

    7500, 

    3900, 

    10000, 

    3900, 

    7500, 

    7400, 

    3400, 

    9200, 

    2800, 

    6100, 

    5700, 

    1500, 

    7000, 

    400, 

    3600, 

    3100, 

    -1100, 

    4200, 

    -2300, 

    800, 

    300, 

    -3800, 

    1600, 

    -4700, 

    -1400, 

    -1700, 

    -5700, 

    000, 

    -6200, 

    -2600, 

    -2600, 

    -6400, 

    -300, 

    -6300, 

    -2500, 

    -2300, 

    -5900, 

    300, 

    -5400, 

    -1400, 

    -1100, 

    -4600, 

    1600, 

    -4000, 

    -100, 

    100, 

    -3300, 

    2800, 

    -3000, 

    800, 

    900, 

    -2700, 

    3300, 

    -2700, 

    900, 

    800, 

    -3000, 

    2800, 

    -3300, 

    100, 

    -100, 

    -4000, 

    1600, 

    -4600, 

    -1100, 

    -1400, 

    -5400, 

    300, 

    -5900, 

    -2300, 

    -2500, 

    -6300, 

    -300, 

    -6400, 

    -2600, 

    -2600, 

    -6200, 

    000, 

    -5700, 

    -1700, 

    -1400, 

    -4700, 

    1600, 

    -3800, 

    300, 

    800, 

    -2300, 

    4200, 

    -1100, 

    3100, 

    3600, 

    400, 

    7000, 

    1500, 

    5700, 

    6100, 

    2800, 

    9200, 

    3400, 

    7400, 

    7500, 

    3900, 

    10000, 

    3900, 

    7500, 

    7400, 

    3400, 

    9200, 

    2800, 

    6100, 

    5700, 

    1500, 

    7000, 

    400, 

    3600, 

    3100, 

    -1100, 

    4200, 

    -2300, 

    800, 

    300, 

    -3800, 

    1600, 

    -4700, 

    -1400, 

    -1700, 

    -5700, 

    000, 

    -6200, 

    -2600, 

    -2600, 

    -6400, 

    -300, 

    -6300, 

    -2500, 

    -2300, 

    -5900, 

    300, 

    -5400, 

    -1400, 

    -1100, 

    -4600, 

    1600, 

    -4000, 

    -100, 

    100, 

    -3300, 

    2800, 

    -3000, 

    800, 

    900, 

    -2700, 

    3300, 

    -2700, 

    900, 

    800, 

    -3000, 

    2800, 

    -3300, 

    100, 

    -100, 

    -4000, 

    1600, 

    -4600, 

    -1100, 

    -1400, 

    -5400, 

    300, 

    -5900, 

    -2300, 

    -2500, 

    -6300, 

    -300, 

    -6400, 

    -2600, 

    -2600, 

    -6200, 

    000, 

    -5700, 

    -1700, 

    -1400, 

    -4700, 

    1600, 

    -3800, 

    300, 

    800, 

    -2300, 

    4200, 

    -1100, 

    3100, 

    3600, 

    400, 

    7000, 

    1500, 

    5700, 

    6100, 

    2800, 

    9200, 

    3400, 

    7400, 

    7500, 

    3900, 

    10000, 

    3900, 

    7500, 

    7400, 

    3400, 

    9200, 

    2800, 

    6100, 

    5700, 

    1500, 

    7000, 

    400, 

    3600, 

    3100, 

    -1100, 

    4200, 

    -2300, 

    800, 

    300, 

    -3800, 

    1600, 

    -4700, 

    -1400, 

    -1700, 

    -5700, 

    000, 

    -6200, 

    -2600, 

    -2600, 

    -6400, 

    -300, 

    -6300, 

    -2500, 

    -2300, 

    -5900, 

    300, 

    -5400, 

    -1400, 

    -1100, 

    -4600, 

    1600, 

    -4000, 

    -100, 

    100, 

    -3300, 

    2800, 

    -3000, 

    800, 

    900, 

    -2700, 

    3300, 

    -2700, 

    900, 

    800, 

    -3000, 

    2800, 

    -3300, 

    100, 

    -100, 

    -4000, 

    1600, 

    -4600, 

    -1100, 

    -1400, 

    -5400, 

    300, 

    -5900, 

    -2300, 

    -2500, 

    -6300, 

    -300, 

    -6400, 

    -2600, 

    -2600, 

    -6200, 

    000, 

    -5700, 

    -1700, 

    -1400, 

    -4700, 

    1600, 

    -3800, 

    300, 

    800, 

    -2300, 

    4200, 

    -1100, 

    3100, 

    3600, 

    400, 

    7000, 

    1500, 

    5700, 

    6100, 

    2800, 

    9200, 

    3400, 

    7400, 

    7500, 

    3900, 

    10000, 

    3900, 

    7500, 

    7400, 

    3400, 

    9200, 

    2800, 

    6100, 

    5700, 

    1500, 

    7000, 

    400, 

    3600, 

    3100, 

    -1100, 

    4200, 

    -2300, 

    800, 

    300, 

    -3800, 

    1600, 

    -4700, 

    -1400, 

    -1700, 

    -5700, 

    000, 

    -6200, 

    -2600, 

    -2600, 

    -6400, 

    -300, 

    -6300, 

    -2500, 

    -2300, 

    -5900, 

    300, 

    -5400, 

    -1400, 

    -1100, 

    -4600, 

    1600, 

    -4000, 

    -100, 

    100, 

    -3300, 

    2800, 

    -3000, 

    800, 

    900, 

    -2700, 

    3300, 

    -2700, 

    900, 

    800, 

    -3000, 

    2800, 

    -3300, 

    100, 

    -100, 

    -4000, 

    1600, 

    -4600, 

    -1100, 

    -1400, 

    -5400, 

    300, 

    -5900, 

    -2300, 

    -2500, 

    -6300, 

    -300, 

    -6400, 

    -2600, 

    -2600, 

    -6200, 

    000, 

    -5700, 

    -1700, 

    -1400, 

    -4700, 

    1600, 

    -3800, 

    300, 

    800, 

    -2300, 

    4200, 

    -1100, 

    3100, 

    3600, 

    400, 

    7000, 

    1500, 

    5700, 

    6100, 

    2800, 

    9200, 

    3400, 

    7400, 

    7500, 

    3900, 

    10000, 

    3900, 

    7500, 

    7400, 

    3400, 

    9200, 

    2800, 

    6100, 

    5700, 

    1500, 

    7000, 

    400, 

    3600, 

    3100, 

    -1100, 

    4200, 

    -2300, 

    800, 

    300, 

    -3800, 

    1600, 

    -4700, 

    -1400, 

    -1700, 

    -5700, 

    000, 

    -6200, 

    -2600, 

    -2600, 

    -6400, 

    -300, 

    -6300, 

    -2500, 

    -2300, 

    -5900, 

    300, 

    -5400, 

    -1400, 

    -1100, 

    -4600, 

    1600, 

    -4000, 

    -100, 

    100, 

    -3300, 

    2800, 

    -3000, 

    800, 

    900, 

    -2700, 

    3300, 

    -2700, 

    900, 

    800, 

    -3000, 

    2800, 

    -3300, 

    100, 

    -100, 

    -4000, 

    1600, 

    -4600, 

    -1100, 

    -1400, 

    -5400, 

    300, 

    -5900, 

    -2300, 

    -2500, 

    -6300, 

    -300, 

    -6400, 

    -2600, 

    -2600, 

    -6200, 

    000, 

    -5700, 

    -1700, 

    -1400, 

    -4700, 

    1600, 

    -3800, 

    300, 

    800, 

    -2300, 

    4200, 

    -1100, 

    3100, 

    3600, 

    400, 

    7000, 

    1500, 

    5700, 

    6100, 

    2800, 

    9200, 

    3400, 

    7400, 

    7500, 

    3900, 

    10000, 

    3900, 

    7500, 

    7400, 

    3400, 

    9200, 

    2800, 

    6100, 

    5700, 

    1500, 

    7000, 

    400, 

    3600, 

    3100, 

    -1100, 

    4200, 

    -2300, 

    800, 

    300, 

    -3800, 

    1600, 

    -4700, 

    -1400, 

    -1700, 

    -5700, 

    000, 

    -6200, 

    -2600, 

    -2600, 

    -6400, 

    -300, 

    -6300, 

    -2500, 

    -2300, 

    -5900, 

    300, 

    -5400, 

    -1400, 

    -1100, 

    -4600, 

    1600, 

    -4000, 

    -100, 

    100, 

    -3300, 

    2800, 

    -3000, 

    800, 

    900, 

    -2700, 

    3300, 

    -2700, 

    900, 

    800, 

    -3000, 

    2800, 

    -3300, 

    100, 

    -100, 

    -4000, 

    1600, 

    -4600, 

    -1100, 

    -1400, 

    -5400, 

    300, 

    -5900, 

    -2300, 

    -2500, 

    -6300, 

    -300, 

    -6400, 

    -2600, 

    -2600, 

    -6200, 

    000, 

    -5700, 

    -1700, 

    -1400, 

    -4700, 

    1600, 

    -3800, 

    300, 

    800, 

    -2300, 

    4200, 

    -1100, 

    3100, 

    3600, 

    400, 

    7000, 

    1500, 

    5700, 

    6100, 

    2800, 

    9200, 

    3400, 

    7400, 

    7500, 

    3900, 

    10000, 

    3900, 

    7500, 

    7400, 

    3400, 

    9200, 

    2800, 

    6100, 

    5700, 

    1500, 

    7000, 

    400, 

    3600, 

    3100, 

    -1100, 

    4200, 

    -2300, 

    800, 

    300, 

    -3800, 

    1600, 

    -4700, 

    -1400, 

    -1700, 

    -5700, 

    000, 

    -6200, 

    -2600, 

    -2600, 

    -6400, 

    -300, 

    -6300, 

    -2500, 

    -2300, 

    -5900, 

    300, 

    -5400, 

    -1400, 

    -1100, 

    -4600, 

    1600, 

    -4000, 

    -100, 

    100, 

    -3300, 

    2800, 

    -3000, 

    800, 

    900, 

    -2700, 

    3300, 

    -2700, 

    900, 

    800, 

    -3000, 

    2800, 

    -3300, 

    100, 

    -100, 

    -4000, 

    1600, 

    -4600, 

    -1100, 

    -1400, 

    -5400, 

    300, 

    -5900, 

    -2300, 

    -2500, 

    -6300, 

    -300, 

    -6400, 

    -2600, 

    -2600, 

    -6200, 

    000, 

    -5700, 

    -1700, 

    -1400, 

    -4700, 

    1600, 

    -3800, 

    300, 

    800, 

    -2300, 

    4200, 

    -1100, 

    3100, 

    3600, 

    400, 

    7000, 

    1500, 

    5700, 

    6100, 

    2800, 

    9200, 

    3400, 

    7400, 

    7500, 

    3900

} ; /* End of test values :  inpSignalROM[] */

/*
 * IIR lowpass 3 section( 5th order )elliptic filter
 * with 0.28 dB passband ripple and 40 dB stopband attenuation.
 * The cutoff frequency is 0.25 f( sample ).
 *
 */    

/*
 * Float version of the IIR lowpass filter coefficients : 
 *
 * float coefLowPass[] = {
 *     0.0552961603F, 
 *    -0.4363630712F,  0.0000000000F,  1.0000000000F,  0.0000000000F, 
 *    -0.5233039260F,  0.8604439497F,  0.7039934993F,  1.0000000000F, 
 *    -0.6965782046F,  0.4860509932F, -0.0103216320F,  1.0000000000F
 * } ; 
 *
 */    

varsize coefLowPass[] = 
{        

#if DATA_SIZE == 1

    /*
     * 16-bit integer version of the IIR lowpass filter coefficients : 
     *
     */    

    553, 

    -4364, 

    0, 

    10000, 

    0, 

    -5233, 

    8604, 

    7040, 

    10000, 

    -6966, 

    4861, 

    -103, 

    10000


#else

    /*
     * 8-bit integer version of the IIR lowpass filter coefficients : 
     *
     */    


    6, 

    -44, 

    0, 

    100, 

    0, 

    -52, 

    86, 

    70, 

    100, 

    -70, 

    49, 

    -1, 

    100


#endif

} ; 

FILTER_DEF iirLow1 = 
{        
    coefLowPass,    /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    3,              /* 3 sections */
} ; 

FILTER_DEF iirLow2 = 
{        
    coefLowPass,    /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    3,              /* 3 sections */
} ; 

FILTER_DEF iirLow3 = 
{        
    coefLowPass,    /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    3,              /* 3 sections */
} ; 

/*
 * IIR highpass 3 section( 6th order )chebyshev filter
 * with 1 dB passband ripple and cutoff frequency of 0.3 f( sample ).
 *
 */    

/*
 * Float version of the IIR high-pass filter coefficients : 
 *
 * float coefHiPass[] = {
 *     0.0025892381F, 
 *     0.5913599133F, 0.8879900575F, -2.0000000000F, 1.0000000000F, 
 *     0.9156184793F, 0.6796731949F, -2.0000000000F, 1.0000000000F, 
 *     1.3316441774F, 0.5193183422F, -2.0000000000F, 1.0000000000F
 * } ; 
 *
 */    

varsize coefHiPass[] = 
{        

#if DATA_SIZE == 1

    /*
     * 16-bit integer version of the IIR lowpass filter coefficients : 
     *
     */    


    26, 

    5914, 

    8880, 

    -20000, 

    10000, 

    9156, 

    6797, 

    -20000, 

    10000, 

    13316, 

    5193, 

    -20000, 

    10000


#else

    /*
     * 8-bit integer version of the IIR lowpass filter coefficients : 
     *
     */    


    1, 

    59, 

    89, 

    -200, 

    100, 

    92, 

    68, 

    -200, 

    100, 

    133, 

    52, 

    -200, 

    100

#endif

} ; 

FILTER_DEF iirHi1 = 
{        
    coefHiPass,     /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    3,              /* 3 sections */
} ; 

FILTER_DEF iirHi2 = 
{        
    coefHiPass,     /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    3,              /* 3 sections */
} ; 

FILTER_DEF iirHi3 = 
{        
    coefHiPass,     /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    3,              /* 3 sections */
} ; 

/*******************************************************************************
    Functions                                                                   
*******************************************************************************/

/*
*  Function :  GetTestData
*
*    Builds the table of input data which represents 'inpSignal', the analog
*    input data which we are filtering.  This is the "real world" data stream which
*    drives the algorithm.  This data is installed in RAM prior to execution of the
*    algorithm, only because it must be scaled dependent on filter parameters.
*
*/    

n_int
GetTestData( n_void )
{        
    int i ; 

    inpSignal = (varsize*)th_malloc( ( NUM_TESTS + 1 ) * sizeof( varsize ) ) ;

    if( inpSignal == NULL )
    {
        return false ; 
    }

    /*  Copy the test values from ROM to RAM  */        
    /*  ...and scale the data, if necessary */    

    for( i = 0 ; i < NUM_TESTS ; i++ )
    {
        inpSignal[i] = inpSignalROM[i] / DATA_SCALE ; 
    }

    return true ; 

}    /* End of function 'GetTestData' */

/*
*  Function :  GetInputValues
*
*    On each pass of the table lookup, a value must be input for 'signalIn'.
*    Each time this function is called, the next input value is
*    pulled from the table in RAM.  The table wraps around, so that input data is
*    continuous.     A flag is returned TRUE whenever the table wraps around.
*
*/    

n_int
GetInputValues( n_void )
{        
    signal_in = (varsize)inpSignal[tableCount] ; 

    /* If you run out of test data, then start table over */    

    if( ++tableCount < NUM_TESTS )
    {
        return false ; 
    }

    tableCount = 0 ; 
    return true ; 

}    /* End of function 'GetInputValues' */

