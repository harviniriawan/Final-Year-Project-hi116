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
* Algorithm #18 :  FIR( Finite Impulse Response )Filter - aifirf01
* Author : dt
* Origin Date : 10/20/198
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
* 35 point low-pass FIR filter cutoff at 0.2
*
* Float version of the FIR low-pass filter coefficients : 
*
* float coefLowPass[] = 
* {
* -0.006849167F,  0.001949014F,  0.01309874F,   0.01100677F, 
* -0.006661435F, -0.01321869F,   0.006819504F,  0.02292400F, 
*  0.000773216F, -0.03153488F,  -0.01384843F,   0.04054618F, 
*  0.03841148F,  -0.04790497F,  -0.08973017F,   0.05285565F, 
*  0.3126515F,    0.4454146F,    0.3126515F,    0.05285565F, 
* -0.08973017F,  -0.04790497F,   0.03841148F,   0.04054618F, 
* -0.01384843F,  -0.03153488F,   0.0007732160F, 0.02292400F, 
*  0.006819504F, -0.01321869F,  -0.006661435F,  0.01100677F, 
*  0.01309874F,   0.001949014F, -0.006849167F } ; 
*
*/    

varsize coefLowPass[] = 
{        

#if DATA_SIZE == 1

    /*
     * 16-bit integer version of the FIR lowpass filter coefficients : 
     *
     */    

    -69, 

    20, 

    131, 

    110, 

    -67, 

    -132, 

    68, 

    229, 

    8, 

    -315, 

    -139, 

    406, 

    384, 

    -479, 

    -897, 

    529, 

    3127, 

    4454, 

    3127, 

    529, 

    -897, 

    -479, 

    384, 

    406, 

    -139, 

    -315, 

    8, 

    229, 

    68, 

    -132, 

    -67, 

    110, 

    131, 

    20, 

    -69
#else
    /*
     * 8-bit integer version of the FIR lowpass filter coefficients : 
     *
     */    

    -4, 

    1, 

    7, 

    6, 

    -4, 

    -7, 

    4, 

    12, 

    1, 

    -16, 

    -7, 

    22, 

    19, 

    -24, 

    -45, 

    27, 

    157, 

    223, 

    157, 

    27, 

    -45, 

    -24, 

    19, 

    22, 

    -7, 

    -16, 

    1, 

    12, 

    4, 

    -7, 

    -4, 

    6, 

    7, 

    1, 

    -4

#endif

} ; 

FILTER_DEF firLow1 = 
{        
    coefLowPass,    /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    35,             /* 35 sections */
} ; 

FILTER_DEF firLow2 = 
{        
    coefLowPass,    /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    35,             /* 35 sections */
} ; 

FILTER_DEF firLow3 = 
{        
    coefLowPass,    /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    35,             /* 35 sections */
} ; 

/*
* 35 point highpass FIR filter cutoff at 0.3
*
* Float version of the FIR high-pass filter coefficients : 
*
* float coefHiPass[] = 
* {
*  6.849167e-003F, 1.949014e-003F, -1.309874e-002F, 1.100677e-002F, 
*  6.661435e-003F, -1.321869e-002F, -6.819504e-003F, 2.292400e-002F, 
* -7.732160e-004F, -3.153488e-002F, 1.384843e-002F, 4.054618e-002F, 
* -3.841148e-002F, -4.790497e-002F, 8.973017e-002F, 5.285565e-002F, 
* -3.126515e-001F, 4.454146e-001F, -3.126515e-001F, 5.285565e-002F, 
*  8.973017e-002F, -4.790497e-002F, -3.841148e-002F, 4.054618e-002F, 
*  1.384843e-002F, -3.153488e-002F, -7.732160e-004F, 2.292400e-002F, 
* -6.819504e-003F, -1.321869e-002F, 6.661435e-003F, 1.100677e-002F, 
* -1.309874e-002F, 1.949014e-003F, 6.849167e-003F } ; 
*
*/    

varsize coefHiPass[] = 
{        
#if DATA_SIZE == 1
    /*
     * 16-bit integer version of the FIR lowpass filter coefficients : 
     *
     */    

    69, 

    20, 

    -131, 

    110, 

    67, 

    -132, 

    -68, 

    229, 

    -8, 

    -315, 

    139, 

    406, 

    -384, 

    -479, 

    897, 

    529, 

    -3127, 

    4454, 

    -3127, 

    529, 

    897, 

    -479, 

    -384, 

    406, 

    139, 

    -315, 

    -8, 

    229, 

    -68, 

    -132, 

    67, 

    110, 

    -131, 

    20, 

    69
#else
    /*
     * 8-bit integer version of the FIR lowpass filter coefficients : 
     *
     */    

    -4, 

    1, 

    7, 

    6, 

    -4, 

    -7, 

    4, 

    12, 

    1, 

    -16, 

    -7, 

    22, 

    19, 

    -24, 

    -45, 

    27, 

    157, 

    223, 

    157, 

    27, 

    -45, 

    -24, 

    19, 

    22, 

    -7, 

    -16, 

    1, 

    12, 

    4, 

    -7, 

    -4, 

    6, 

    7, 

    1, 

    -4
#endif

} ; 

FILTER_DEF firHi1 = 
{        
    coefHiPass,     /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    35,             /* 35 sections */

} ; 

FILTER_DEF firHi2 = 
{        
    coefHiPass,     /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    35,             /* 35 sections */
} ; 

FILTER_DEF firHi3 = 
{        
    coefHiPass,     /* Points to filter coefficients */
    NULL,           /* Placeholder for history pointer */
    35,             /* 35 sections */
} ; 

/*******************************************************************************
    Functions                                                                   
*******************************************************************************/

/*
*  Function :  GetTestData
*
*  Builds the table of input data which represents 'inpSignal', the analog
*  input data which we are filtering. This is the "real world" data stream which
*  drives the algorithm. This data is installed in RAM prior to execution of the
*  algorithm, only because it must be scaled dependent on filter parameters.
*
*/    

n_int
GetTestData( n_void )
{        
    n_int i ; 

    inpSignal = (varsize*)
        th_malloc( ( NUM_TESTS + 1 )*sizeof( varsize ) ) ; 

    if( inpSignal == NULL )
    {
        return false ; 
    }

    /* Copy the test values from ROM to RAM and scale the data, if necessary */    
    for( i = 0 ; i < NUM_TESTS ; i++ )
    {
        inpSignal[i] = inpSignalROM[i] / DATA_SCALE ; 
    }

    return true ; 

} /* End of function 'GetTestData' */

/*
*  Function :  GetInputValues
*
*  On each pass of the table lookup, a value must be input for 'signalIn'.
*  Each time this function is called, the next input value is
*  pulled from the table in RAM.  The table wraps around, so that input data is
*  continuous.     A flag is returned TRUE whenever the table wraps around.
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

} /* End of function 'GetInputValues' */
