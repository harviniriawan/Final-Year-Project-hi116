/*==============================================================================
 *$RCSfile: algo.h,v $
 *
 *   DESC : Fast Fourier Transform
 *
 * AUTHOR : dt
 *
 *  EEMBC : Automotive/Industrial Subcommittee 
 *
 *    CVS : $Revision: 1.2 $
 *          $Date: 2002/04/25 20:10:43 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/aifftr01/algo.h,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: algo.h,v $
 * Revision 1.2  2002/04/25 20:10:43  rick
 * sprintf to th_sprintf
 *
 *
 *------------------------------------------------------------------------------
 * Copyright (c) 1998-2002 by the EDN Embedded Microprocessor 
 * Benchmark Consortium (EEMBC), Inc. 
 * 
 * All Rights Reserved. This is licensed program product and 
 * is owned by EEMBC. The Licensee understands and agrees that the 
 * Benchmarks licensed by EEMBC hereunder (including methods or concepts 
 * utilized therein) contain certain information that is confidential 
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
 * ATTRIBUTES, WHETHER EXPRESS OR IMPLIED (IN LAW OR IN FACT), ORAL OR 
 * WRITTEN. 
 * 
 * Licensee hereby agrees by accessing this source code that all benchmark 
 * scores related to this code must be certified by ECL prior to publication 
 * in any media, form, distribution, or other means of conveyance of 
 * information subject to the terms of the EEMBC Member License Agreement 
 * and/or EEMBC Licensee Agreement. 
 * 
 * Other Copyright Notice (if any): 
 * 
 * For conditions of distribution and use, see the accompanying README file.
 * ===========================================================================*/

#ifndef __ALGO_H
#define __ALGO_H

/*******************************************************************************
    Includes (thlib.h provides eembc_dt.h for us)                                                                   
*******************************************************************************/

#include "thlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* PLATFORM-SPECIFIC DEFINITIONS  */    

/* Compilation switches to be defined( or not )based on application follow */

/* Define( =1 )when compiling for 32-bit variables */    

/* ...or( =0 )when compiling for 16-bit variables */    

#define DATA_SIZE 1 

#if DATA_SIZE == 1          /* LONG variables */
typedef n_long varsize ;      /* Data and variables are 32 bits */
#else                       /* Else, SHORT variables */
typedef n_short varsize ;     /* Data and variables are 16 bits */
#endif /* DATA_SIZE */

/* Define( =1 )to direct debug text to RAM file */    

/* ...or( =0 )to direct debug text to console */    

#define RAM_OUT 0        

/*******************************************************************************
    Defines                                                                     
*******************************************************************************/

#define false    0
#define true    !false

#if (BMDEBUG && RAM_OUT == 1)	/* Debug buffer size == 32K */ 
#define MAX_FILESIZE    8192    /* Maximum size of output file */
#else
#define MAX_FILESIZE    256     /* Maximum size of output file */
#endif 
#define NUM_TESTS    4096  /* Number of sets of input test data stimuli */
#define VAR_COUNT    3     /* Number of variables which must be allocated */
#define HEADER_SIZE  100   /* Approximate size of title and header messages */

#define NUM_POINTS   512   /* Number of points in FFT */
#define FFT_LENGTH   9     /* length = log2( NUM_POINTS )*/

#ifdef PI
#undef PI
#endif
#define PI 3.141592654 

/* TRIG_SCALE_FACTOR is used for greatest twiddle factor precision */    
#define TRIG_SCALE_FACTOR       32768

/* BUTTERFLY_SCALE_FACTOR = log2( TRIG_SCALE_FACTOR )is used to
accomodate accumulator size limit of 32 bits */    
#define BUTTERFLY_SCALE_FACTOR  15          /* Divide by 32768 */

/* STAGE_SCALE_FACTOR is used to prevent overflow from
accumulation at each processing stage */    
#define STAGE_SCALE_FACTOR      1           /* Divide by 2 */

/*******************************************************************************
    Global Variables                                                            
*******************************************************************************/

#ifndef ALGO_GLOBALS         /* Don't define these twice! */

extern n_int   tableCount ;  /* Number of passes through table */
extern varsize *inpSignal ;  /* Pointer to array of input signal data values */

#endif /* ALGO_GLOBALS */    

/*******************************************************************************
    Function Prototypes                                                         
*******************************************************************************/

n_int  GetTestData( n_void ) ; 
n_int  GetInputValues( varsize *, varsize * ) ; 
n_void DebugOut( n_char * ) ; 
n_void WriteOut( varsize  ) ; 

#endif /* __ALGO_H */
