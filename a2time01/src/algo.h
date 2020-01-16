/*==============================================================================
 *$RCSfile: algo.h,v $
 *
 *   DESC : Angle-to-Time Conversion - a2time00
 *
 * AUTHOR : dt
 *
 *  EEMBC : Automotive Subcommittee 
 *
 *    CVS : $Revision: 1.2 $
 *          $Date: 2002/04/19 18:31:37 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/a2time01/algo.h,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: algo.h,v $
 * Revision 1.2  2002/04/19 18:31:37  rick
 * Bug #146: global tablecount uninitialized
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
    Includes (thlib includes eembc_dt.h)                                                                     
*******************************************************************************/

#include "thlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Compilation switches to be defined( or not )based on application follow */

/* Define (=1) when compiling for DOUBLE variables */
#define DATA_SIZE   1       
/* ..or( =0 )when compiling for FLOAT variables */
#if DATA_SIZE == 0           /* SHORT variables */
typedef n_short varsize ;    /* Data and variables are 16 bits */
#define MAX_VARIABLE 0x7FFF  /* Must match sim. real-time ctr in test data */
#else                        /* Else, LONG variables */
typedef n_long varsize;      /* Data and variables are 32 bits */
#define MAX_VARIABLE 0x7FFF  /* Must match sim. real-time ctr in test data */
#endif /* DATA_SIZE */

#define RAM_OUT 0   /* Define (=1)to direct debug text to RAM file */
                    /* or( =0 )to direct debug text to console */

/*******************************************************************************
    Defines                                                                     
*******************************************************************************/

#define false           0
#define true            !false

#if (BMDEBUG && RAM_OUT == 1)	/* Debug buffer size == 32K */ 
#define MAX_FILESIZE    8192    /* Maximum size of output file */
#else
#define MAX_FILESIZE    256     /* Maximum size of output file */
#endif 
#define NUM_TESTS       500     /* Number of sets of input test data stimuli */
#define VAR_COUNT       1       /* Number of variables which must be allocated */
#define HEADER_SIZE     100     /* Approx size of text title and header messages */

#define CYLINDERS       8       /* We're simulating an eight cylinder engine */
#define TENTH_DEGREES   3600    /* Number of 1/10-degrees in a circle */

/* Cylinder #1 firing angle (*10) */
#define FIRE1_ANGLE (TENTH_DEGREES/CYLINDERS*1)
/* Cylinder #2 firing angle (*10) */
#define FIRE2_ANGLE (TENTH_DEGREES/CYLINDERS*2)
/* Cylinder #3 firing angle (*10) */
#define FIRE3_ANGLE (TENTH_DEGREES/CYLINDERS*3)   
/* Cylinder #4 firing angle (*10) */
#define FIRE4_ANGLE (TENTH_DEGREES/CYLINDERS*4)  
/* Cylinder #5 firing angle (*10) */
#define FIRE5_ANGLE (TENTH_DEGREES/CYLINDERS*5)   
/* Cylinder #6 firing angle (*10) */
#define FIRE6_ANGLE (TENTH_DEGREES/CYLINDERS*6)   
/* Cylinder #7 firing angle (*10) */
#define FIRE7_ANGLE (TENTH_DEGREES/CYLINDERS*7)   
/* Cylinder #8 firing angle (*10) */
#define FIRE8_ANGLE (TENTH_DEGREES/CYLINDERS*8)   

#define CYL1            1       /* Cylinder #1 firing window */
#define CYL2            2       /* Cylinder #2 firing window */
#define CYL3            3       /* Cylinder #3 firing window */
#define CYL4            4       /* Cylinder #4 firing window */
#define CYL5            5       /* Cylinder #5 firing window */
#define CYL6            6       /* Cylinder #6 firing window */
#define CYL7            7       /* Cylinder #7 firing window */
#define CYL8            8       /* Cylinder #8 firing window */

#define TDC_TEETH       2       /* Number of missing teeth (=1) at TDC */
#define TDC_MARGIN      0.9     /* Discrimination window for TDC teeth */
#define NUM_TEETH       60      /* Number of teeth on tonewheel */

/* Arbitrary scale factor for computing internal RPM */
#define RPM_SCALE_FACTOR        3600000 

/*******************************************************************************
    Global Variables                                                            
*******************************************************************************/

#ifndef ALGO_GLOBALS            /* Don't define these twice! */

extern n_int   tableCount ;     /* Number of passes through table */
extern varsize angleCounter ;   /* Current 'angleCounter' pulled from  data */
extern varsize *inpAngleCount ; /* Array of 'angleCounter' test data values */
extern varsize tonewheelTeeth ; /* Number of teeth on the tonewheel */
/*extern n_int isTopDeadCenter ;  * TRUE/FALSE flag when TDC occurs */

#endif /* ALGO_GLOBALS */       

/*******************************************************************************
    Function Prototypes                                                         
*******************************************************************************/

n_int  GetTestData( n_void ) ;
n_int  GetInputValues( n_void ) ;
n_void DebugOut( n_char * ) ;
n_void WriteOut( varsize ) ;

#endif /* __ALGO_H */
