/*==============================================================================
 *$RCSfile: algo.h,v $
 *
 *   DESC : Table lookup and interpolation
 *
 * AUTHOR : dt
 *
 *  EEMBC : Automotive/Industrial Subcommittee 
 *
 *    CVS : $Revision: 1.2 $
 *          $Date: 2002/04/19 18:31:38 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/tblook01/algo.h,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: algo.h,v $
 * Revision 1.2  2002/04/19 18:31:38  rick
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


/*******************************************************************************
    Defines                                                                     
*******************************************************************************/

#define false            0
#define true            !false

#if (BMDEBUG && RAM_OUT == 1)	/* Debug buffer size == 32K */ 
#define MAX_FILESIZE    8192    /* Maximum size of output file */
#else
#define MAX_FILESIZE    256     /* Maximum size of output file */
#endif 
#define NUM_TESTS       232     /* Number of sets of input test data stimuli */
#define VAR_COUNT       2       /* Number of vars which must be allocated */
#define HEADER_SIZE     100     /* Approx size of title and header messages */

#define NUM_X_ENTRIES   50      /* Number of entries on each axis of the */
#define NUM_Y_ENTRIES   50      /* ...2D table */

/* Compilation switches to be defined( or not )based on application follow */

/* Define (=1) when compiling for DOUBLE variables */
#define DATA_SIZE   1       
/* ..or( =0 )when compiling for FLOAT variables */
#if DATA_SIZE == 0              /* SHORT variables */
typedef n_short varsize ;         /* Data and variables are 16 bits */
#else                           /* Else, LONG variables */
typedef n_long varsize ;          /* Data and variables are 32 bits */
#endif /* DATA_SIZE */

#define RAM_OUT 0   /* Define (=1)to direct debug text to RAM file */
                    /* or( =0 )to direct debug text to console */


/*******************************************************************************
    Global Variables                                                            
*******************************************************************************/

#ifndef ALGO_GLOBALS            /* Don't define these twice! */


extern varsize numXEntries ;       /* The table 'x' size spec'd here */
extern varsize numYEntries ;       /* The table 'y' size spec'd here */
extern n_int   tableCount ;        /* Number of passes through table */
extern varsize loadValue ;         /* 'Load' pulled from test data */
extern varsize speedValue ;        /* 'Speed' pulled from test data */
extern varsize *inpLoadValue ;     /* Array of 'load' test data */
extern varsize *inpSpeedValue ;    /* Array of 'speed' test data */

extern const varsize *engSpeed ;   /* Linear array of 'y' axis */
extern const varsize *engLoad ;    /* Linear array of 'x' axis */
extern const varsize *angleTable ; /* 2D array of f( x, y )= 'angle' */

#endif /* ALGO_GLOBALS */

/*******************************************************************************
    Function Prototypes                                                         
*******************************************************************************/

n_int GetTestData( n_void ) ; 
n_int GetInputValues( n_void ) ; 
n_void DebugOut( n_char * ) ; 
n_void WriteOut( varsize ) ; 

#endif /* __ALGO_H */
