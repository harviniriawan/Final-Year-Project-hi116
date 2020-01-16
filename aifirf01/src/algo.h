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
* File : algo.h
* Subcommittee : EEMBC Automotive/Industrial Subcommittee 
* Algorithm # 18 :  FIR( Finite Impulse Response )Filter - aifirf01
* Author : dt 
* Origin Date : 10/20/98
* Current Internal_Revision : 1.0
* Revision History : 
* 
*   5/14/99 pjt     1.0
*       Initial cleanup
*   7/4/99  arw     1.0
*       Added eembc_dt.h datatypes & additional cleanup
*       "thlib.h" will #include eembc_dt.h for us
*	7/13/99 arw     1.0 per Auto/Indust.Tech Posse decision
*		Changed MAX_FILESIZE to 4KB (4096 bytes)
*		Changed variable names starting with debugOut* to RAMfile*
*		Changed various declarations
* 
* 
* - END OF HEADER - 
*/ 

#ifndef __ALGO_H
#define __ALGO_H

/*******************************************************************************
    Includes (thlib.h provides eembc_dt.h for us)                                                                   
*******************************************************************************/

#include "thlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* PLATFORM-SPECIFIC DEFINITIONS  */    

/* Compilation switches to be defined( or not )based on application follow */

/* Define( =1 )when compiling for 32-bit variables */    
/* ...or( =0 )when compiling for 16-bit variables */    
#define DATA_SIZE 1 

#if DATA_SIZE == 1          /* LONG variables */
#define COEF_SCALE 13       /* Filter coefficients are scaled */
#define DATA_SCALE 1        /* ...and so is the input data */
#define THRESHOLD  1000     /* Threshold for binary output */
typedef n_long varsize ;      /* Data and variables are 32 bits */
#else                       /* Else, SHORT variables */
#define COEF_SCALE 11       /* Filter coefficients are scaled */
#define DATA_SCALE 50       /* ...and so is the input data */
#define THRESHOLD  100      /* Threshold for binary output */
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
#define NUM_TESTS    1000   /* Number of sets of input test data stimuli */
#define VAR_COUNT    2      /* Number of variables which must be allocated */
#define HEADER_SIZE  100    /* Approx. size of text title and header messages */

/* Moved here from algotst.c to eliminate th_mallocs in lite version */
#define	FIRLOW1_SECTIONS	35
#define FIRLOW2_SECTIONS	35
#define FIRLOW3_SECTIONS	35
#define FIRHI1_SECTIONS		35
#define FIRHI2_SECTIONS		35
#define FIRHI3_SECTIONS		35

#define HYSTERISIS (0.01*THRESHOLD) /* Hysterisis band around THRESHOLD */

/* FILTER DEFINITION STRUCTURE */    

typedef struct FILTER_DEF {        
    varsize *coef ;     /* Pointer to coefficients of filter */
    varsize *history ;  /* Pointer to history for filter */
    n_int sections ;      /* Number of filter sections */
} FILTER_DEF ; 

/*******************************************************************************
    Global Variables                                                            
*******************************************************************************/

#ifndef ALGO_GLOBALS         /* Don't define these twice! */

extern n_int   tableCount ;     /* Number of passes through table */
extern varsize *inpSignal ;  /* Pointer to array of input signal data values */
extern varsize signal_in ;   /* The input signal to be filtered */

#endif /* ALGO_GLOBALS */    

/*******************************************************************************
    Function Prototypes                                                         
*******************************************************************************/

n_int  GetTestData( n_void ) ; 
n_int  GetInputValues( n_void ) ; 
n_void DebugOut( n_char * ) ; 
n_void WriteOut( varsize  ) ; 

#endif /* __ALGO_H */
