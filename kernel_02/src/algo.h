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
* Algorithm # 13 :  Cache Buster - cacheb01
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
#include <math.h>

/* DECLARATIONS */

typedef void( *funcPtr )( void ) ;

/*  PLATFORM-SPECIFIC DEFINITIONS  */

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
#define RAM_OUT 1

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
#define NUM_TESTS    154   /* Number of chars of input test data stimuli */
#define VAR_COUNT    2     /* Number of variables which must be allocated */
#define HEADER_SIZE  100   /* Approx. size of text title and header messages */
#define ARRAY_SIZE   2048  /* Size of each source/destination array in RAM */
#define NUM_ARRAYS   16    /* Number of source/destination RAM arrays */

/*******************************************************************************
    Global Variables
*******************************************************************************/

#ifndef ALGO_GLOBALS    /* Don't define these twice! */

extern n_int   tableCount ;  /* Number of passes through table */
extern varsize *inputToken ; /* Pointer to array of input tokens */

extern varsize *array1 ;    /* Data source or destination in RAM */
extern varsize *array2 ;    /* Data source or destination in RAM */
extern varsize *array3 ;    /* Data source or destination in RAM */
extern varsize *array4 ;    /* Data source or destination in RAM */
extern varsize *array5 ;    /* Data source or destination in RAM */
extern varsize *array6 ;    /* Data source or destination in RAM */
extern varsize *array7 ;    /* Data source or destination in RAM */
extern varsize *array8 ;    /* Data source or destination in RAM */
extern varsize *array9 ;    /* Data source or destination in RAM */
extern varsize *array10 ;   /* Data source or destination in RAM */
extern varsize *array11 ;   /* Data source or destination in RAM */
extern varsize *array12 ;   /* Data source or destination in RAM */
extern varsize *array13 ;   /* Data source or destination in RAM */
extern varsize *array14 ;   /* Data source or destination in RAM */
extern varsize *array15 ;   /* Data source or destination in RAM */
extern varsize *array16 ;   /* Data source or destination in RAM */

extern varsize func ;       /* Index to selected function */

/* Indicies to source/destination arrays in RAM */
extern varsize arg1 ;
extern varsize arg2 ;
extern varsize arg3 ;
extern varsize arg4 ;
extern varsize arg5 ;
extern varsize arg6 ;

#endif /* ALGO_GLOBALS */

/*******************************************************************************
    Function Prototypes
*******************************************************************************/

n_int GetTestData( n_void ) ;
n_int GetInputValues( n_void ) ;
n_void DebugOut( n_char * ) ;
n_void WriteOut( varsize ) ;

n_void function1( n_void ) ;
n_void function2( n_void ) ;
n_void function3( n_void ) ;
n_void function4( n_void ) ;
n_void function5( n_void ) ;
n_void function6( n_void ) ;
n_void function7( n_void ) ;
n_void function8( n_void ) ;
n_void function9( n_void ) ;
n_void function10( n_void ) ;
n_void function11( n_void ) ;
n_void function12( n_void ) ;
n_void function13( n_void ) ;
n_void function14( n_void ) ;
n_void function15( n_void ) ;
n_void function16( n_void ) ;

#endif /* __ALGO_H */
