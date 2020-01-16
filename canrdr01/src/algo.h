/*==============================================================================
 *$RCSfile: algo.h,v $
 *
 *   DESC : Header file for CAN reader algorithm
 *
 * AUTHOR : 
 *
 *  EEMBC : Automotive/Industrial Subcommittee 
 *
 *    CVS : $Revision: 1.2 $
 *          $Date: 2002/07/19 23:10:23 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/canrdr01/algo.h,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: algo.h,v $
 * Revision 1.2  2002/07/19 23:10:23  rick
 * Fix iteration dependant NI CRC's
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

/*  PLATFORM-SPECIFIC DEFINITIONS  */    

/* Compilation switches to be defined( or not )based on application follow */

/* Define (=1) when compiling for DOUBLE variables */
#define DATA_SIZE   1       
/* ..or( =0 )when compiling for FLOAT variables */
#if DATA_SIZE == 0           /* SHORT variables */
typedef n_short varsize ;      /* Data and variables are 16 bits */
#define MAX_VARIABLE 0x7FFF  /* Must match sim. real-time ctr in test data */
#else                        /* Else, LONG variables */
typedef n_long varsize ;       /* Data and variables are 32 bits */
#define MAX_VARIABLE 0x7FFF  /* Must match sim. real-time ctr in test data */
#endif /* DATA_SIZE */

#define RAM_OUT 0   /* Define (=1)to direct debug text to RAM file */
                    /* or( =0 )to direct debug text to console */

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

#define NUM_TESTS       1494    /* Number of sets of input test data stimuli */
#define VAR_COUNT       1       /* Number of vars which must be allocated */
#define HEADER_SIZE     100     /* Approx size of title and header messages */

#define ENG_TEMPERATURE 1000    /* CAN message ID for Engine Temperature */
#define LOAD            999     /* CAN message ID for Engine Load */
#define ENG_SPEED       998     /* CAN message ID for Engine Speed( RPM )*/
#define BATTERY_VOLTS   1001    /* CAN message ID for Battery Voltage */
#define ACCELERATOR     1100    /* CAN message ID for Accelerator Position */
#define ROAD_SPEED      1200    /* CAN message ID for Road Speed */

#define ENG_TEMP_LENGTH     1   /* CAN message length */
#define LOAD_LENGTH         1   /* CAN message length */
#define ENG_SPEED_LENGTH    1   /* CAN message length */
#define BATTERY_LENGTH      1   /* CAN message length */

#define LOOKING     0           /* CAN message processing state */
#define GOT_ID      1           /* CAN message processing state */
#define GOT_EXTID   2           /* CAN message processing state */
#define GOT_IDE     3           /* CAN message processing state */
#define GOT_RTR     4           /* CAN message processing state */
#define GET_DATA    5           /* CAN message processing state */
#define GOT_ERROR   6           /* CAN message processing state */

typedef struct CAN_MESSAGE      /* This is a CAN message block */
{            
    Word address ; 
    Dword ext_address ; 
    n_int ide ; 
    n_int rtr ; 
    n_int length ; 
    n_char data1 ; 
    n_char data2 ; 
    n_char data3 ; 
    n_char data4 ; 
    n_char data5 ; 
    n_char data6 ; 
    n_char data7 ; 
    n_char data8 ; 
} CAN_MESSAGE ; 

/*******************************************************************************
    Global Variables                                                            
*******************************************************************************/

#ifndef ALGO_GLOBALS            /* Don't define these twice! */

extern n_int   tableCount ;     /* Number of passes through table */
extern varsize inputData ;      /* Input data from the CAN message stream */
extern varsize *inpVariable ;   /* Pointer to array of input data values */
extern varsize engineTemp ;     /* Phony engine data to xmit */
extern varsize engineLoad ;     /* Phony engine data to xmit */
extern varsize engineSpeed ;    /* Phony engine data to xmit */
extern varsize batteryVolts ;   /* Phony engine data to xmit */

#endif /* ALGO_GLOBALS */    

/*******************************************************************************
    Function Prototypes                                                         
*******************************************************************************/

int  GetTestData( n_void ) ; 
int  GetInputValues( n_void ) ; 
void DebugOut( n_char * ) ; 
void WriteOut( varsize  ) ; 

#endif /* __ALGO_H */
