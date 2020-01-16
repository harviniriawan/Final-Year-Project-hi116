/*==============================================================================
 *$RCSfile: algo.h,v $
 *
 *   DESC : Tooth to Spark
 *
 * AUTHOR : dt
 *
 *  EEMBC : Automotive/Industrial Subcommittee 
 *
 *    CVS : $Revision: 1.2 $
 *          $Date: 2002/04/19 18:31:38 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/ttsprk01/algo.h,v $
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
    Includes (thlib.h will provide eembc_dt.h for us)                                                                   
*******************************************************************************/

#include "thlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
#define NUM_TESTS    200   /* Number of sets of input test data stimuli */
#define VAR_COUNT    9     /* Number of variables which must be allocated */
#define HEADER_SIZE  100   /* Approximate size of title and header messages */

#define NUM_X_ENTRIES 50 /* Number of entries on each axis of the */
#define NUM_Y_ENTRIES 50 /*  2D table */

#define CYLINDERS       4    /* Simulating a 4-cylinder, 4-stroke engine */
#define FIRE_TIME_NOM   1000 /* Nominal firing time for ignition, ticks BTDC */
#define DWELL_MIN       360  /* Minimum dwell time, in ECU ticks */
#define INJECT_TIME_NOM 3000 /* Nominal injector firing time,
                              * in ECU ticks BTDC */
#define INJECT_MIN      100  /* Minimum injector duration, in ECU ticks */
#define INJECT_MAX      2800 /* Maximum injector duration, in ECU ticks */
#define ENG_SPEED_MAX   8000 /* Maximum engine speed limit */
#define KNOCK_MAX       200  /* Maximum retardation for knocking */
#define KNOCK_STEP      20   /* Step size to retard ignition for knocking */
#define ACCEL_MIN       5    /* Acceleration defined as minimum change 
                              * in throttle */
#define ACCEL_TIME      3    /* # of ECU times over which compensation
                              * is reduced( 2**ACCEL_TIME )*/

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
    Global Variables                                                            
*******************************************************************************/

#ifndef ALGO_GLOBALS    /* Don't define these twice! */

extern n_int   tableCount ;     /* Number of passes through table */
/* INPUT VARIABLES : presented in the test data */    

extern varsize tonewheelTeeth ; /* Number of teeth on the tonewheel */
extern varsize engSpeed ;       /* Engine RPM's expressed in ECU units */
extern varsize angleTime ;      /* Crankshaft angle expressed at ECU time */
extern varsize airIntake ;      /* Intake-air volume(measure of engine load)*/
extern varsize airTemp ;        /* Intake-air temperature */
extern varsize engTemp ;        /* Engine coolant temperature */
extern varsize battery ;        /* Vehicle battery voltage */
extern varsize throttle ;       /* Throttle position */
extern varsize lastThrottle ;   /* Previous throttle position */
extern n_int knocked ;            /* Engine knock has been detected */
extern n_int cranking ;           /* We're starting the engine */
/* NOT PART OF BENCHMARK to free malloc */
extern varsize *engSpeedValue ;  /* Test data input variable table for 'engSpeed' */
extern varsize *angleTimeValue ; /* Test data input variable table for 'angleTime' */
extern varsize *airIntakeValue ; /* Test data input variable table for 'airIntake' */
extern varsize *airTempValue ;   /* Test data input variable table for 'airTemp' */
extern varsize *engTempValue ;   /* Test data input variable table for 'engTemp' */
extern varsize *batteryValue ;   /* Test data input variable table for 'battery' */
extern varsize *throttleValue ;  /* Test data input variable table for 'throttle' */
extern n_int *knockedValue ;       /* Test data input variable table for 'knocked' */
extern n_int *crankingValue ;      /* Test data input variable table for 'cranking' */
/*  */
extern const varsize *engSpeedAxis ;    /* Linear array of 'y' axis */
extern const varsize *airIntakeAxis ;   /* Linear array of 'x' axis */
extern const varsize *batteryAxis ;     /* Linear array of 'x' axis */
extern const varsize *lambdaTbl ;       /* Y array of f( x, y )= 'lambda' */
extern const varsize *warmupTbl ;       /* Y array of f( x, y )= 'warmup' */
extern const varsize *advanceTbl ;      /* Y array of f( x, y )= 'advance' */
extern const varsize *dwellTbl ;        /* Y array of f( x, y )= 'dwell' */
extern const varsize *voltAxis ;        /* x axis for 'voltTbl' */
extern const varsize *voltTbl ;         /* y axis, injector compensation, 
                                         * f( battery )
                                         */
extern const varsize *aTempAxis ;       /* x axis for 'airTempTbl' */
extern const varsize *airTempTbl ;      /* y axis, injector compensation, 
                                         *  f( airTemp )
                                         */
extern const varsize *eTempAxis ;       /* x axis for 'engWarmTbl' and 
                                         *  'engTempTbl' 
                                         */
extern const varsize *engWarmTbl ;      /* y axis, injector compensation, 
                                         *  f( engTemp )
                                         */
extern const varsize *engTempTbl ;      /* y axis, ignition advnce compensation,
                                         *  f( engTemp )
                                         */ 
extern const varsize *accelTbl ;        /* y axis, injector compensation,
                                         *  f( engTemp )
                                         */

extern n_long voltEntries ;    /* Number of entries in each table */
extern n_long eTempEntries ; 
extern n_long aTempEntries ; 
extern n_long loadEntries ; 
extern n_long speedEntries ; 
extern n_long battEntries ; 

#endif /* ALGO_GLOBALS */    

/*******************************************************************************
    Function Prototypes                                                         
*******************************************************************************/

n_int GetTestData( n_void ) ; 
n_int GetInputValues( n_void ) ; 
n_int GetZTables( n_void ) ; 
n_int GetYTables( n_void ) ; 
n_void DebugOut( n_char * ) ; 
n_void WriteOut( varsize ) ; 

#endif /* __ALGO_H */
