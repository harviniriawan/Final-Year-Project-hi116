/*==============================================================================
 *$RCSfile: bmark_lite.c,v $
 *
 *   DESC : This file contains the Test Main and other TH support functions
 *
 * AUTHOR : ARM, Ltd., Rick Foos, ECL, LLC
 *
 *  EEMBC : Automotive Subcommittee 
 *
 *    CVS : $Revision: 1.14 $
 *          $Date: 2002/08/07 22:21:49 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/ttsprk01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.14  2002/08/07 22:21:49  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.13  2002/07/22 21:59:37  rick
 * General cleanup Beta 2b
 *
 * Revision 1.12  2002/07/22 16:10:19  rick
 * Fix iteration dependant NI CRC
 *
 * Revision 1.11  2002/07/11 22:13:34  rick
 * Initialize tcdef results
 *
 * Revision 1.10  2002/07/10 19:01:30  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.9  2002/05/29 22:25:38  rick
 * Set recommended iterations with make
 *
 * Revision 1.8  2002/05/10 17:20:37  rick
 * Add al_main to API
 *
 * Revision 1.7  2002/04/25 20:10:46  rick
 * sprintf to th_sprintf
 *
 * Revision 1.6  2002/04/19 18:31:39  rick
 * Bug #146: global tablecount uninitialized
 *
 * Revision 1.5  2002/03/12 18:31:05  rick
 * ITERATIONS, CRC_CHECK, NON_INTRUSIVE_CRC_CHECK, add standards headers
 *
 * Revision 1.4  2002/02/25 17:15:34  rick
 * Add comment blocks, fix atime th_report call.
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

/*******************************************************************************
    Includes                                                                    
*******************************************************************************/
#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "xipipsu.h"
#include "xipipsu_hw.h"
#include "xil_cache.h"

#define ALGO_GLOBALS    1   /* Next time, we'll skip these */
#include "algo.h"

/* Estimate of allocation for NUM_TESTS*( debug test + 2 variables )*/
#define T_BSIZE (MAX_FILESIZE+((NUM_TESTS+1)*VAR_COUNT*4))

/* IPI device ID to use for this test */
#define TEST_CHANNEL_ID XPAR_XIPIPSU_0_DEVICE_ID
/* Test message length in words. Max is 8 words (32 bytes) */
#define TEST_MSG_LEN    4
/* Interrupt Controller device ID */
#define INTC_DEVICE_ID  XPAR_SCUGIC_0_DEVICE_ID
/* Time out parameter while polling for response */
#define TIMEOUT_COUNT 10000
/* Uncomment this define when running test with ECC */
/*#define NO_ECC*/
/* Uncomment this define to listen for injection from R5_1 */
#define LISTEN_A53

XScuGic GicInst;
XIpiPsu IpiInst;

/* Buffers to store message from the other core */
static u32 __attribute__((section(".ocm"))) TmpBufPtr[TEST_MSG_LEN] = {0};

#include "thlib.h"

/* ======================================================================== */
/*         F U N C T I O N   P R O T O T Y P E S                            */
/* ======================================================================== */
int main(int argc, const char* argv[] );
int t_run_test(struct TCDef *tcdef, int argc, const char* argv[]);
/*Additions to do IPI & Error Injection*/
void IpiIntrHandler(void *XIpiPsuPtr);
static XStatus SetupInterruptSystem(XScuGic *IntcInstancePtr,XIpiPsu *IpiInstancePtr, u32 IpiIntrId);
/* TCM Corruption function */
void corrupt_tcm(u32 addr,u32 mask,u32 shift);

/* Define iterations */
#if !defined(ITERATIONS) || CRC_CHECK || ITERATIONS==DEFAULT
#undef ITERATIONS
#if CRC_CHECK
#define ITERATIONS 10000	/* required iterations for crc */
#else
#define ITERATIONS 10000	/* recommended iterations for benchmark */
#endif
#endif

/* Pre-caclculated CRC values */
#if	CRC_CHECK
#define EXPECTED_CRC 0x6c6c
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC 0x00B1
#else
#define EXPECTED_CRC 0x0000
#endif

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/

TCDef the_tcdef = 
{
    "AUT ttsprk01   ",
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Algorithm #5 -- Tooth-To-Spark V1.0H0 - ttsprk01", 
    TCDEF_REVISION, 
    { EEMBC_TH_MAJOR, EEMBC_TH_MINOR, EEMBC_TH_STEP, EEMBC_TH_REVISION }, 
    { 0, 0, 0, 0 },   /* Target Hardware Version Number Required( make all zeros to ignore )*/
    { 1, 0, 'H', 0 }, /* The Version number of this Benchmark */
	ITERATIONS,    /* recomended iterations for benchmark score */
	0,
	0,
	0,
	0,
	0,
	0,
	0
} ; 

varsize ZTableLookup( varsize, varsize, n_long, n_long, const varsize *, 
                      const varsize *, const varsize * ) ; 
varsize YTableLookup( varsize, n_long, const varsize *, const varsize * ) ; 

extern const varsize inpValueROM[] ;    /* Source of the test data, in "ROM" */
extern const varsize airIntakeROM[] ;   /* The tables are in "ROM" */
extern const varsize engSpeedROM[] ; 
extern const varsize lambdaROM[] ; 
extern const varsize warmupROM[] ; 
extern const varsize advanceROM[] ; 
extern const varsize batteryROM[] ; 
extern const varsize dwellROM[] ; 
extern const varsize eTempAxisROM[] ; 
extern const varsize engWarmROM[] ; 
extern const varsize engTempROM[] ; 
extern const varsize aTempAxisROM[] ; 
extern const varsize airTempROM[] ; 
extern const varsize voltAxisROM[] ; 
extern const varsize voltROM[] ; 
extern const varsize accelROM[] ; 

n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree; /* NOT PART OF BENCHMARK to free RAMfile */
/* NOT PART OF BENCHMARK to free malloc */
varsize *engSpeedValue ;  /* Test data input variable table for 'engSpeed' */
varsize *angleTimeValue ; /* Test data input variable table for 'angleTime' */
varsize *airIntakeValue ; /* Test data input variable table for 'airIntake' */
varsize *airTempValue ;   /* Test data input variable table for 'airTemp' */
varsize *engTempValue ;   /* Test data input variable table for 'engTemp' */
varsize *batteryValue ;   /* Test data input variable table for 'battery' */
varsize *throttleValue ;  /* Test data input variable table for 'throttle' */
n_int *knockedValue ;       /* Test data input variable table for 'knocked' */
n_int *crankingValue ;      /* Test data input variable table for 'cranking' */
/*   */

/* INPUT VARIABLES : presented in the test data */    

varsize tonewheelTeeth ;    /* Number of teeth on the tonewheel */
varsize engSpeed ;          /* Engine speed( RPM's )expressed in ECU units */
varsize angleTime ;         /* Crankshaft angle expressed at ECU time */
varsize airIntake ;         /* Intake-air volume( measure of engine load )*/
varsize airTemp ;           /* Intake-air temperature */
varsize engTemp ;           /* Engine coolant temperature */
varsize battery ;           /* Vehicle battery voltage */
varsize throttle ;          /* Throttle position */
varsize lastThrottle ;      /* Previous throttle position */
n_int knocked ;               /* Engine knock has been detected */
n_int cranking ;              /* We're starting the engine */

const varsize *engSpeedAxis ;  /* Linear array of 'y' axis */
const varsize *airIntakeAxis ; /* Linear array of 'x' axis */
const varsize *batteryAxis ;   /* Linear array of 'x' axis */
const varsize *lambdaTbl ;     /* Y array of f( x, y )= 'lambda' */
const varsize *warmupTbl ;     /* Y array of f( x, y )= 'warmup' */
const varsize *advanceTbl ;    /* Y array of f( x, y )= 'advance' */
const varsize *dwellTbl ;      /* Y array of f( x, y )= 'dwell' */
const varsize *voltAxis ;      /* 'x' axis for 'voltTbl' */
const varsize *voltTbl ;       /* 'y' axis, injector compensation */
const varsize *aTempAxis ;     /* 'x' axis for 'airTempTbl' */
const varsize *airTempTbl ;    /* 'y' axis, injector compensation */
const varsize *eTempAxis ;     /* 'x' axis for 'engWarmTbl' and 'engTempTbl' */
const varsize *engWarmTbl ;    /* 'y' axis, injector compensation */
const varsize *engTempTbl ;    /* 'y' axis, ignition advance compensation */
const varsize *accelTbl ;      /* 'y' axis, injector compensation */

n_long voltEntries ;             /* Number of entries in each table */
n_long eTempEntries ; 
n_long aTempEntries ; 
n_long loadEntries ; 
n_long speedEntries ; 
n_long battEntries ; 

/* >> IMPORTANT NOTE << 
*
* Since benchmarks can be entered( run )multiple times, the benchmark
* MUST NOT depend on global data being initialized.  E.g. it must
* complelty initialize itself EVERY TIME its t_run_test()function
* is called.
* 
*/    


/*********************************************************************************
* FUNC	: int t_run_test( struct TCDef *tcdef,int argc, const char *argv[] )
* DESC  : This is the functions that carries out the algorithm. This function
*         is called from the main.  Same algorithm is called three times. Input
*         data for the algorithm is stored in 'algotst'c'.  All variables declared
*         and initialized in 'init.c'.  When 'BMDEBUG' and 'WINDOWS_EXAMPLE_CODE' 
*         defined in 'thcfg.h' it will write bunch of debug message in the RAM.  
*         All debug routines are in 'debug.c'. It calculates CRC using intermediate
*         values from the algorithms.  CRC is used to check if algorithm carried out
*         correctly.
* ARGUMENT: arc - not used
*           argv - not used
*           tcdef - structure defined in 'thlib.h'.  Following members of tcdef
*     			rec_iterations- recommend number of times algorithm should be carried out
*   			iterations  - actual number of times algorithm was carried
*                             most cases same as 'rec_iterations'
*               duration	- Total time to carry out the algorithm
*               CRC			- calculated CRC
*
* RETURNS : Success if the the CRC matches. 
*****************************************************************************************/    
int t_run_test( struct TCDef *tcdef,int argc, const char *argv[] )
{    
    size_t		loop_cnt = tcdef->rec_iterations;
#if BMDEBUG
    char *szTitle = 
        "\nEEMBC Automotive/Industrial Subcommittee Benchmarks ( c )1998-1999\n"
        "Algorithm 5 :  Tooth-to-Spark  Rev. 1.0H0 - ttsprk01\n" ; 
    char *szHeader = 
        "\neSpeed, angle, airIntake, airTemp, engTemp, battery, "
        "throttle, knock, crank\n[dwell, firingT, inject, advance, retard, "
        "accComp, lambda, injectOn, Pump, Ignition] counter\n" ; 
    char szDebug[100] ; 
#else
    varsize i ; 
#endif /* BMDEBUG */
    static n_char szDataSize[40] ; 
    n_int isTableLooped = false ;    /* Input test data table looped */
    /* Hardware enable for the fuel pump */
    static n_int fuelPump1 ;
    static n_int fuelPump2 ;
    static n_int fuelPump3 ;    
    /* Hardware enable for the igniters */
    static n_int enbIgnition1 ;
    static n_int enbIgnition2 ;
    static n_int enbIgnition3 ; 
    /* The injector( s )is/are on to deliver fuel */
    static n_int injectOn1 ; 
    static n_int injectOn2 ;
    static n_int injectOn3 ;    
    /* History of the 'airIntake' variable */
    static varsize lastAirIntake1 ;
    static varsize lastAirIntake2 ;
    static varsize lastAirIntake3 ;   
    /* History of the 'engSpeed' variable */
    static varsize lastEngSpeed1 ;
    static varsize lastEngSpeed2 ;
    static varsize lastEngSpeed3 ;    
    /* History of the 'throttle' variable */
    static varsize lastThrottle1 ;
    static varsize lastThrottle2 ;
    static varsize lastThrottle3 ;    
    /* Injector duration to control fuel mix */
    static varsize inject1 ;
    static varsize inject2 ;
    static varsize inject3 ;    
    /* Time( angle )to turn on the injector */
    static varsize injectTime1 ;
    static varsize injectTime2 ;
    static varsize injectTime3 ;
    /* Injector compensation for load/speed */
    static varsize lambda1 ;
    static varsize lambda2 ;
    static varsize lambda3 ;    
    /* Injector compensation for battery volts */
    static varsize voltComp1 ;
    static varsize voltComp2 ;
    static varsize voltComp3 ;  
    /* Injector compensation for engine warmup */
    static varsize warmup1 ;
    static varsize warmup2 ;
    static varsize warmup3 ;    
    /* Injector compensation for load/speed( modifies warmup )*/
    static varsize warmupComp1 ;
    static varsize warmupComp2 ;
    static varsize warmupComp3 ;    
    /* Injector compensation for intake-air temp */
    static varsize airTempComp1 ;
    static varsize airTempComp2 ;
    static varsize airTempComp3 ;   
    /* Time( angle )to turn on igniter */
    static varsize firingTime1 ;
    static varsize firingTime2 ;
    static varsize firingTime3 ;    
    /* Igniter on-time duration */
    static varsize dwell1 ;
    static varsize dwell2 ;
    static varsize dwell3 ;    
    /* Igniter advance for load/speed */
    static varsize advance1 ;
    static varsize advance2 ;
    static varsize advance3 ;    
    /* Igniter compensation for engine temp */
    static varsize tempComp1 ;
    static varsize tempComp2 ;
    static varsize tempComp3 ;   
    /* Igniter dwell compensation for battery volts */
    static varsize dwellComp1 ;
    static varsize dwellComp2 ;
    static varsize dwellComp3 ; 
    /* Time( angle )to retard igniter due to knocking */
    static varsize retard1 ;
    static varsize retard2 ;
    static varsize retard3 ;    
    /* Injector compensation for acceleration( change in demand )*/
    static varsize accelComp1 ;
    static varsize accelComp2 ;
    static varsize accelComp3 ;    
    /* Amount to decrease compensation due to acceleration */
    static varsize accelDelta1 ;
    static varsize accelDelta2 ;
    static varsize accelDelta3 ;    

    /* Unused */
    argc = argc ;
    argv = argv ;

    /*
     * First, initialize the data structures we need for the test
     * and allocate memory as needed.  Report an error if we can't.
     *
     */    

    /* Variable initializations at t=0 */    

    /* No engine load history, */
    lastAirIntake1 = 0 ;   
    lastAirIntake2 = 0 ; 
    lastAirIntake3 = 0 ; 
    /*  and no engine speed history */
    lastEngSpeed1 = 0 ;    
    lastEngSpeed2 = 0 ; 
    lastEngSpeed3 = 0 ; 
    /*  and no throttle history */
    lastThrottle1 = 0 ;    
    lastThrottle2 = 0 ; 
    lastThrottle3 = 0 ; 
    /* Don't have to retard due to knock */
    retard1 = 0 ;    
    retard2 = 0 ; 
    retard3 = 0 ; 
    /* Just want to start the delta somewhere */
    accelDelta1 = 0 ;    
    accelDelta2 = 0 ; 
    accelDelta3 = 0 ; 
    /*  and there's been no acceleration */
    accelComp1 = 0 ;    
    accelComp2 = 0 ; 
    accelComp3 = 0 ; 
    /* Set base firing time somewhere */
    firingTime1 = FIRE_TIME_NOM ;    
    firingTime2 = FIRE_TIME_NOM ; 
    firingTime3 = FIRE_TIME_NOM ; 
    /*  and set base dwell time somewhere */
    dwell1 = DWELL_MIN ;    
    dwell2 = DWELL_MIN ; 
    dwell3 = DWELL_MIN ; 
    /*  and set base injector time somewhere */
    injectTime1 = INJECT_TIME_NOM ;    
    injectTime2 = INJECT_TIME_NOM ; 
    injectTime3 = INJECT_TIME_NOM ; 
    /*  and set base injector duration somewhere */
    inject1 = INJECT_MIN ;    
    inject2 = INJECT_MIN ; 
    inject3 = INJECT_MIN ; 
    /* No ignition advance yet */
    advance1 = 0 ;    
    advance2 = 0 ; 
    advance3 = 0 ; 
    /*  and no air/fuel mix */
    lambda1 = 0 ;    
    lambda2 = 0 ; 
    lambda3 = 0 ; 
    /* Hardware enable for the fuel pump */
    fuelPump1 = false ;    
    fuelPump2 = false ; 
    fuelPump3 = false ; 
    /* Hardware enable for the igniters */
    enbIgnition1 = false ;    
    enbIgnition2 = false ; 
    enbIgnition3 = false ; 
    /* The injector( s )is/are on to deliver fuel */
    injectOn1 = false ;    
    injectOn2 = false ; 
    injectOn3 = false ; 
    /* Set base firing time */
    firingTime1 = FIRE_TIME_NOM ;    
    firingTime2 = FIRE_TIME_NOM ; 
    firingTime3 = FIRE_TIME_NOM ; 
    /*  and set base dwell time */
    dwell1 = DWELL_MIN ;    
    dwell2 = DWELL_MIN ; 
    dwell3 = DWELL_MIN ; 
    /*  and set base injector time */
    injectTime1 = INJECT_TIME_NOM ;    
    injectTime2 = INJECT_TIME_NOM ; 
    injectTime3 = INJECT_TIME_NOM ; 
    /*  and set base injector duration */
    inject1 = INJECT_MIN ;    
    inject2 = INJECT_MIN ; 
    inject3 = INJECT_MIN ; 
    tableCount = 0 ;    /* Start out at beginning of input test data */

    /* If debug output is desired, then must allocate some RAM... */    
    RAMfilePtr = 0 ;   /* Point to beginning of test output file */
    /* Set size of output file (1K) */    
    RAMfileSize = MAX_FILESIZE ; 

    /* Allocate some RAM for output file */    
    RAMfileSize = MAX_FILESIZE ;

    /* Allocate some RAM for output file */   
    RAMfile         = (n_int *)th_malloc( RAMfileSize * sizeof(n_int) + sizeof (varsize) ) ;
    RAMfileFree     = RAMfile;
    if ( RAMfile == NULL )
          th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s:%d", __FILE__,__LINE__ );

 
	/* align it to varsize boundary */ 
	RAMfile       = ((n_int *)ROUNDUP(RAMfile, sizeof (varsize) ));
    RAMfilePtr    = RAMfile; /* Point to beginning of test output file */
    RAMfileEOF    = RAMfile + RAMfileSize;  /* should be OK ANSI C */

    if (sizeof(varsize) > sizeof(n_int)) {    /* this is a floating point benchmark! */
        RAMfile_increment = sizeof(varsize) / sizeof(n_int);
        if (sizeof(varsize) % sizeof(n_int)) RAMfile_increment += 1;  /* should never happen, but ... */
    }
    else{
        RAMfile_increment = 1;
    }
  
    /* Tell us the compiled data size */    
#if BMDEBUG
    th_sprintf( szDataSize, "Data size = %s\n\n", 
        ( DATA_SIZE ? "LONG" : "SHORT" ) ) ;
#else
#if DATA_SIZE
    szDataSize[0] = 'L' ; 
#else
    szDataSize[0] = 'S' ; 
#endif
    szDataSize[1] = '\0' ; 
#endif /* BMDEBUG */

    /* Initialize the test data -- stimuli for the algorithm. */

    if( !GetTestData() )   /* Allocate for the test input data table */
    {
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
            __FILE__, __LINE__ ) ; 
    }

      tcdef->CRC = 0;
    th_signal_start() ;    /* Tell the host that the test has begun */

#if BMDEBUG
    DebugOut( szTitle ) ;       /* Print the title message in RAM file */
    DebugOut( szDataSize ) ;    /* ...and the data size */
    DebugOut( szHeader ) ;      /* Print the test output file hdr in RAM file */
#endif /* BMDEBUG */

    GetZTables() ;  /* Get the 3D tables into RAM, or point to ROM */
    GetYTables() ;  /* Get the 2D tables into RAM, or point to ROM */

    /* This is the actual benchmark algorithm. */    

    /*
    * Perform tooth-to-spark algorithm : 
    *
    * 1. Get input variables for engine speed, crankshaft position, 
    *    intake air quantity, intake air temperature, engine temperature, 
    *    throttle position, battery voltage.
    *
    * 2. Output to the fuel pump, only when engine is running( engine
    *    speed is present ).
    *
    * 3. Output to the ignition for spark, via 'firingTime'.  This is adjusted
    *    by engine speed, engine load( intake air quantity )and engine temp.
    *    Also may be retarded due to engine knock.
    *
    * 4. Output to the fuel injectors to control fuel flow.  The injector 
    *    duration is adjusted by engine speed, engine load, intake air temp,
    *    acceleration and battery voltage.
    *
    * It is presumed that engine speed and crankshaft position have
    * been developed from the cranshaft tonewheel pulses elsewhere in
    * the controller( see Algorithm #2 ).
    *
    */    

    for( loop_cnt = 0 ; loop_cnt < tcdef->rec_iterations  ; loop_cnt++ )
    {

#if BMDEBUG
        if( isTableLooped ) 
        {
            DebugOut( "END--END--END\n" )
        }
#endif /* BMDEBUG */

        /***********************************************************************
            First Pass                                                          
        ***********************************************************************/

        /* Gets a set of input variables from test data*/
        isTableLooped = GetInputValues() ;    

#if BMDEBUG        
        th_sprintf( szDebug, " %5ld, %5ld, %5ld, %5ld, %5ld, %5ld, %5ld, %2d, %2d", 
            (n_long)engSpeed, 
            (n_long)angleTime, 
            (n_long)airIntake, 
            (n_long)airTemp, 
            (n_long)engTemp, 
            (n_long)battery, 
            (n_long)throttle, 
            knocked, 
            cranking ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Engine running ? */
        if( engSpeed > 0 )
        {            
            /* Yes, enable fuel pump */
            fuelPump1 = true ;    
            /* ...and allow ignition firing */
            enbIgnition1 = true ;    

            /* Engine just cranking ? */
            if( !cranking )
            {            

                /* No, engine running, but has an engine parameter changed ? */
                if( ( airIntake != lastAirIntake1 ) || 
                    ( engSpeed != lastEngSpeed1 ) || 
                    ( throttle != lastThrottle1 ) )
                {        
                    /* Decelerating engine speed with same or less load ? */
                    /* ...or excessive RPM's ? */
                    if( ( ( engSpeed < lastEngSpeed1 ) &&
                          ( airIntake <= lastAirIntake1 ) ) ||
                          ( engSpeed > ENG_SPEED_MAX ) )
                    {            
                        /* Yes, so no more injection! */
                        inject1 = 0 ;    
                        /* ...and no enrichment for acceleration */
                        accelComp1 = 0 ;    
                    }
                    else 
                    {            
                        /* Else, normal injection */
                    
                        /* 
                         * Compute baseline injector duration from 
                         * load/speed map 
                         *
                         */    

                        lambda1 = ZTableLookup( airIntake, engSpeed, 
                            loadEntries, speedEntries, airIntakeAxis, 
                            engSpeedAxis, lambdaTbl ) ; 
                        
                        inject1 = lambda1 ; 

                        /* 
                         * Enrich based on engine-warmup, compensated by 
                         * warmup load/speed map 
                         *
                         */    

                        warmup1 = YTableLookup( engTemp, eTempEntries, 
                            eTempAxis, engWarmTbl ) ; 
                        
                        warmupComp1 = (varsize)( ( ZTableLookup( airIntake, 
                            engSpeed, loadEntries, speedEntries, 
                            airIntakeAxis, engSpeedAxis, warmupTbl ) * 
                            warmup1 ) / 100 ) ; 

                        inject1 = (varsize)
                            ( ( inject1 * warmupComp1 ) / 100 ) ; 

                        /* Adjust enrichment based on intake-air temperature */    
                        airTempComp1 = YTableLookup( airTemp, aTempEntries,
                            aTempAxis, airTempTbl ) ; 
 
                        inject1 = (varsize)
                            ( ( inject1 * airTempComp1 ) / 100 ) ; 

                        /* Adjust for change in throttle position( demand )*/    
                        if( ( throttle - lastThrottle1 ) > ACCEL_MIN )
                        {            
                            /* Only if accelerating */
                            accelComp1 = YTableLookup( engTemp, eTempEntries,
                                eTempAxis, accelTbl ) ; 
                            accelDelta1 = accelComp1 >> ACCEL_TIME ;                        
                        }
                        else 
                        {            
                            /* Else, maybe compensation is ongoing */
                            if( accelComp1 > 0 )   
                            {
                                /* So, decrease it with ECU time */
                                accelComp1 -= accelDelta1 ; 
                            }

                            if( accelComp1 < 0 )
                            {
                                /* ...until no more compensation */
                                accelComp1 = 0 ;    
                            }
                        }

                        /* ...and compensate for acceleration */        
                        inject1 = (varsize)
                            ( ( inject1 * ( 100 + accelComp1 ) ) / 100 ) ; 
                        
                        /* Adjust injector duration for battery voltage */    
                        voltComp1 = YTableLookup( battery, voltEntries,
                            voltAxis, voltTbl ) ; 

                        inject1 = ( varsize )( inject1 + voltComp1 ) ; 

                        /* Watch out for injector duration maximum limit */    
                        if( inject1 > INJECT_MAX ){
                            inject1 = INJECT_MAX ; 
                        }
                    } /* End of 'normal injection' */

                    /* 
                     * Determine the ignition advance, from engine speed and 
                     * air intake 
                     *
                     */    
                    advance1 = ZTableLookup( airIntake, engSpeed, loadEntries,
                        speedEntries, airIntakeAxis, engSpeedAxis,
                        advanceTbl ) ; 

                    /* Adjust the ignition advance for engine temp */    
                    
                    tempComp1 = YTableLookup( engTemp, eTempEntries,
                        eTempAxis, engTempTbl ) ; 

                    firingTime1 = (varsize)
                       ( FIRE_TIME_NOM - ( ( advance1 * tempComp1 ) / 100 ) ) ; 

                    /* 
                     * Compute the dwell time compensation from battery 
                     * voltage 
                     *
                     */    
                    dwellComp1 = ZTableLookup( battery, engSpeed, battEntries,
                        speedEntries, batteryAxis, engSpeedAxis, dwellTbl ) ; 

                    dwell1 = (varsize)
                        ( ( DWELL_MIN * ( dwellComp1 + 100 ) ) / 100 ) ; 

                    /* No knock adjustment */
                    retard1 = 0 ;    

                } /* End of 'ignition/dwell adjustment' */
                else 
                {  
                    /* Else, did engine knock ? */
                    if( knocked && ( retard1 < KNOCK_MAX ) )
                    {        
                        /* Yes, so bump the ingnition retarder */
                        retard1 += KNOCK_STEP ;
                    }
                    else 
                    {        
                        if( retard1 > 0 )
                        {
                            /* Else, decrease the knock retarder to zero */
                            retard1 -= KNOCK_STEP ;    
                        }
                    }
                } /* End of 'knock detection' */

                /* Adjust firing time with knock retarder */
                firingTime1 = (varsize)( firingTime1 - retard1 ) ;
            } /* End of 'not just cranking' */
            else 
            {        
                /* Set base firing time */
                firingTime1 = FIRE_TIME_NOM ;    
                /* ...and set base dwell time */
                dwell1 = DWELL_MIN ;    
                /* ...and set base injector time */
                injectTime1 = INJECT_TIME_NOM ;    
                /* ...and set base injector duration */
                inject1 = INJECT_MIN ;    
            } /* End of 'just cranking' */

            /* Update engine parameter history */
            lastAirIntake1 = airIntake ;
            lastEngSpeed1 = engSpeed ; 
            lastThrottle1 = throttle ; 

        } /* End of 'engine running' */
        else 
        {            
            /* Else, engine is stalled! */
            /* So don't pump any fuel */
            fuelPump1 = false ;
            /* ...and don't fire the ignitors */
            enbIgnition1 = false ;
            /* Yes, so no more injection( duration )! */
            inject1 = 0 ;    
            /* ...and no enrichment for acceleration */
            accelComp1 = 0 ;    
            /* Set base firing time */
            firingTime1 = FIRE_TIME_NOM ;    
            /* ...and set base dwell time */
            dwell1 = DWELL_MIN ;    
            /* ...and set base injector time */
            injectTime1 = INJECT_TIME_NOM ;    

        } /* End of 'stalled engine' */

#if BMDEBUG        
        th_sprintf( szDebug, 
            "\n[%5ld, %5ld, %5ld, %5ld, %5ld, %5ld, "
            "%5ld, %2d, %2d, %2d]\n", 
            (n_long)dwell1, 
            (n_long)firingTime1, 
            (n_long)inject1, 
            (n_long)advance1, 
            (n_long)retard1, 
            (n_long)accelComp1, 
            (n_long)lambda1, 
            injectOn1, 
            fuelPump1, 
            enbIgnition1 ) ; 
        DebugOut( szDebug ) ; 
#else        /* Or write value directly to RAM file */
        WriteOut( firingTime1 ) ; 
        WriteOut( inject1 ) ; 
        WriteOut( advance1 ) ; 
        WriteOut( retard1 ) ; 
        WriteOut( accelComp1 ) ; 
        WriteOut( lambda1 ) ; 
        WriteOut( injectOn1 ) ; 
        WriteOut( fuelPump1 ) ; 
        WriteOut( enbIgnition1 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/

        isTableLooped += GetInputValues() ;

#if BMDEBUG        
        th_sprintf( szDebug, " %5ld, %5ld, %5ld, %5ld, %5ld, %5ld, %5ld, %2d, %2d", 
            (n_long)engSpeed, 
            (n_long)angleTime, 
            (n_long)airIntake, 
            (n_long)airTemp, 
            (n_long)engTemp, 
            (n_long)battery, 
            (n_long)throttle, 
            knocked, 
            cranking ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Engine running ? */
        if( engSpeed > 0 )
        {            
            /* Yes, enable fuel pump */
            fuelPump2 = true ;    
            /* ...and allow ignition firing */
            enbIgnition2 = true ;    

            /* Engine just cranking ? */
            if( !cranking )
            {            

                /* No, engine running, but has an engine parameter changed ? */
                if( ( airIntake != lastAirIntake2 ) || 
                    ( engSpeed != lastEngSpeed2 ) || 
                    ( throttle != lastThrottle2 ) )
                {        
                    /* Decelerating engine speed with same or less load ? */
                    /* ...or excessive RPM's ? */
                    if( ( ( engSpeed < lastEngSpeed2 ) &&
                          ( airIntake <= lastAirIntake2 ) ) ||
                          ( engSpeed > ENG_SPEED_MAX ) )
                    {            
                        /* Yes, so no more injection! */
                        inject2 = 0 ;    
                        /* ...and no enrichment for acceleration */
                        accelComp2 = 0 ;    
                    }
                    else 
                    {            
                        /* Else, normal injection */
                    
                        /* 
                         * Compute baseline injector duration from 
                         * load/speed map 
                         *
                         */    

                        lambda2 = ZTableLookup( airIntake, engSpeed, 
                            loadEntries, speedEntries, airIntakeAxis, 
                            engSpeedAxis, lambdaTbl ) ; 
                        
                        inject2 = lambda2 ; 

                        /* 
                         * Enrich based on engine-warmup, compensated by 
                         * warmup load/speed map 
                         *
                         */    

                        warmup2 = YTableLookup( engTemp, eTempEntries, 
                            eTempAxis, engWarmTbl ) ; 
                        
                        warmupComp2 = (varsize)( ( ZTableLookup( airIntake, 
                            engSpeed, loadEntries, speedEntries, 
                            airIntakeAxis, engSpeedAxis, warmupTbl ) * 
                            warmup2 ) / 100 ) ; 

                        inject2 = (varsize)
                            ( ( inject2 * warmupComp2 ) / 100 ) ; 

                        /* Adjust enrichment based on intake-air temperature */    
                        airTempComp2 = YTableLookup( airTemp, aTempEntries,
                            aTempAxis, airTempTbl ) ; 
 
                        inject2 = (varsize)
                            ( ( inject2 * airTempComp2 ) / 100 ) ; 

                        /* Adjust for change in throttle position( demand )*/    
                        if( ( throttle - lastThrottle2 ) > ACCEL_MIN )
                        {            
                            /* Only if accelerating */
                            accelComp2 = YTableLookup( engTemp, eTempEntries,
                                eTempAxis, accelTbl ) ; 
                            accelDelta2 = accelComp2 >> ACCEL_TIME ;                        
                        }
                        else 
                        {            
                            /* Else, maybe compensation is ongoing */
                            if( accelComp2 > 0 )   
                            {
                                /* So, decrease it with ECU time */
                                accelComp2 -= accelDelta2 ; 
                            }

                            if( accelComp2 < 0 )
                            {
                                /* ...until no more compensation */
                                accelComp2 = 0 ;    
                            }
                        }

                        /* ...and compensate for acceleration */        
                        inject2 = (varsize)
                            ( ( inject2 * ( 100 + accelComp2 ) ) / 100 ) ; 
                        
                        /* Adjust injector duration for battery voltage */    
                        voltComp2 = YTableLookup( battery, voltEntries,
                            voltAxis, voltTbl ) ; 

                        inject2 = ( varsize )( inject2 + voltComp2 ) ; 

                        /* Watch out for injector duration maximum limit */    
                        if( inject2 > INJECT_MAX ){
                            inject2 = INJECT_MAX ; 
                        }
                    } /* End of 'normal injection' */

                    /* 
                     * Determine the ignition advance, from engine speed and 
                     * air intake 
                     *
                     */    
                    advance2 = ZTableLookup( airIntake, engSpeed, loadEntries,
                        speedEntries, airIntakeAxis, engSpeedAxis,
                        advanceTbl ) ; 

                    /* Adjust the ignition advance for engine temp */    
                    
                    tempComp2 = YTableLookup( engTemp, eTempEntries,
                        eTempAxis, engTempTbl ) ; 

                    firingTime2 = (varsize)
                       ( FIRE_TIME_NOM - ( ( advance2 * tempComp2 ) / 100 ) ) ; 

                    /* 
                     * Compute the dwell time compensation from battery 
                     * voltage 
                     *
                     */    
                    dwellComp2 = ZTableLookup( battery, engSpeed, battEntries,
                        speedEntries, batteryAxis, engSpeedAxis, dwellTbl ) ; 

                    dwell2 = (varsize)
                        ( ( DWELL_MIN * ( dwellComp2 + 100 ) ) / 100 ) ; 

                    /* No knock adjustment */
                    retard2 = 0 ;    

                } /* End of 'ignition/dwell adjustment' */
                else 
                {  
                    /* Else, did engine knock ? */
                    if( knocked && ( retard2 < KNOCK_MAX ) )
                    {        
                        /* Yes, so bump the ingnition retarder */
                        retard2 += KNOCK_STEP ;
                    }
                    else 
                    {        
                        if( retard2 > 0 )
                        {
                            /* Else, decrease the knock retarder to zero */
                            retard2 -= KNOCK_STEP ;    
                        }
                    }
                } /* End of 'knock detection' */

                /* Adjust firing time with knock retarder */
                firingTime2 = (varsize)( firingTime2 - retard2 ) ;
            } /* End of 'not just cranking' */
            else 
            {        
                /* Set base firing time */
                firingTime2 = FIRE_TIME_NOM ;    
                /* ...and set base dwell time */
                dwell2 = DWELL_MIN ;    
                /* ...and set base injector time */
                injectTime2 = INJECT_TIME_NOM ;    
                /* ...and set base injector duration */
                inject2 = INJECT_MIN ;    
            } /* End of 'just cranking' */

            /* Update engine parameter history */
            lastAirIntake2 = airIntake ;
            lastEngSpeed2 = engSpeed ; 
            lastThrottle2 = throttle ; 

        } /* End of 'engine running' */
        else 
        {            
            /* Else, engine is stalled! */
            /* So don't pump any fuel */
            fuelPump2 = false ;
            /* ...and don't fire the ignitors */
            enbIgnition2 = false ;
            /* Yes, so no more injection( duration )! */
            inject2 = 0 ;    
            /* ...and no enrichment for acceleration */
            accelComp2 = 0 ;    
            /* Set base firing time */
            firingTime2 = FIRE_TIME_NOM ;    
            /* ...and set base dwell time */
            dwell2 = DWELL_MIN ;    
            /* ...and set base injector time */
            injectTime2 = INJECT_TIME_NOM ;    

        } /* End of 'stalled engine' */

#if BMDEBUG        
        th_sprintf( szDebug, 
            "\n[%5ld, %5ld, %5ld, %5ld, %5ld, %5ld, "
            "%5ld, %2d, %2d, %2d]\n", 
            (n_long)dwell2, 
            (n_long)firingTime2, 
            (n_long)inject2, 
            (n_long)advance2, 
            (n_long)retard2, 
            (n_long)accelComp2, 
            (n_long)lambda2, 
            injectOn2, 
            fuelPump2, 
            enbIgnition2 ) ; 
        DebugOut( szDebug ) ; 
#else        /* Or write value directly to RAM file */
        WriteOut( firingTime2 ) ; 
        WriteOut( inject2 ) ; 
        WriteOut( advance2 ) ; 
        WriteOut( retard2 ) ; 
        WriteOut( accelComp2 ) ; 
        WriteOut( lambda2 ) ; 
        WriteOut( injectOn2 ) ; 
        WriteOut( fuelPump2 ) ; 
        WriteOut( enbIgnition2 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/

        isTableLooped += GetInputValues() ;

#if BMDEBUG        
        th_sprintf( szDebug, " %5ld, %5ld, %5ld, %5ld, %5ld, %5ld, %5ld, %2d, %2d", 
            (n_long)engSpeed, 
            (n_long)angleTime, 
            (n_long)airIntake, 
            (n_long)airTemp, 
            (n_long)engTemp, 
            (n_long)battery, 
            (n_long)throttle, 
            knocked, 
            cranking ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Engine running ? */
        if( engSpeed > 0 )
        {            
            /* Yes, enable fuel pump */
            fuelPump3 = true ;    
            /* ...and allow ignition firing */
            enbIgnition3 = true ;    

            /* Engine just cranking ? */
            if( !cranking )
            {            

                /* No, engine running, but has an engine parameter changed ? */
                if( ( airIntake != lastAirIntake3 ) || 
                    ( engSpeed != lastEngSpeed3 ) || 
                    ( throttle != lastThrottle3 ) )
                {        
                    /* Decelerating engine speed with same or less load ? */
                    /* ...or excessive RPM's ? */
                    if( ( ( engSpeed < lastEngSpeed3 ) &&
                          ( airIntake <= lastAirIntake3 ) ) ||
                          ( engSpeed > ENG_SPEED_MAX ) )
                    {            
                        /* Yes, so no more injection! */
                        inject3 = 0 ;    
                        /* ...and no enrichment for acceleration */
                        accelComp3 = 0 ;    
                    }
                    else 
                    {            
                        /* Else, normal injection */
                    
                        /* 
                         * Compute baseline injector duration from 
                         * load/speed map 
                         *
                         */    

                        lambda3 = ZTableLookup( airIntake, engSpeed, 
                            loadEntries, speedEntries, airIntakeAxis, 
                            engSpeedAxis, lambdaTbl ) ; 
                        
                        inject3 = lambda3 ; 

                        /* 
                         * Enrich based on engine-warmup, compensated by 
                         * warmup load/speed map 
                         *
                         */    

                        warmup3 = YTableLookup( engTemp, eTempEntries, 
                            eTempAxis, engWarmTbl ) ; 
                        
                        warmupComp3 = (varsize)( ( ZTableLookup( airIntake, 
                            engSpeed, loadEntries, speedEntries, 
                            airIntakeAxis, engSpeedAxis, warmupTbl ) * 
                            warmup3 ) / 100 ) ; 

                        inject3 = (varsize)
                            ( ( inject3 * warmupComp3 ) / 100 ) ; 

                        /* Adjust enrichment based on intake-air temperature */    
                        airTempComp3 = YTableLookup( airTemp, aTempEntries,
                            aTempAxis, airTempTbl ) ; 
 
                        inject3 = (varsize)
                            ( ( inject3 * airTempComp3 ) / 100 ) ; 

                        /* Adjust for change in throttle position( demand )*/    
                        if( ( throttle - lastThrottle3 ) > ACCEL_MIN )
                        {            
                            /* Only if accelerating */
                            accelComp3 = YTableLookup( engTemp, eTempEntries,
                                eTempAxis, accelTbl ) ; 
                            accelDelta3 = accelComp3 >> ACCEL_TIME ;                        
                        }
                        else 
                        {            
                            /* Else, maybe compensation is ongoing */
                            if( accelComp3 > 0 )   
                            {
                                /* So, decrease it with ECU time */
                                accelComp3 -= accelDelta3 ; 
                            }

                            if( accelComp3 < 0 )
                            {
                                /* ...until no more compensation */
                                accelComp3 = 0 ;    
                            }
                        }

                        /* ...and compensate for acceleration */        
                        inject3 = (varsize)
                            ( ( inject3 * ( 100 + accelComp3 ) ) / 100 ) ; 
                        
                        /* Adjust injector duration for battery voltage */    
                        voltComp3 = YTableLookup( battery, voltEntries,
                            voltAxis, voltTbl ) ; 

                        inject3 = ( varsize )( inject3 + voltComp3 ) ; 

                        /* Watch out for injector duration maximum limit */    
                        if( inject3 > INJECT_MAX ){
                            inject3 = INJECT_MAX ; 
                        }
                    } /* End of 'normal injection' */

                    /* 
                     * Determine the ignition advance, from engine speed and 
                     * air intake 
                     *
                     */    
                    advance3 = ZTableLookup( airIntake, engSpeed, loadEntries,
                        speedEntries, airIntakeAxis, engSpeedAxis,
                        advanceTbl ) ; 

                    /* Adjust the ignition advance for engine temp */    
                    
                    tempComp3 = YTableLookup( engTemp, eTempEntries,
                        eTempAxis, engTempTbl ) ; 

                    firingTime3 = (varsize)
                       ( FIRE_TIME_NOM - ( ( advance3 * tempComp3 ) / 100 ) ) ; 

                    /* 
                     * Compute the dwell time compensation from battery 
                     * voltage 
                     *
                     */    
                    dwellComp3 = ZTableLookup( battery, engSpeed, battEntries,
                        speedEntries, batteryAxis, engSpeedAxis, dwellTbl ) ; 

                    dwell3 = (varsize)
                        ( ( DWELL_MIN * ( dwellComp3 + 100 ) ) / 100 ) ; 

                    /* No knock adjustment */
                    retard3 = 0 ;    

                } /* End of 'ignition/dwell adjustment' */
                else 
                {  
                    /* Else, did engine knock ? */
                    if( knocked && ( retard3 < KNOCK_MAX ) )
                    {        
                        /* Yes, so bump the ingnition retarder */
                        retard3 += KNOCK_STEP ;
                    }
                    else 
                    {        
                        if( retard3 > 0 )
                        {
                            /* Else, decrease the knock retarder to zero */
                            retard3 -= KNOCK_STEP ;    
                        }
                    }
                } /* End of 'knock detection' */

                /* Adjust firing time with knock retarder */
                firingTime3 = (varsize)( firingTime3 - retard3 ) ;
            } /* End of 'not just cranking' */
            else 
            {        
                /* Set base firing time */
                firingTime3 = FIRE_TIME_NOM ;    
                /* ...and set base dwell time */
                dwell3 = DWELL_MIN ;    
                /* ...and set base injector time */
                injectTime3 = INJECT_TIME_NOM ;    
                /* ...and set base injector duration */
                inject3 = INJECT_MIN ;    
            } /* End of 'just cranking' */

            /* Update engine parameter history */
            lastAirIntake3 = airIntake ;
            lastEngSpeed3 = engSpeed ; 
            lastThrottle3 = throttle ; 

        } /* End of 'engine running' */
        else 
        {            
            /* Else, engine is stalled! */
            /* So don't pump any fuel */
            fuelPump3 = false ;
            /* ...and don't fire the ignitors */
            enbIgnition3 = false ;
            /* Yes, so no more injection( duration )! */
            inject3 = 0 ;    
            /* ...and no enrichment for acceleration */
            accelComp3 = 0 ;    
            /* Set base firing time */
            firingTime3 = FIRE_TIME_NOM ;    
            /* ...and set base dwell time */
            dwell3 = DWELL_MIN ;    
            /* ...and set base injector time */
            injectTime3 = INJECT_TIME_NOM ;    

        } /* End of 'stalled engine' */

#if BMDEBUG        
        th_sprintf( szDebug, 
            "\n[%5ld, %5ld, %5ld, %5ld, %5ld, %5ld, "
            "%5ld, %2d, %2d, %2d]\n", 
            (n_long)dwell3, 
            (n_long)firingTime3, 
            (n_long)inject3, 
            (n_long)advance3, 
            (n_long)retard3, 
            (n_long)accelComp3, 
            (n_long)lambda3, 
            injectOn3, 
            fuelPump3, 
            enbIgnition3 ) ; 
        DebugOut( szDebug ) ; 
#else        /* Or write value directly to RAM file */
        WriteOut( firingTime3 ) ; 
        WriteOut( inject3 ) ; 
        WriteOut( advance3 ) ; 
        WriteOut( retard3 ) ; 
        WriteOut( accelComp3 ) ; 
        WriteOut( lambda3 ) ; 
        WriteOut( injectOn3 ) ; 
        WriteOut( fuelPump3 ) ; 
        WriteOut( enbIgnition3 ) ; 

#if DATA_SIZE == 0
        i = (varsize)( loop_cnt & 0x0000FFFF ) ; 
        WriteOut( i ) ;
        i = (varsize)( loop_cnt >> 16 ) ; 
        WriteOut( i ) ; 
#else
        WriteOut( loop_cnt ) ; 
#endif
        i = 0xAAAA ; 
        WriteOut( i ) ;
#endif /* BMDEBUG */

#if BMDEBUG
        if( !th_harness_poll() )break ; 
#endif /* BMDEBUG */
    } 

	tcdef->duration = th_signal_finished() ;
    tcdef->iterations = loop_cnt ; 
    tcdef->v1 = 0 ; 
    tcdef->v2 = 0 ; 
    tcdef->v3 = 0 ; 
    tcdef->v4 = 0 ; 

/* CRC_CHECK ok */
#if NON_INTRUSIVE_CRC_CHECK
/* Final results iteration dependent */
	tcdef->CRC=0;
	for (loop_cnt=0;loop_cnt<NUM_TESTS;loop_cnt++) {
		tcdef->CRC = Calc_crc32((e_u32)engSpeedROM[loop_cnt],tcdef->CRC); 
		tcdef->CRC = Calc_crc32((e_u32)airIntakeROM[loop_cnt],tcdef->CRC); 
		tcdef->CRC = Calc_crc32((e_u32)airTempROM[loop_cnt],tcdef->CRC); 
		tcdef->CRC = Calc_crc32((e_u32)engTempROM[loop_cnt],tcdef->CRC); 
		tcdef->CRC = Calc_crc32((e_u32)batteryROM[loop_cnt],tcdef->CRC); 
	}
#elif	CRC_CHECK
/* Iteration dependant results check */
	tcdef->CRC=0;
	tcdef->CRC = Calc_crc32((e_u32)firingTime1,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)inject1,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)advance1,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)retard1,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)accelComp1,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)lambda1,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)injectOn1,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)fuelPump1,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)enbIgnition1,tcdef->CRC); 

	tcdef->CRC = Calc_crc32((e_u32)firingTime2,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)inject2,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)advance2,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)retard2,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)accelComp2,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)lambda2,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)injectOn2,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)fuelPump2,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)enbIgnition2,tcdef->CRC); 

	tcdef->CRC = Calc_crc32((e_u32)firingTime3,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)inject3,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)advance3,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)retard3,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)accelComp3,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)lambda3,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)injectOn3,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)fuelPump3,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)enbIgnition3,tcdef->CRC); 
#else
	tcdef->CRC=0;
#endif

	return	th_report_results(tcdef,EXPECTED_CRC);
} 

/* Accepts 3 parameters, scratch address is fixed
   Code inspired by a section in Cortex R Programmers' Guide:
   TCM ECC fault injection

 1) Address to be corrupted
 2) 4-bytes bit mask, tell which bit to corrupt
 3) Shift, depending on which byte to corrupt, shift appropriately
    as to store byte that is not changed
*/
void corrupt_tcm(u32 addr,u32 mask,u32 shift){
#ifndef NO_ECC
    asm volatile("PUSH {r0,r1,r2,r3,r4,r10}");
    asm volatile("MOV r10,%0\n\t" : : "r"(addr));
    asm volatile("LDR r4, = 0x2fff0\n\t");
    asm volatile("LDRD r0, [r10]\n\t");
/*Introduce corruption to word according to mask*/
    asm volatile("EOR r0, %0\n\t" : : "r"(mask));
    asm volatile("STRB r0, [r10]\n\t"
                 "ROR r0, r0, #24\n\t"
                 "STRB r0, [r10,#3]\n\t"
                 "ROR r0, r0, #16\n\t"
                 "STRB r0, [r10,#1]\n\t"
                 "ROR r0,r0, #8\n\t"
                 "STRB r0, [r10,#2]\n\t");
/*Turn off ECC Check*/
    asm volatile("MRC p15,0,r0,c1,c0\n\t"
    "BIC r0,r0,#0x0E000000\n\t"
    "MCR p15,0,r0,c1,c0,1\n\t"
    "DMB\n\t"
    "DSB\n\t"
    "ISB\n\t");
/*Load corrupted address and store to scratch location*/
    asm volatile("LDRD r0,[r10]\n\t");
    asm volatile("STRD r0,[r4]\n\t");
    asm volatile("EOR r0, %0\n\t": : "r"(mask));
/* Do unaligned store to scratch to avoid ECC recomputation */
    asm volatile("STRB r0, [r4]\n\t"
                 "ROR r0, r0, #24\n\t"
                 "STRB r0, [r4,#3]\n\t"
                 "ROR r0, r0, #16\n\t"
                 "STRB r0, [r4,#1]\n\t"
                 "ROR r0,r0, #8\n\t"
                 "STRB r0, [r4,#2]\n\t");
/*Load corrected data, but with ECC bits according to corrupted data*/
    asm volatile("LDRD r0, [r4]\n\t");
    asm volatile("LSR r2,r0, %0\n\t" : : "r"(shift));
    asm volatile("LSR r3,r3,#3\n\t");
/*Store byte that is uncorrupted to set up the internal registers*/
/*This stage seems important, if not storing uncorrupted byte,
the injection does not work*/
    asm volatile ("STRB r2,[r10,r3]\n\t");
/* Turn On ECC Check again */
    asm volatile("MRC p15,0,r0,c1,c0,1\n\t"
    "ORR r0,r0,#0x0E000000\n\t"
    "MCR p15,0,r0,c1,c0,1\n\t"
    "DMB\n\t"
    "DSB\n\t"
    "ISB\n\t");
    asm volatile("POP {r0,r1,r2,r3,r4,r10}");
#else
/*If No ECC Check, just corrupt word according to mask as per usual*/
    asm volatile("PUSH {r0,r1,r2,r3,r4,r10}");
    asm volatile("MOV r10,%0\n\t" : : "r"(addr));
    asm volatile("LDRD r0, [r10]\n\t");
    asm volatile("EOR r0, %0\n\t" : : "r"(mask));
    asm volatile("STR r0, [r10]");
    asm volatile("POP {r0,r1,r2,r3,r4,r10}");
#endif
}

/***************************************************************************/
n_int failTest;
n_int benchIter;
int main(int argc, const char* argv[] )
{
    init_platform();
/* */

    XIpiPsu_Config *CfgPtr;

    int Status = XST_FAILURE;

    /* Look Up the config data */
    CfgPtr = XIpiPsu_LookupConfig(TEST_CHANNEL_ID);

    /* Init with the Cfg Data */
    XIpiPsu_CfgInitialize(&IpiInst, CfgPtr, CfgPtr->BaseAddress);

    /* Setup the GIC */
    SetupInterruptSystem(&GicInst, &IpiInst, (IpiInst.Config.IntId));

    /* Enable reception of IPIs from all CPUs */
    XIpiPsu_InterruptEnable(&IpiInst, XIPIPSU_ALL_MASK);

    /* Clear Any existing Interrupts */
    XIpiPsu_ClearInterruptStatus(&IpiInst, XIPIPSU_ALL_MASK);
    /* initialise variable to 0 */
    failTest = 0;
    benchIter = 0;
    /* target specific inititialization */
    al_main(argc, argv);
    xil_printf(">>     Start of ttsprk...\n\r");
    /* Benchmark Execution */
    while (failTest == 0) {
        failTest = t_run_test(&the_tcdef,argc,argv);
        if (failTest != 0)
        {
            
            xil_printf(">>     Dumping RAMfile information to the log...\n\r");
            for (n_int i = 0 ; i < RAMfileSize ; i++)
            {
                xil_printf("%8u\n\r",*RAMfilePtr++);
            }
        } else {
            th_free(RAMfileFree); /* Free RAMfile for next iteration so no Malloc error */ 
            th_free(engSpeedValue);
            th_free(angleTimeValue);
            th_free(airIntakeValue);
            th_free(airTempValue);
            th_free(engTempValue);
            th_free(batteryValue);
            th_free(throttleValue);
            th_free(knockedValue);
            th_free(crankingValue);
            xil_printf("%20d\n\r",benchIter++);

            
        }
    }
    xil_printf(">>      ttsprk test is finished\n\r");
    /*cleanup_platform();*/
    return failTest;
} 


#if BMDEBUG

n_void DebugOut( n_char *szSrc )
{        
#if RAM_OUT == 1
    n_int i ;
    n_int length ; 

    /* Make sure we don't overwrite the end of the RAM file */    
    length = (n_int)strlen( szSrc ) ; 

    for( i = 0 ; i < length ; i++ )
    {        
        *RAMfilePtr++ = szSrc[i] ; 

        if ( RAMfilePtr >= RAMfileEOF )
            RAMfilePtr = RAMfile;
    }
#else
#if WINDOWS_EXAMPLE_CODE
    printf( szSrc ) ;
#endif
#endif /* RAM_OUT */

} /* End of function 'DebugOut' */

#endif /* BMDEBUG */

/*
*    Function :  WriteOut
*
*    Outputs results to the RAM file so that it can be downloaded to host for
*  verification.  Also serves to defeat optimization which would remove the
*  code used to develop the results when not in DEBUG mode.
*
*/    

n_void	WriteOut( varsize value )
{

if (( RAMfilePtr+RAMfile_increment) > RAMfileEOF )
    RAMfilePtr = RAMfile;

    *(varsize *)RAMfilePtr = value;

} /* End of function 'WriteOut' */


/*
* FUNC    : ZTableLookup
*
* DESC    : 
*         
* RETURNS :
*
*/    

varsize
ZTableLookup( varsize xValue, 
              varsize yValue, 
              n_long xEntries, 
              n_long yEntries, 
              const varsize *xAxis, 
              const varsize *yAxis, 
              const varsize *zTable )
{        

    n_int i ;
    n_int j ; 
    n_long xDelta ;
    n_long yDelta ; 
    varsize outValue ; 

    /* First, find the 'x' axis grid coefficients */    
    for( i = 0 ; i < ( xEntries - 1 ) ; i++ )
    {        
        if( ( xValue < xAxis[i + 1] ) && ( xValue >= xAxis[i] ) )
        {
            break ;    
        }
    }

    if( i == ( xEntries - 1 ) )
    {
        xValue = xAxis[i] ; 
    }

    /*  
     * Use the 'x' coefficients to calculate the proportional delta
     * such that 'xDelta' is in the range 0..1000 
     *
     */    
    xDelta = (n_long)( ( xValue - xAxis[i] )* 1000 ) / 
                     ( xAxis[i + 1] - xAxis[i] ) ; 

    /* Next, find the 'y' axis grid coefficients */    
    for( j = 0 ; j < ( yEntries - 1 ) ; j++ )
    {        
        if( ( yValue < yAxis[j + 1] ) && ( yValue >= yAxis[j] ) )
        {
            break ;    
        }
    }

    if( j == ( yEntries - 1 ) )yValue = yAxis[j] ; 

    /*  
     * Use the 'y' coefficients to calculate the proportional grid delta
     * such that 'yDelta' is in the range 0..1000 
     *
     */    

    yDelta = (n_long)( ( yValue - yAxis[j] )* 1000 ) / 
                     ( yAxis[j + 1] - yAxis[j] ) ; 

    /*  
     * Now we can determine the interpolated output value for the input 
     * data from the table 'z' values 
     *
     */    

    outValue = (varsize)( ( ( ( 1000 - xDelta ) * ( 1000 - yDelta ) * 
        (n_long)zTable[i +( j * xEntries )] ) / 1000000 ) + 
        ( ( xDelta * ( 1000 - yDelta ) * 
        (n_long)zTable[( i + 1 )+( j *xEntries )] ) / 1000000 ) + 
        ( ( xDelta *yDelta *(n_long)zTable[( i + 1 ) + 
        ( xEntries *( j + 1 ) )] )/ 1000000 ) + 
        ( ( ( 1000 - xDelta ) * yDelta * 
        (n_long)zTable[i +( xEntries * ( j + 1 ) )] ) / 1000000 ) ) ; 

    return outValue ; 
} /* End of function 'ZTableLookup' */

/*
* FUNC    : ZTableLookup
*
* DESC    : 
*         
* RETURNS :
*
*/    

varsize
 YTableLookup( varsize xValue,
               n_long xEntries, 
               const varsize *xAxis, 
               const varsize *yTable )
{        

    n_int i ; 
    n_long xDelta ; 
    varsize outValue ; 

    /*  First, find the 'x' axis range coefficients */    
    for( i = 0 ; i < ( xEntries - 1 ) ; i++ )
    {        
        if( ( xValue < xAxis[i + 1] ) && ( xValue >= xAxis[i] ) )
        {
            break ;    
        }
    }

    if( i == ( xEntries - 1 ) )
    {
        xValue = xAxis[i] ; 
    }

    /*
     * Use the 'x' coefficients to calculate the proportional delta
     * such that 'xDelta' is in the range 0..1000 
     *
     */    
    xDelta = (n_long)( ( xValue - xAxis[i] ) * 1000 ) / 
                     ( xAxis[i + 1] - xAxis[i] ) ; 

    /*  
     * Now we can determine the interpolated output value for the input data 
     * from the table 'y' values 
     *
     */    

    outValue = (varsize)( ( ( ( 1000 - xDelta ) * (n_long)yTable[i] ) / 1000 ) +
        ( ( xDelta * (n_long)yTable[i + 1] ) / 1000 ) ) ; 

    return outValue ; 

} /* End of function 'YTableLookup' */

void IpiIntrHandler(void *XIpiPsuPtr)
{

    u32 IpiSrcMask; /**< Holds the IPI status register value */
    u32 Index;


    u32 SrcIndex;
    XIpiPsu *InstancePtr = (XIpiPsu *) XIpiPsuPtr;

    Xil_AssertVoid(InstancePtr!=NULL);

    IpiSrcMask = XIpiPsu_GetInterruptStatus(InstancePtr);

    /* Poll for each source and read response*/
    /* Based on response, corrupt TCM accordingly*/
    for (SrcIndex = 0U; SrcIndex < InstancePtr->Config.TargetCount;
            SrcIndex++) {

        if (IpiSrcMask & InstancePtr->Config.TargetList[SrcIndex].Mask) {

#ifdef LISTEN_A53
            XIpiPsu_ReadMessage(InstancePtr,XPAR_XIPIPS_TARGET_PSU_CORTEXA53_0_CH0_MASK, TmpBufPtr,
                    TEST_MSG_LEN, XIPIPSU_BUF_TYPE_MSG);
#else
            XIpiPsu_ReadMessage(InstancePtr,XPAR_XIPIPS_TARGET_PSU_CORTEXR5_1_CH0_MASK, TmpBufPtr,
                    TEST_MSG_LEN, XIPIPSU_BUF_TYPE_MSG);
#endif
            Xil_DCacheFlush();
            corrupt_tcm(TmpBufPtr[0],TmpBufPtr[1],TmpBufPtr[2]);
            xil_printf("TCM addr corrupted: 0x%x\r\n, Bit Mask : 0x%x ,Shift %d, bits corrupted: %d\r\n",TmpBufPtr[0],TmpBufPtr[1],TmpBufPtr[2],TmpBufPtr[3]);

            XIpiPsu_ClearInterruptStatus(InstancePtr,InstancePtr->Config.TargetList[SrcIndex].Mask);
        }
    }


}

static XStatus SetupInterruptSystem(XScuGic *IntcInstancePtr,XIpiPsu *IpiInstancePtr, u32 IpiIntrId)
{
    u32 Status = 0;
    XScuGic_Config *IntcConfig; /* Config for interrupt controller */

    /* Initialize the interrupt controller driver */
    IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
    if (NULL == IntcConfig) {
        return XST_FAILURE;
    }

    Status = XScuGic_CfgInitialize(&GicInst, IntcConfig,
            IntcConfig->CpuBaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /*
     * Connect the interrupt controller interrupt handler to the
     * hardware interrupt handling logic in the processor.
     */
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
            (Xil_ExceptionHandler) XScuGic_InterruptHandler, IntcInstancePtr);

    /*
     * Connect a device driver handler that will be called when an
     * interrupt for the device occurs, the device driver handler
     * performs the specific interrupt processing for the device
     */
     xil_printf("Interrupt ID for R5_0: %d\r\n",IpiIntrId);
    Status = XScuGic_Connect(IntcInstancePtr, IpiIntrId,
            (Xil_InterruptHandler) IpiIntrHandler, (void *) IpiInstancePtr);

    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    /* Enable the interrupt for the device */
    XScuGic_Enable(IntcInstancePtr, IpiIntrId);

    /* Enable interrupts */
    Xil_ExceptionEnable();

    return XST_SUCCESS;
}
