/*==============================================================================
 *$RCSfile: bmark_lite.c,v $
 *
 *   DESC : This file contains the Test Main and other TH support functions
 *
 * AUTHOR : ARM, Ltd., Rick Foos, ECL, LLC
 *
 *  EEMBC : Automotive Subcommittee 
 *
 *    CVS : $Revision: 1.15 $
 *          $Date: 2002/08/07 22:21:44 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/rspeed01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.15  2002/08/07 22:21:44  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.14  2002/07/22 21:59:32  rick
 * General cleanup Beta 2b
 *
 * Revision 1.13  2002/07/22 16:10:16  rick
 * Fix iteration dependant NI CRC
 *
 * Revision 1.12  2002/07/19 23:10:33  rick
 * Fix iteration dependant NI CRC's
 *
 * Revision 1.11  2002/07/10 19:01:25  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.10  2002/05/29 22:25:32  rick
 * Set recommended iterations with make
 *
 * Revision 1.9  2002/05/10 23:57:47  rick
 * Fix missing shifts in 16, and 32 bit CRC calculations
 *
 * Revision 1.8  2002/05/10 17:20:37  rick
 * Add al_main to API
 *
 * Revision 1.7  2002/04/25 20:10:45  rick
 * sprintf to th_sprintf
 *
 * Revision 1.6  2002/04/19 18:31:38  rick
 * Bug #146: global tablecount uninitialized
 *
 * Revision 1.5  2002/04/10 19:36:59  rick
 * Fixes to reduce Lite vs. Regular variances in timing
 *
 * Revision 1.4  2002/03/12 18:31:04  rick
 * ITERATIONS, CRC_CHECK, NON_INTRUSIVE_CRC_CHECK, add standards headers
 *
 * Revision 1.3  2002/02/25 17:15:33  rick
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

#define ALGO_GLOBALS    1
#include "algo.h"

/* Estimate of allocation for NUM_TESTS*( debug test + 2 variables )*/
#define T_BSIZE (MAX_FILESIZE+((NUM_TESTS+1)*VAR_COUNT*4))

/* ======================================================================== */
/*         F U N C T I O N   P R O T O T Y P E S                            */
/* ======================================================================== */
int main(int argc, const char* argv[] );
int t_run_test(struct TCDef *tcdef, int argc, const char* argv[]);

/* Define iterations */
#if !defined(ITERATIONS) || CRC_CHECK || ITERATIONS==DEFAULT
#undef ITERATIONS
#if CRC_CHECK
#define ITERATIONS 100000	/* required iterations for crc */
#else
#define ITERATIONS 100000	/* recommended iterations for benchmark */
#endif
#endif

/* Pre-caclculated CRC values */
#if	CRC_CHECK
#define EXPECTED_CRC 0xe631
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC 0x76c7
#else
#define EXPECTED_CRC 0x0000
#endif

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/

TCDef the_tcdef = 
{
	"AUT rspeed01   ",
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Algorithm #6 -- Road Speed Calculation V1.0E0 - rspeed00", 
    TCDEF_REVISION, 
    { EEMBC_TH_MAJOR, EEMBC_TH_MINOR, EEMBC_TH_STEP, EEMBC_TH_REVISION }, 
    { 0, 0, 0, 0 },   /* Target Hardware Version Number Required( make all zeros to ignore )*/
    { 1, 0, 'F', 0 }, /* The Version number of this Benchmark */
	ITERATIONS,    /* recomended iterations for benchmark score */
	0,
	0,
	0,
	0,
	0,
	0,
	0
} ; 

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/

/* Source of the test data, in "ROM" */
extern const varsize inpTonewheelROM[] ;    

n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree; /* NOT PART OF BENCHMARK to free RAMfile */

/* Current 'tonewheelCounter' pulled from test data */
varsize tonewheelCounter ;   
/* Array of 'tonewheelCounter' test data values */
varsize *inpTonewheelCount ; 
/* Number of teeth on the tonewheel */
varsize tonewheelTeeth ;     

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
        "Algorithm 6 :  Road Speed Calculation  Rev. 1.0F0\n" ; 
    char *szHeader = 
        "\n\ntonewheelCount, toothDeltaT, toothCount, toothTimeAccum, roadSpeed"
        ", counter\n" ; 
    char szDebug[100] ; 
#endif /* BMDEBUG */
    varsize i ; 

    static char szDataSize[40] ; 
    /* Input test data table looped */
    int isTableLooped = false ;    
    /* Output variable 'roadSpeed' is calculated */
    static varsize roadSpeed1 ;    
    static varsize roadSpeed2 ; 
    static varsize roadSpeed3 ; 
    /*  from inter-pulse period, 'toothDeltaTime' */
    static varsize toothDeltaTime1 ;    
    static varsize toothDeltaTime2 ; 
    static varsize toothDeltaTime3 ; 
    /* History of 'toothDeltaTime' */
    static varsize toothDeltaTimeLast1 ;    
    static varsize toothDeltaTimeLast2 ; 
    static varsize toothDeltaTimeLast3 ; 
    /* History of the pulse timer/counter */
    static varsize tonewheelCounterLast1 ;    
    static varsize tonewheelCounterLast2 ; 
    static varsize tonewheelCounterLast3 ; 
    /* Accumulator for filtering pulse time */
    static long toothTimeAccum1 ;    
    static long toothTimeAccum2 ; 
    static long toothTimeAccum3 ; 
    /* Counter for filtering pulse time */
    static varsize toothCount1 ;    
    static varsize toothCount2 ; 
    static varsize toothCount3 ; 

    /* Unused */
    argc = argc ;
    argv = argv ;

    /* 
     * First, initialize the data structures we need for the test
     * and allocate memory as needed.  Report an error if we can't.
     *
     */    

    /* Variable initializations at t=0 */    

    /* ...and pulse filter */
    toothCount1 = 0 ;    
    toothCount2 = 0 ; 
    toothCount3 = 0 ; 
    /* ...and haven't accumulated for filter */
    toothTimeAccum1 = 0 ;    
    toothTimeAccum2 = 0 ; 
    toothTimeAccum3 = 0 ; 
    /* ...Road speed starts at '0' */
    roadSpeed1 = 0 ;    
    roadSpeed2 = 0 ; 
    roadSpeed3 = 0 ; 
    /* ...and no pulse history yet */
    tonewheelCounterLast1 = 0 ;    
    tonewheelCounterLast2 = 0 ; 
    tonewheelCounterLast3 = 0 ; 
    toothDeltaTimeLast1 = MAX_TOOTH_TIME ; 
    toothDeltaTimeLast2 = MAX_TOOTH_TIME ; 
    toothDeltaTimeLast3 = MAX_TOOTH_TIME ; 
    tableCount = 0 ;    /* Start out at beginning of input test data */

    /* If debug output is desired, then must allocate some RAM... */    
    RAMfilePtr = 0 ;    /* Point to beginning of test output file */
    /* Set size of output file (1K)*/    
    RAMfileSize = MAX_FILESIZE; 
   
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
    szDataSize[0] = ( char )( '0' + DATA_SIZE ) ; 
    szDataSize[1] = '\0' ; 
#endif /* BMDEBUG */

    /* Initialize the test data -- stimuli for the algorithm. */
    if( !GetTestData() )   /* Allocate for the test input data table */
    {
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
            __FILE__, __LINE__ ) ; 
    }

      tcdef->CRC = 0;
    th_signal_start() ; /* Tell the host that the test has begun */

#if BMDEBUG
    DebugOut( szTitle ) ;    /* Print the title message in RAM file */
    DebugOut( szDataSize ) ; /* ...and the data size */
    DebugOut( szHeader ) ;   /* Print the test output file header in RAM file */
#endif /* BMDEBUG */

    /* This is the actual benchmark algorithm. */    

    /* Perform road-speed calculation */    

    for( loop_cnt = 0 ; loop_cnt < tcdef->rec_iterations  ; loop_cnt++ )
    {

#if BMDEBUG
        if( isTableLooped )
        {
            DebugOut( "END--END--END\n" ) ;
        }
#endif /* BMDEBUG */

        /***********************************************************************
            First Pass                                                          
        ***********************************************************************/

        isTableLooped = GetInputValues() ;

#if BMDEBUG
        th_sprintf( szDebug, "%10ld", (long)tonewheelCounter ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*
         * Compute 'toothDeltaTime' -- the difference in counter 
         * readings from the last pulse to the current one.  Note that
         * the realtime counter may roll over, so the elapsed time may
         * need to be corrected when this occurs.
         *
         * The 'roadSpeed' calculation will be performed twice for each
         * revolution of the tonewheel, so some filtering of pulse-period
         * is performed.
         *
         * If the period between pulses goes out to some maximum, then
         * road speed will be forced to zero.  Also, if the period between
         * pulses gets unrealistically small, we'll ignore it, presuming
         * that we're just picking up noise.
         *
         * We also watch out for strange excursions in the pulse period, to
         * filter out errors :  if the period suddenly jumps to several times
         * the last period measured, then it is ignored.
         *
         */    

        /* Compute period between pulses */    
        toothDeltaTime1 = (varsize)
            ( tonewheelCounter - tonewheelCounterLast1 ) ; 

        /* Check for timer overflow */
        if( tonewheelCounterLast1 > tonewheelCounter )
        {
            /* ...and correct for it */
            toothDeltaTime1 += ( varsize )( MAX_VARIABLE + 1 ) ;
        }

        /* Watch out for erroneously short */
        if( toothDeltaTime1 < MIN_TOOTH_TIME )   
        {
            /* ...pulse period */
            toothDeltaTime1 = toothDeltaTimeLast1 ;    
        }
        
        /* Watch out for erroneously long */
        if( toothDeltaTime1 > 4 * toothDeltaTimeLast1 )
        {
            /* ...pulse period */
            toothDeltaTime1 = toothDeltaTimeLast1 ;
        }

        /* Update timer history */
        tonewheelCounterLast1 = tonewheelCounter ;
        /* ...and accumulate pulses for filtering */
        toothTimeAccum1 += toothDeltaTime1 ;
        /* ...counting the pulses being filtered */
        toothCount1++ ;
        /* Remember this pulse's period */
        toothDeltaTimeLast1 = toothDeltaTime1 ;

        /* Time to update ? */
        if( toothCount1 >= tonewheelTeeth / 2 )
        {            
            /* Yes, */
            if( toothTimeAccum1 > MAX_TOOTH_TIME *tonewheelTeeth / 2 )   
            {    
                /* ...check for zero road speed */
                roadSpeed1 = 0 ;    
            }
            else
            {
                /* ...or compute road speed */
                roadSpeed1 = (varsize ) ( SPEEDO_SCALE_FACTOR  / 
                      ( toothTimeAccum1 / tonewheelTeeth * 2 ) ) ; 
                /* ...then reset the filter counter */
                toothCount1 = 0 ;
                /* ...and clear the accumulator */
                toothTimeAccum1 = 0 ;
            }
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %4ld, %4ld, %6ld", (long)toothDeltaTime1, 
            (long)toothCount1, (long)toothTimeAccum1 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %6ld\n", (long)roadSpeed1 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( roadSpeed1 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/

        isTableLooped += GetInputValues() ;

#if BMDEBUG
        th_sprintf( szDebug, "%10ld", (long)tonewheelCounter ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Compute period between pulses */    
        toothDeltaTime2 = (varsize)
            ( tonewheelCounter - tonewheelCounterLast2 ) ; 

        /* Check for timer overflow */
        if( tonewheelCounterLast2 > tonewheelCounter )
        {
            /* ...and correct for it */
            toothDeltaTime2 += ( varsize )( MAX_VARIABLE + 1 ) ;
        }

        /* Watch out for erroneously short */
        if( toothDeltaTime2 < MIN_TOOTH_TIME )   
        {
            /* ...pulse period */
            toothDeltaTime2 = toothDeltaTimeLast2 ;    
        }
        
        /* Watch out for erroneously long */
        if( toothDeltaTime2 > 4 * toothDeltaTimeLast2 )
        {
            /* ...pulse period */
            toothDeltaTime2 = toothDeltaTimeLast2 ;
        }

        /* Update timer history */
        tonewheelCounterLast2 = tonewheelCounter ;
        /* ...and accumulate pulses for filtering */
        toothTimeAccum2 += toothDeltaTime2 ;
        /* ...counting the pulses being filtered */
        toothCount2++ ;
        /* Remember this pulse's period */
        toothDeltaTimeLast2 = toothDeltaTime2 ;

        /* Time to update ? */
        if( toothCount2 >= tonewheelTeeth / 2 )
        {            
            /* Yes, */
            if( toothTimeAccum2 > MAX_TOOTH_TIME *tonewheelTeeth / 2 )   
            {    
                /* ...check for zero road speed */
                roadSpeed2 = 0 ;    
            }
            else
            {
                /* ...or compute road speed */
                roadSpeed2 = (varsize ) ( SPEEDO_SCALE_FACTOR  / 
                      ( toothTimeAccum2 / tonewheelTeeth * 2 ) ) ; 
                /* ...then reset the filter counter */
                toothCount2 = 0 ;
                /* ...and clear the accumulator */
                toothTimeAccum2 = 0 ;
            }
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %4ld, %4ld, %6ld", (long)toothDeltaTime2, 
            (long)toothCount2, (long)toothTimeAccum2 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %6ld\n", (long)roadSpeed2 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( roadSpeed2 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/

        isTableLooped += GetInputValues() ;

#if BMDEBUG
        th_sprintf( szDebug, "%10ld", (long)tonewheelCounter ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Compute period between pulses */    
        toothDeltaTime3 = (varsize)
            ( tonewheelCounter - tonewheelCounterLast3 ) ; 

        /* Check for timer overflow */
        if( tonewheelCounterLast3 > tonewheelCounter )
        {
            /* ...and correct for it */
            toothDeltaTime3 += ( varsize )( MAX_VARIABLE + 1 ) ;
        }

        /* Watch out for erroneously short */
        if( toothDeltaTime3 < MIN_TOOTH_TIME )   
        {
            /* ...pulse period */
            toothDeltaTime3 = toothDeltaTimeLast3 ;    
        }
        
        /* Watch out for erroneously long */
        if( toothDeltaTime3 > 4 * toothDeltaTimeLast3 )
        {
            /* ...pulse period */
            toothDeltaTime3 = toothDeltaTimeLast3 ;
        }

        /* Update timer history */
        tonewheelCounterLast3 = tonewheelCounter ;
        /* ...and accumulate pulses for filtering */
        toothTimeAccum3 += toothDeltaTime3 ;
        /* ...counting the pulses being filtered */
        toothCount3++ ;
        /* Remember this pulse's period */
        toothDeltaTimeLast3 = toothDeltaTime3 ;

        /* Time to update ? */
        if( toothCount3 >= tonewheelTeeth / 3 )
        {            
            /* Yes, */
            if( toothTimeAccum3 > MAX_TOOTH_TIME *tonewheelTeeth / 3 )   
            {    
                /* ...check for zero road speed */
                roadSpeed3 = 0 ;    
            }
            else
            {
                /* ...or compute road speed */
                roadSpeed3 = (varsize ) ( SPEEDO_SCALE_FACTOR  / 
                      ( toothTimeAccum3 / tonewheelTeeth * 3 ) ) ; 
                /* ...then reset the filter counter */
                toothCount3 = 0 ;
                /* ...and clear the accumulator */
                toothTimeAccum3 = 0 ;
            }
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %4ld, %4ld, %6ld", (long)toothDeltaTime3, 
            (long)toothCount3, (long)toothTimeAccum3 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %6ld\n", (long)roadSpeed3 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( roadSpeed3 ) ; 
#if DATA_SIZE == 0
        i = (varsize)( loop_cnt &0x0000FFFF ) ; 
        WriteOut( i ) ;
        i = (varsize)( loop_cnt >> 16 ) ; 
        WriteOut( i ) ; 
#else
        WriteOut( loop_cnt ) ; 
#endif /* DATA_SIZE */
        i = ( varsize )0xAAAA ; 
        WriteOut( i ) ;
#endif /* BMDEBUG */

#if BMDEBUG
        if( !th_harness_poll() )
        {
            break ; 
        }
#endif /* BMDEBUG */
    }

	tcdef->duration = th_signal_finished() ;
    tcdef->iterations = loop_cnt ; 
    tcdef->v1 = 0 ; 
    tcdef->v2 = 0 ; 
    tcdef->v3 = 0 ; 
    tcdef->v4 = 0 ; 

#if NON_INTRUSIVE_CRC_CHECK
/* Final results iteration dependent */
	tcdef->CRC=0;
	for (loop_cnt=0;loop_cnt<NUM_TESTS;loop_cnt++) 
		tcdef->CRC = Calc_crc32((e_u32)inpTonewheelCount[loop_cnt],tcdef->CRC);
#elif	CRC_CHECK
/* Iteration dependant results check */
	tcdef->CRC = 0;
	tcdef->CRC = Calc_crc32((e_u32)roadSpeed1,tcdef->CRC);
	tcdef->CRC = Calc_crc32((e_u32)roadSpeed2,tcdef->CRC);
	tcdef->CRC = Calc_crc32((e_u32)roadSpeed3,tcdef->CRC);
#else
	tcdef->CRC=0;
#endif

	return	th_report_results(tcdef,EXPECTED_CRC);
} 


/***************************************************************************/
n_int failTest;
n_int benchIter;
int main(int argc, const char* argv[] )
{
    /* initialise variable to 0 */
    failTest = 0;
    benchIter = 0;
    init_platform();
    /* target specific inititialization */
    al_main(argc, argv);
    xil_printf(">>     Start of rspeed...\n\r");
    /* Benchmark Execution */
    while (failTest == 0) {
        failTest = t_run_test(&the_tcdef,argc,argv);
        if (failTest != 0)
        {
            xil_printf(">>     CRC check has failed at iteration %8ld, see logfile\n\r",benchIter);
            xil_printf(">>     Dumping RAMfile information to the log...\n\r");
            for (n_int i = 0 ; i < RAMfileSize ; i++)
            {
                xil_printf("%8ld\n\r",*RAMfilePtr++);
            }
        } else {
            th_free(RAMfileFree); /* Free RAMfile for next iteration so no Malloc error */ 
            th_free(inpTonewheelCount);

            xil_printf(">>   Test is working just fine, iteration: %8ld\n\r",benchIter++);
        }
    }
    xil_printf(">>      rspeed test is finished\n\r");
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
     RAMfilePtr += RAMfile_increment;

} /* End of function 'WriteOut' */

