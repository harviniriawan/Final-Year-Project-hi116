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
 *          $Date: 2002/08/07 22:20:38 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/aifirf01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.15  2002/08/07 22:20:38  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.14  2002/07/22 21:59:10  rick
 * General cleanup Beta 2b
 *
 * Revision 1.13  2002/07/22 16:09:59  rick
 * Fix iteration dependant NI CRC
 *
 * Revision 1.12  2002/07/18 23:33:48  rick
 * Fix iteration dependencies in NI CRC
 *
 * Revision 1.11  2002/07/10 19:00:49  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.10  2002/05/29 22:24:51  rick
 * Set recommended iterations with make
 *
 * Revision 1.9  2002/05/10 23:57:44  rick
 * Fix missing shifts in 16, and 32 bit CRC calculations
 *
 * Revision 1.8  2002/05/10 17:20:36  rick
 * Add al_main to API
 *
 * Revision 1.7  2002/04/25 20:10:43  rick
 * sprintf to th_sprintf
 *
 * Revision 1.6  2002/04/10 19:36:58  rick
 * Fixes to reduce Lite vs. Regular variances in timing
 *
 * Revision 1.5  2002/03/11 22:11:47  rick
 * ITERATIONS, CRC_CHECK, NON_INTRUSIVE TCDef Usage
 *
 * Revision 1.4  2002/02/25 17:15:31  rick
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
#define NO_ECC
/* Uncomment this define to listen for injection from R5_1 */
#define LISTEN_A53

XScuGic GicInst;
XIpiPsu IpiInst;

/* Buffers to store message from the other core */
static u32 TmpBufPtr[TEST_MSG_LEN] = { 0 };

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

#if CRC_CHECK
#define EXPECTED_CRC	0x0000
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC	0x9208
#else
#define EXPECTED_CRC	0x0000
#endif

TCDef the_tcdef = 
{
    "AUT aifirf01   ",
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Alg. #18 - Finite Impulse Response(FIR)Filter V1.0E0-aifirf01", 
    TCDEF_REVISION, 
    { EEMBC_TH_MAJOR, EEMBC_TH_MINOR, EEMBC_TH_STEP, EEMBC_TH_REVISION }, 
    { 0, 0, 0, 0 },   /* Target Hardware Version Number Required( make all zeros to ignore )*/
    { 1, 0, 'E', 0 }, /* The Version number of this Benchmark */
	ITERATIONS,
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

/* Input stimuli test data table */
/*extern const varsize inpSignalROM[] ;*/

n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree; /* NOT PART OF BENCHMARK to free RAMfile */

n_int   *inpSignal ;   /* Pointer to array of input signal data values */
varsize signal_in ;   /* The input signal to be filtered */
/* NOT PART OF BENCHMARK to free filter history */
/*
varsize firLow1HistoryFree;
varsize firLow2HistoryFree;
varsize firLow3HistoryFree;
varsize firHi1HistoryFree;
varsize firHi2HistoryFree;
varsize firHi3HistoryFree;
*/

extern FILTER_DEF firLow1 ;     /* The low-pass fir filter parameters */
extern FILTER_DEF firLow2 ;     /* The low-pass fir filter parameters */
extern FILTER_DEF firLow3 ;     /* The low-pass fir filter parameters */
extern FILTER_DEF firHi1 ;      /* The high-pass fir filter parameters */
extern FILTER_DEF firHi2 ;      /* The high-pass fir filter parameters */
extern FILTER_DEF firHi3 ;      /* The high-pass fir filter parameters */


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
        "\nEEMBC Automotive/Industrial Subcommittee Benchmarks (c)1998-1999\n"
        "Algorithm #18 -- Finite Impulse Response( FIR )Filter  V1.0E0 - aifirf01\n" ; 
    char *szHeader = 
        "\n\nsignal_in, signalOutLow, signalOutHi, loop counter\n" ; 
    char szDebug[100] ; 
#endif /* BMDEBUG */
    n_char szDataSize[40] ; 
    n_int isTableLooped = false ;    /* Input test data table looped */
    /* Filter coefficients pointer */
    static varsize *coefficient1 ;
    static varsize *coefficient2 ; 
    static varsize *coefficient3 ; 
    /* Low-pass filter feedback history' pointer */
    static varsize *history1Low1 ;    
    static varsize *history2Low1 ; 
    static varsize *history3Low1 ; 
    /* Low-pass filter feedback history'' pointer */
    static varsize *history1Low2 ;    
    static varsize *history2Low2 ; 
    static varsize *history3Low2 ; 
    /* High-pass filter feedback history' pointer */
    static varsize *history1Hi1 ;    
    static varsize *history2Hi1 ; 
    static varsize *history3Hi1 ; 
    /* High-pass filter feedback history'' pointer */
    static varsize *history1Hi2 ;    
    static varsize *history2Hi2 ; 
    static varsize *history3Hi2 ; 
    /* Filtered output signal for low-pass filter */
    static varsize signalOutLow1 ;    
    static varsize signalOutLow2 ; 
    static varsize signalOutLow3 ; 
    /* Filtered output signal for high-pass filter */
    static varsize signalOutHi1 ;    
    static varsize signalOutHi2 ; 
    static varsize signalOutHi3 ; 
    varsize i1 ;
    varsize i2 ;
    varsize i3 ; 

    /* Unused */
    argc = argc ;
    argv = argv ;

    /*
     * First, initialize the data structures we need for the test
     * and allocate memory as needed.  Report an error if we can't.
     *
     */    

    /* Variable initializations at t=0 */    
    tableCount = 0 ;    /* Start out at beginning of input test data */
    /* If debug output is desired, then must allocate some RAM... */    
    RAMfilePtr = 0 ;   /* Point to beginning of test output file */
    /* Set size of output file (1K) */    
    RAMfileSize = MAX_FILESIZE;

    /* Allocate some RAM for output file */    
    RAMfile         = (n_int *)th_malloc( RAMfileSize * sizeof(n_int) + sizeof (varsize) ) ;
    RAMfileFree = RAMfile;
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
    szDataSize[0] = (n_char)( '0' + DATA_SIZE ) ; 
    szDataSize[1] = '\0' ; 
#endif /* BMDEBUG */

    /* Initialize the test data -- stimuli for the algorithm */    

    if( !GetTestData() )   /* Allocate for the test input data table */
    {
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
            __FILE__, __LINE__ ) ; 
    }

    /* Allocate history arrays */    
    firLow1.history = (varsize*)
        th_malloc( firLow1.sections * sizeof(varsize) ) ; 
    firHi1.history = (varsize*)
        th_malloc( firHi1.sections * sizeof(varsize) ) ;

    if( ( firLow1.history == NULL ) || ( firHi1.history == NULL ) )
    {
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d", 
            __FILE__, __LINE__ ) ; 
    }

    /* Must clear out the history */

    /* ...for the low-pass filter */
    history1Low1 = firLow1.history ;
    for( i1 = 0 ; i1 < firLow1.sections ; i1++ )
    {
        *history1Low1++ = 0 ; 
    }

    /* ...and for the high-pass filter */
    history1Low1 = firHi1.history ; 
    for( i1 = 0 ; i1 < firHi1.sections ; i1++ )
    {
        *history1Low1++ = 0 ; 
    }

    /* Allocate again, for the second iteration */    

    firLow2.history = (varsize*)
        th_malloc( firLow2.sections * sizeof(varsize) ) ; 
    firHi2.history = (varsize*)
        th_malloc( firHi2.sections * sizeof(varsize) ) ; 

    if( ( firLow2.history == NULL ) || ( firHi2.history == NULL ) )
    {
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d", 
            __FILE__, __LINE__ ) ; 
    }

    /* Must clear out the history */

    /* ...for the low-pass filter */
    history2Low1 = firLow2.history ;
    for( i2 = 0 ; i2 < firLow2.sections ; i2++ )
    {
        *history2Low1++ = 0 ; 
    }

    /* ...and for the high-pass filter */
    history2Low1 = firHi2.history ; 
    for( i2 = 0 ; i2 < firHi2.sections ; i2++ )
    {
        *history2Low1++ = 0 ; 
    }

    /* And allocate a third time */    

    firLow3.history = (varsize*)
        th_malloc( firLow3.sections * sizeof(varsize) ) ; 
    firHi3.history = (varsize*)
        th_malloc( firHi3.sections * sizeof(varsize) ) ; 

    if( ( firLow3.history == NULL ) || ( firHi3.history == NULL ) )
    {
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d", 
            __FILE__, __LINE__ ) ; 
    }

    /* Must clear out the history */

    /* ...for the low-pass filter */
    history3Low1 = firLow3.history ;
    for( i3 = 0 ; i3 < firLow3.sections ; i3++ )
    {
        *history3Low1++ = 0 ; 
    }

    /* ...and for the high-pass filter */
    history3Low1 = firHi3.history ; 
    for( i3 = 0 ; i3 < firHi3.sections ; i3++ )
    {
        *history3Low1++ = 0 ; 
    }

    /* Tell the host that the test has begun */
      tcdef->CRC = 0;
    th_signal_start() ;

#if BMDEBUG
    DebugOut( szTitle ) ;     /* Print the title message in RAM file */
    DebugOut( szHeader ) ;    /* Print the test output file hdr in RAM file */
    DebugOut( szDataSize ) ;  /* ...and the data size */
#endif /* BMDEBUG */

    /*
     * This is the actual benchmark algorithm.
     *
     */    

    /*
     * Perform FIR filtering on a sample by sample basis.  Filters an input
     * signal 'signal_in' to produce two output signals : 'signalOutLow1' and
     * 'signalOutHi1'. There are two FIR filters, a low pass and a high pass.
     * 'signalOutLow1' is the output of the low-pass filter, 'signalOutHi1'
     * is the output of the hi-pass filter.
     *
     * There is a FILTER definition structure for each of the filters. So the
     * filters can actually be two different types, have different number of
     * sections, different coefficients, etc.
     *
     * The FILTER structure contains : 
     *   the number of filter sections, 
     *   pointer to "history" array, 
     *   pointer to filter coefficients.
     *
     * Note that the filter coefficients are scaled up to maintain precision.
     * Normally, the coefficients are float values with small magnitudes, so
     * their precision would be lost if simply converted to integers.
     * Therefore, the coefficients are scaled up. This means that whenever
     * they are used in computing the filter output and history, the results
     * must be rounded off and then scaled back down.
     *
     */ 


    for( loop_cnt = 0 ; loop_cnt < tcdef->rec_iterations ; loop_cnt++ )
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

        /* Gets 'signal_in' value from test data*/
        isTableLooped = GetInputValues() ;

#if BMDEBUG        
        th_sprintf( szDebug, "%6ld", (n_long)signal_in ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* LOW-PASS FIR FILTER */    

        /* Start by performing the low-pass FIR filter function */    

        /* Must set pointer to last coefficient1 */
        coefficient1 = firLow1.coef + firLow1.sections - 1 ;
        /* ...and point to history */
        history1Low1 = history1Low2 = firLow1.history ;
        
        /* First, compute output from history and filter coefficients */    

        signalOutLow1 = (varsize)( ( *coefficient1-- ) * ( *history1Low1++ ) ) ; 
    
        /* Must round off */    
        signalOutLow1 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    
        signalOutLow1 >>= COEF_SCALE ; 

        /* Ripple through the filter history */    
        for( i1 = 2 ; i1 < firLow1.sections ; i1++ )
        {        
            *history1Low2++ = *history1Low1 ; 
            signalOutLow1 += (varsize)
                ( ( *coefficient1-- ) * ( *history1Low1++ ) ) ; 

            /* Must round off */    
            signalOutLow1 += 1 <<( COEF_SCALE - 1 ) ; 
            /* ...and must scale down, because coefficients are scaled up */    
            signalOutLow1 >>= COEF_SCALE ; 
        }

        *history1Low1 = signal_in ; 

        /* Then bring in the input signal */    
        signalOutLow1 += (varsize)( ( *coefficient1 ) * signal_in ) ; 

        /* Must round off */    
        signalOutLow1 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    

        signalOutLow1 >>= COEF_SCALE ; 

        /* End of LOW-PASS FILTER */    

        /* HIGH-PASS FIR FILTER */    

        /* Next, perform the high-pass FIR filter function */    
        /* Must set pointer to last coefficient1 */
        coefficient1 = firHi1.coef + firHi1.sections - 1 ;
        /* ...and point to history */
        history1Hi1 = history1Hi2 = firHi1.history ;
        
        /* First, compute output from history and filter coefficients */    
    
        signalOutHi1 = (varsize)( ( *coefficient1-- ) * ( *history1Hi1++ ) ) ; 
    
        /* Must round off */    
        signalOutHi1 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    
        signalOutHi1 >>= COEF_SCALE ; 
    
        /* ...and ripple through the filter history */    
        for( i1 = 2 ; i1 < firHi1.sections ; i1++ )
        {        
            *history1Hi2++ = *history1Hi1 ; 
            signalOutHi1 += (varsize)
                ( ( *coefficient1-- ) * ( *history1Hi1++ ) ) ; 

            /* Must round off */    
            signalOutHi1 += 1 << ( COEF_SCALE - 1 ) ; 
            /* ...and must scale down, because coefficients are scaled up */    
            signalOutHi1 >>= COEF_SCALE ;        
        }

        *history1Hi1 = signal_in ; 

        /* Then bring in the input signal */    
        signalOutHi1 += (varsize)( ( *coefficient1 ) * signal_in ) ; 
        /* Must round off */    
        signalOutHi1 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    
        signalOutHi1 >>= COEF_SCALE ; 

        /* End of HIGH-PASS FILTER */    

#if BMDEBUG        
        th_sprintf( szDebug, ", %8ld, %8ld\n", (n_long)signalOutLow1, 
            (n_long)signalOutHi1 ) ; 
        DebugOut( szDebug ) ; 
#else       
        WriteOut( signalOutLow1 ) ; 
        WriteOut( signalOutHi1 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/

        /* Gets 'signal_in' value from test data*/
        isTableLooped += GetInputValues() ;
        
#if BMDEBUG        
        th_sprintf( szDebug, "%6ld", (n_long)signal_in ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* LOW-PASS FIR FILTER */    

        /* Start by performing the low-pass FIR filter function */    

        /* Must set pointer to last coefficient2 */
        coefficient2 = firLow1.coef + firLow1.sections - 1 ;
        /* ...and point to history */
        history2Low1 = history2Low2 = firLow1.history ;

        /* First, compute output from history and filter coefficients */    

        signalOutLow2 = (varsize)( ( *coefficient2-- ) * ( *history2Low1++ ) ) ; 
    
        /* Must round off */    
        signalOutLow2 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    
        signalOutLow2 >>= COEF_SCALE ; 

        /* Ripple through the filter history */    
        for( i2 = 2 ; i2 < firLow1.sections ; i2++ )
        {        
            *history2Low2++ = *history2Low1 ; 
            signalOutLow2 += (varsize)
                ( ( *coefficient2-- ) * ( *history2Low1++ ) ) ; 

            /* Must round off */    
            signalOutLow2 += 1 <<( COEF_SCALE - 1 ) ; 
            /* ...and must scale down, because coefficients are scaled up */    
            signalOutLow2 >>= COEF_SCALE ; 
        }

        *history2Low1 = signal_in ; 

        /* Then bring in the input signal */    
        signalOutLow2 += (varsize)( ( *coefficient2 ) * signal_in ) ; 

        /* Must round off */    
        signalOutLow2 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    

        signalOutLow2 >>= COEF_SCALE ; 

        /* End of LOW-PASS FILTER */    

        /* HIGH-PASS FIR FILTER */    

        /* Next, perform the high-pass FIR filter function */    
        /* Must set pointer to last coefficient2 */
        coefficient2 = firHi1.coef + firHi1.sections - 1 ;
        /* ...and point to history */
        history2Hi1 = history2Hi2 = firHi1.history ;
        
        /* First, compute output from history and filter coefficients */    
    
        signalOutHi2 = (varsize)( ( *coefficient2-- ) * ( *history2Hi1++ ) ) ; 
    
        /* Must round off */    
        signalOutHi2 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    
        signalOutHi2 >>= COEF_SCALE ; 
    
        /* ...and ripple through the filter history */    
        for( i2 = 2 ; i2 < firHi1.sections ; i2++ )
        {        
            *history2Hi2++ = *history2Hi1 ; 
            signalOutHi2 += (varsize)
                ( ( *coefficient2-- ) * ( *history2Hi1++ ) ) ; 

            /* Must round off */    
            signalOutHi2 += 1 << ( COEF_SCALE - 1 ) ; 
            /* ...and must scale down, because coefficients are scaled up */    
            signalOutHi2 >>= COEF_SCALE ;        
        }

        *history2Hi1 = signal_in ; 

        /* Then bring in the input signal */    
        signalOutHi2 += (varsize)( ( *coefficient2 ) * signal_in ) ; 
        /* Must round off */    
        signalOutHi2 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    
        signalOutHi2 >>= COEF_SCALE ; 

        /* End of HIGH-PASS FILTER */    

#if BMDEBUG        
        th_sprintf( szDebug, ", %8ld, %8ld\n", (n_long)signalOutLow2, 
            (n_long)signalOutHi2 ) ; 
        DebugOut( szDebug ) ; 
#else       
        WriteOut( signalOutLow2 ) ; 
        WriteOut( signalOutHi2 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/

        /* Gets 'signal_in' value from test data*/
        isTableLooped += GetInputValues() ; 
        
#if BMDEBUG        
        th_sprintf( szDebug, "%6ld", (n_long)signal_in ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* LOW-PASS FIR FILTER */    

        /* Start by performing the low-pass FIR filter function */    

        /* Must set pointer to last coefficient3 */
        coefficient3 = firLow1.coef + firLow1.sections - 1 ;
        /* ...and point to history */
        history3Low1 = history3Low2 = firLow1.history ;

        /* First, compute output from history and filter coefficients */    

        signalOutLow3 = (varsize)( ( *coefficient3-- ) * ( *history3Low1++ ) ) ; 
    
        /* Must round off */    
        signalOutLow3 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    
        signalOutLow3 >>= COEF_SCALE ; 

        /* Ripple through the filter history */    
        for( i3 = 2 ; i3 < firLow1.sections ; i3++ )
        {        
            *history3Low2++ = *history3Low1 ; 
            signalOutLow3 += (varsize)
                ( ( *coefficient3-- ) * ( *history3Low1++ ) ) ; 

            /* Must round off */    
            signalOutLow3 += 1 <<( COEF_SCALE - 1 ) ; 
            /* ...and must scale down, because coefficients are scaled up */    
            signalOutLow3 >>= COEF_SCALE ; 
        }

        *history3Low1 = signal_in ; 

        /* Then bring in the input signal */    
        signalOutLow3 += (varsize)( ( *coefficient3 ) * signal_in ) ; 

        /* Must round off */    
        signalOutLow3 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    

        signalOutLow3 >>= COEF_SCALE ; 

        /* End of LOW-PASS FILTER */    

        /* HIGH-PASS FIR FILTER */    

        /* Next, perform the high-pass FIR filter function */    
        /* Must set pointer to last coefficient3 */
        coefficient3 = firHi1.coef + firHi1.sections - 1 ;
        /* ...and point to history */
        history3Hi1 = history3Hi2 = firHi1.history ;
        
        /* First, compute output from history and filter coefficients */    
    
        signalOutHi3 = (varsize)( ( *coefficient3-- ) * ( *history3Hi1++ ) ) ; 
    
        /* Must round off */    
        signalOutHi3 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    
        signalOutHi3 >>= COEF_SCALE ; 
    
        /* ...and ripple through the filter history */    
        for( i3 = 2 ; i3 < firHi1.sections ; i3++ )
        {        
            *history3Hi2++ = *history3Hi1 ; 
            signalOutHi3 += (varsize)
                ( ( *coefficient3-- ) * ( *history3Hi1++ ) ) ; 

            /* Must round off */    
            signalOutHi3 += 1 << ( COEF_SCALE - 1 ) ; 
            /* ...and must scale down, because coefficients are scaled up */    
            signalOutHi3 >>= COEF_SCALE ;        
        }

        *history3Hi1 = signal_in ; 

        /* Then bring in the input signal */    
        signalOutHi3 += (varsize)( ( *coefficient3 ) * signal_in ) ; 
        /* Must round off */    
        signalOutHi3 += 1 << ( COEF_SCALE - 1 ) ; 
        /* ...and must scale down, because coefficients are scaled up */    
        signalOutHi3 >>= COEF_SCALE ; 

        /* End of HIGH-PASS FILTER */    

#if BMDEBUG        
        th_sprintf( szDebug, ", %8ld, %8ld\n", (n_long)signalOutLow3, 
            (n_long)signalOutHi3 ) ; 
        DebugOut( szDebug ) ; 
#else       
        WriteOut( signalOutLow3 ) ; 
        WriteOut( signalOutHi3 ) ; 

#if DATA_SIZE == 0
        i3 = (varsize)( loop_cnt &0x0000FFFF ) ; 
        WriteOut( i3 ) ;
        i3 = (varsize)( loop_cnt >> 16 ) ; 
        WriteOut( i3 ) ; 
#else
        WriteOut((varsize)loop_cnt ) ; 
#endif
        i3 = (varsize) 0xAAAA ; 
        WriteOut(i3 ) ;
#endif /* BMDEBUG */

#if BMDEBUG
        if( !th_harness_poll() )
        {
            break ; 
        }
#endif
    }

    
	tcdef->duration = th_signal_finished() ;
    tcdef->iterations = loop_cnt ; 
    tcdef->v1 = 0 ; 
    tcdef->v2 = 0 ; 
    tcdef->v3 = 0 ; 
    tcdef->v4 = 0 ; 
/* NON_INTRUSIVE_CRC_CHECK OK */
#if NON_INTRUSIVE_CRC_CHECK
	tcdef->CRC = 0;
	tcdef->CRC = Calc_crc32((e_u32)signalOutLow1,tcdef->CRC ) ; 
	tcdef->CRC = Calc_crc32((e_u32)signalOutHi1,tcdef->CRC ) ; 
	tcdef->CRC = Calc_crc32((e_u32)signalOutLow2,tcdef->CRC ) ; 
	tcdef->CRC = Calc_crc32((e_u32)signalOutHi2,tcdef->CRC ) ; 
	tcdef->CRC = Calc_crc32((e_u32)signalOutLow3,tcdef->CRC ) ; 
	tcdef->CRC = Calc_crc32((e_u32)signalOutHi3,tcdef->CRC ) ; 
#elif	CRC_CHECK
	tcdef->CRC=0;
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
#ifdef NO_ECC
/*Turn off ECC Check and correction if NO_ECC*/
    asm volatile("MRC p15,0,r0,c1,c0\n\t"
    "BIC r0,r0,#0x0E000000\n\t"
    "MCR p15,0,r0,c1,c0,1\n\t"
    "DMB\n\t"
    "DSB\n\t"
    "ISB\n\t");

    asm volatile("MRC p15,0,r0,c15,c0,0\n\t"
    "ORR r0,r0,#0xC\n\t"
    "MCR p15,0,r0,c15,c0,0\n\t"
    "DMB\n\t"
    "DSB\n\t"
    "ISB\n\t");
#endif
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
    xil_printf(">>     Start of FIR...\n\r");
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
            th_free(inpSignal);
            th_free(firLow1.history);
            th_free(firLow2.history);
            th_free(firLow3.history);
            th_free(firHi1.history);
            th_free(firHi2.history);
            th_free(firHi3.history);
            xil_printf("%20d\n\r",benchIter++);
    
        }
    }
    xil_printf(">>      FIR test is finished\n\r");
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
    th_printf( szSrc ) ;
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
     RAMfilePtr += RAMfile_increment; /* this might need to be casted back to (n_int *) ??*/

} /* End of function 'WriteOut' */

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
