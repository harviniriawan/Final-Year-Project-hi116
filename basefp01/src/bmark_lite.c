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
 *          $Date: 2002/08/07 22:20:45 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/basefp01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.15  2002/08/07 22:20:45  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.14  2002/07/22 21:59:13  rick
 * General cleanup Beta 2b
 *
 * Revision 1.13  2002/07/22 16:10:05  rick
 * Fix iteration dependant NI CRC
 *
 * Revision 1.12  2002/07/18 23:33:51  rick
 * Fix iteration dependencies in NI CRC
 *
 * Revision 1.11  2002/07/11 22:13:27  rick
 * Initialize tcdef results
 *
 * Revision 1.10  2002/07/10 19:01:01  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.9  2002/05/29 22:24:58  rick
 * Set recommended iterations with make
 *
 * Revision 1.8  2002/05/10 23:57:45  rick
 * Fix missing shifts in 16, and 32 bit CRC calculations
 *
 * Revision 1.7  2002/05/10 17:20:36  rick
 * Add al_main to API
 *
 * Revision 1.6  2002/04/25 20:10:44  rick
 * sprintf to th_sprintf
 *
 * Revision 1.5  2002/03/11 22:11:47  rick
 * ITERATIONS, CRC_CHECK, NON_INTRUSIVE TCDef Usage
 *
 * Revision 1.4  2002/02/25 17:15:32  rick
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
e_u16 Calc_crc16( e_u16 data, e_u16 crc );

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
#define EXPECTED_CRC	0xf33d
#else
#define EXPECTED_CRC	0x0000
#endif

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/


/* Input stimuli test data table */
extern const double inpVariableROM[] ;

n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree; /* NOT PART OF BENCHMARK to free RAMfile */

varsize *inpVariable ;      /* Pointer to array of input data values */
varsize *inpVariableFree ;  /* NOT PART OF BENCHMARK */
varsize inputNum ;          /* The input argument for computation */

/*
 * Arrays of coefficients of the terms of numerator( P )and
 * denominator( Q )of telescoping series for 'arctan( x )'.
 *
 */    

const double constantP[] = 
{        
    0.215140596260244e+05, 
    0.735974338028844e+05, 
    0.100272561830630e+06, 
    0.694392975003225e+05, 
    0.258580973971909e+05, 
    0.503863918550126e+04, 
    0.460158880463535e+03, 
    0.150876773587003e+02, 
    0.752305281875762e-01
} ; /* End of 'constantP' array */

const double constantQ[] = 
{
    0.215140596260244e+05, 
    0.807687870115592e+05, 
    0.122892678909278e+06, 
    0.973232034905355e+05, 
    0.428685765204640e+05, 
    0.104011349156689e+05, 
    0.128975056911611e+04, 
    0.685193783101896e+02, 
    0.100000000000000e+01
} ; /* End of 'constantQ' array */

#ifdef PI
#undef PI
#endif /* Some compilers define PI */
#define PI 3.14159265358979



TCDef the_tcdef = 
{
    "AUT basefp01   ",
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Alg. #10 - Basic Floating-Point V1.0C0 -basefp01", 
    TCDEF_REVISION, 
    { EEMBC_TH_MAJOR, EEMBC_TH_MINOR, EEMBC_TH_STEP, EEMBC_TH_REVISION }, 
    { 0, 0, 0, 0 },   /* Target Hardware Version Number Required( make all zeros to ignore )*/
    { 1, 0, 'C', 0 }, /* The Version number of this Benchmark */
	ITERATIONS,    /* recomended iterations for benchmark score */
	0,
	0,
	0,
	0,
	0,
	0,
	0
} ; 

/*  DECLARATIONS */    

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
    n_char *szTitle = 
        "\nEEMBC Automotive/Industrial Subcommittee Benchmarks (c)1998-1999 - bainfp01 \n"
        "Algorithm 10 : Basic Integer and Floating Point "
        "Functions  Rev. 1.0C0\n" ; 
    n_char *szHeader = 
        "\n\ninputNum, arctanX, [atan( x ), ] counter\n" ; 
    n_char szDebug[100] ; 
    n_char *szFormat1 ;
    n_char *szFormat2 ; 
#endif /* BMDEBUG */
    n_char szDataSize[40] ; 
    /* Input test data table looped */
    n_int isTableLooped = false ;
    static n_int i1 ;
    static n_int i2 ;
    static n_int i3 ; 
    /* The input test data must be domain-limited */
    static varsize inputTan1 ;
    static varsize inputTan2 ;
    static varsize inputTan3 ;
    /* If the domain is limited, input is inverted */
    static n_int inverted1 ;
    static n_int inverted2 ;
    static n_int inverted3 ;
    /* Powers of 'x^2', terms of polynomial */
    static varsize polyX1[9] ;
    static varsize polyX2[9] ;
    static varsize polyX3[9] ;
    /* Polynomial numerator */
    static varsize P1 ;
    static varsize P2 ;
    static varsize P3 ;
    /* Polynomial denominator */
    static varsize Q1 ;
    static varsize Q2 ;
    static varsize Q3 ;
    /* The 'arctan( x )' result */
    static varsize arctanX1 ;
    static varsize arctanX2 ;
    static varsize arctanX3 ;

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
    /* Set size of output file (2K) */    
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
        ( DATA_SIZE ? "DOUBLE" : "FLOAT" ) ) ; 
#else
    szDataSize[0] = (n_char)( '0' + DATA_SIZE ) ; 
    szDataSize[1] = '\0' ; 
#endif /* BMDEBUG */

    /* Initialize the test data -- stimuli for the algorithm. */    

    if( !GetTestData() )
    {
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
            __FILE__, __LINE__ ) ; 
    }

      tcdef->CRC = 0;
    th_signal_start() ;

#if BMDEBUG
    DebugOut( szTitle ) ;    /* Print the title message in RAM file */
    DebugOut( szDataSize ) ; /* ...and the data size */
    DebugOut( szHeader ) ;   /* Print the test output file hdr in RAM file */
#endif /* BMDEBUG */

    /*
     * This is the actual benchmark algorithm.
     *
     */    

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

        /* Gets 'signal_in' value from test data*/
        isTableLooped = GetInputValues() ;
        
#if BMDEBUG
#if DATA_SIZE == 0  /* Change format, depending on data size */
        szFormat1 = "%8.7f" ; 
#else
        szFormat1 = "%15.14f" ; 
#endif  /* DATA_SIZE */
        th_sprintf( szDebug, szFormat1, inputNum ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* First, we want to limit the input domain.
         * This can be done by observing that the tan function, 
         * and therefore the arctan function, repeats over the range
         * from 0 to PI/2 in all of the other quadrants( just need to
         * change the sign of the result ).  So, if you can calculate
         * 'arctan' for the interval 0..PI/2, then you have 'arctan'
         * for all 'x'.
         *
         * Next observation, you can limit the domain to 0..PI/4, 
         * because for the interval from PI/4..PI/2, tan is simply
         * the inverse : 
         *
         *        tan( x )= 1/tan( x-PI/4 ), for x = PI/4..PI/2
         *
         */    

        /* Limit the domain to 0..tan( PI/4 ), and remember you did that */

        /* Of course, tan( PI/4 )= 1 */
        if( inputNum > 1 )
        {
            /* ...so if input is >1, then invert it */
            inputTan1 = 1 / inputNum ;
            /* ...and remember to adjust the range later */
            inverted1 = true ;
        }
        else 
        {        
            inverted1 = false ;
            inputTan1 = inputNum ; 
        }

        /* We take the input variable 'x', and calculate 'arctan( x )'
         * from a telescoping series : 
         *
         *        arctan( x )= x * P( x^2 )/ Q( x^2 ), 
         *
         * where P and Q are polynomials, and x is in the range from
         * 0 to tan( pi/4 )( i.e., tan( 0 )..tan( 45 )degrees ).
         *
         */    

        /* First, we need powers of x^2 */    
        polyX1[1] = inputTan1 * inputTan1 ; 

        for( i1 = 2 ; i1 <= 8 ; i1++ )
        {
            polyX1[i1] = polyX1[i1 - 1] * polyX1[1] ; 
        }

        /* Next, calculate the terms of numerator, polynomial P */    
        P1 = (varsize)constantP[0] ; 

        for( i1 = 1 ; i1 <= 8 ; i1++ )
        {
            P1 += (varsize)constantP[i1] * polyX1[i1] ; 
        }

        /* Now, calculate the terms of denominator, polynomial Q */    
        Q1 = (varsize)constantQ[0] ; 

        for( i1 = 1 ; i1 <= 8 ; i1++ )
        {
            Q1 += (varsize)constantQ[i1] * polyX1[i1] ; 
        }

        /* Finally, put it all together for 'arctan( x )' */    
        arctanX1 = inputTan1 * P1 / Q1 ; 

        /* Domain might have been limited */
        if( inverted1 )
        {
            /* ...so this is interval PI/4..PI/2 */
            arctanX1 = ( PI / 2 ) - arctanX1 ;
        }

#if BMDEBUG

#if DATA_SIZE == 0
        szFormat2 = ", %8.7f\n" ; 
#else
        szFormat2 = ", %15.14f\n" ; 
#endif  /* DATA_SIZE */

        th_sprintf( szDebug, szFormat2, (double)arctanX1 ) ; 
        DebugOut( szDebug ) ; 

#if ATAN_SUPPORT == 1
        /* Show platform's result for it's own math library */
        th_sprintf( szDebug, szFormat2, atan( inputNum ) ) ; 
        DebugOut( szDebug ) ; 
#endif /* ATAN_SUPPORT */    

#else /* not BMDEBUG */
        WriteOut( arctanX1 ) ; 

#if ATAN_SUPPORT == 1  
        /* Show platform's result for it's own math library */
        arctanX1 = atan( inputNum ) ; 
        WriteOut( arctanX1 ) ; 
#endif /* ATAN_SUPPORT */    

#endif /* BMDEBUG */

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/

        isTableLooped += GetInputValues() ;
        
#if BMDEBUG
#if DATA_SIZE == 0  /* Change format, depending on data size */
        szFormat1 = "%8.7f" ; 
#else
        szFormat1 = "%15.14f" ; 
#endif  /* DATA_SIZE */
        th_sprintf( szDebug, szFormat1, inputNum ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Limit the domain to 0..tan( PI/4 ), and remember you did that */

        /* Of course, tan( PI/4 )= 1 */
        if( inputNum > 1 )
        {
            /* ...so if input is >1, then invert it */
            inputTan2 = 1 / inputNum ;
            /* ...and remember to adjust the range later */
            inverted2 = true ;
        }
        else 
        {        
            inverted2 = false ;
            inputTan2 = inputNum ; 
        }

        /* First, we need powers of x^2 */    
        polyX2[1] = inputTan2 * inputTan2 ; 

        for( i2 = 2 ; i2 <= 8 ; i2++ )
        {
            polyX2[i2] = polyX2[i2 - 1] * polyX2[1] ; 
        }

        /* Next, calculate the terms of numerator, polynomial P */    
        P2 = (varsize)constantP[0] ; 

        for( i2 = 1 ; i2 <= 8 ; i2++ )
        {
            P2 += (varsize)constantP[i2] * polyX2[i2] ; 
        }

        /* Now, calculate the terms of denominator, polynomial Q */    
        Q2 = (varsize)constantQ[0] ; 

        for( i2 = 1 ; i2 <= 8 ; i2++ )
        {
            Q2 += (varsize)constantQ[i2] * polyX2[i2] ; 
        }

        /* Finally, put it all together for 'arctan( x )' */    
        arctanX2 = inputTan2 * P2 / Q2 ; 

        /* Domain might have been limited */
        if( inverted2 )
        {
            /* ...so this is interval PI/4..PI/2 */
            arctanX2 = ( PI / 2 ) - arctanX2 ;
        }

#if BMDEBUG

#if DATA_SIZE == 0
        szFormat2 = ", %8.7f\n" ; 
#else
        szFormat2 = ", %15.14f\n" ; 
#endif  /* DATA_SIZE */

        th_sprintf( szDebug, szFormat2, (double)arctanX2 ) ; 
        DebugOut( szDebug ) ; 

#if ATAN_SUPPORT == 1
        /* Show platform's result for it's own math library */
        th_sprintf( szDebug, szFormat2, atan( inputNum ) ) ; 
        DebugOut( szDebug ) ; 
#endif /* ATAN_SUPPORT */    

#else /* not BMDEBUG */
        WriteOut( arctanX2 ) ; 

#if ATAN_SUPPORT == 1  
        /* Show platform's result for it's own math library */
        arctanX2 = atan( inputNum ) ; 
        WriteOut( arctanX2 ) ; 
#endif /* ATAN_SUPPORT */    

#endif /* BMDEBUG */

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/

        isTableLooped += GetInputValues() ;
        
#if BMDEBUG
#if DATA_SIZE == 0  /* Change format, depending on data size */
        szFormat1 = "%8.7f" ; 
#else
        szFormat1 = "%15.14f" ; 
#endif  /* DATA_SIZE */
        th_sprintf( szDebug, szFormat1, inputNum ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Limit the domain to 0..tan( PI/4 ), and remember you did that */

        /* Of course, tan( PI/4 )= 1 */
        if( inputNum > 1 )
        {
            /* ...so if input is >1, then invert it */
            inputTan3 = 1 / inputNum ;
            /* ...and remember to adjust the range later */
            inverted3 = true ;
        }
        else 
        {        
            inverted3 = false ;
            inputTan3 = inputNum ; 
        }

        /* First, we need powers of x^2 */    
        polyX3[1] = inputTan3 * inputTan3 ; 

        for( i3 = 2 ; i3 <= 8 ; i3++ )
        {
            polyX3[i3] = polyX3[i3 - 1] * polyX3[1] ; 
        }

        /* Next, calculate the terms of numerator, polynomial P */    
        P3 = (varsize)constantP[0] ; 

        for( i3 = 1 ; i3 <= 8 ; i3++ )
        {
            P3 += (varsize)constantP[i3] * polyX3[i3] ; 
        }

        /* Now, calculate the terms of denominator, polynomial Q */    
        Q3 = (varsize)constantQ[0] ; 

        for( i3 = 1 ; i3 <= 8 ; i3++ )
        {
            Q3 += (varsize)constantQ[i3] * polyX3[i3] ; 
        }

        /* Finally, put it all together for 'arctan( x )' */    
        arctanX3 = inputTan3 * P3 / Q3 ; 

        /* Domain might have been limited */
        if( inverted3 )
        {
            /* ...so this is interval PI/4..PI/2 */
            arctanX3 = ( PI / 2 ) - arctanX3 ;
        }

#if BMDEBUG

#if DATA_SIZE == 0
        szFormat2 = ", %8.7f\n" ; 
#else
        szFormat2 = ", %15.14f\n" ; 
#endif  /* DATA_SIZE */

        th_sprintf( szDebug, szFormat2, (double)arctanX3 ) ; 
        DebugOut( szDebug ) ; 

#if ATAN_SUPPORT == 1
        /* Show platform's result for it's own math library */
        th_sprintf( szDebug, szFormat2, atan( inputNum ) ) ; 
        DebugOut( szDebug ) ; 
#endif /* ATAN_SUPPORT */    

#else /* not BMDEBUG */
        WriteOut( arctanX3 ) ; 

#if ATAN_SUPPORT == 1  
        /* Show platform's result for it's own math library */
        arctanX3 = atan( inputNum ) ; 
        WriteOut( arctanX3 ) ; 
#endif /* ATAN_SUPPORT */    

#endif /* BMDEBUG */

#if BMDEBUG
        th_sprintf( szDebug, ", %08lX\n", (n_ulong)loop_cnt ) ; 
        DebugOut( szDebug ) ; 
#else
#if DATA_SIZE == 0
        i3 = (n_int)( loop_cnt &0x0000FFFF ) ; 
        WriteOut( i3 ) ;
        i3 = (n_int)( loop_cnt >> 16 ) ; 
        WriteOut( i3 ) ; 
#else
        WriteOut( loop_cnt ) ; 
#endif /* DATA_SIZE */

#endif /* BMDEBUG */
        i3 = 0xAAAA ; 
        WriteOut( i3 ) ;

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
/* NON_INTRUSIVE_CRC_CHECK ok */
#if NON_INTRUSIVE_CRC_CHECK
	tcdef->CRC=0;
	tcdef->CRC = Calc_crc32((e_u32)arctanX1,tcdef->CRC);
	tcdef->CRC = Calc_crc32((e_u32)arctanX2,tcdef->CRC);
	tcdef->CRC = Calc_crc32((e_u32)arctanX3,tcdef->CRC);
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
    init_platform();
    /* target specific inititialization */
    al_main(argc, argv);
    xil_printf(">>     Start of BaseFP...\n\r");
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
            th_free(inpVariableFree);
            xil_printf("%20d\n\r",benchIter++);
            
        }
    }
    xil_printf(">>      BaseFP test is finished\n\r");
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
            Xil_DCacheFlush();
            corrupt_tcm(TmpBufPtr[0],TmpBufPtr[1],TmpBufPtr[2]);
            xil_printf("TCM addr corrupted: 0x%x\r\n, Bit Mask : 0x%x ,Shift %d, bits corrupted: %x\r\n",TmpBufPtr[0],TmpBufPtr[1],TmpBufPtr[2],TmpBufPtr[3]);

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
