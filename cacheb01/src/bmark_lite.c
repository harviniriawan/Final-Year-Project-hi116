/*
 * bmark_lite.c
 *
 *  Created on: 12 Nov 2019
 *      Author: harvi
 */
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
 *          $Date: 2002/08/07 22:21:04 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/cacheb01/bmark_lite.c,v $
 *
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.15  2002/08/07 22:21:04  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.14  2002/07/22 21:59:20  rick
 * General cleanup Beta 2b
 *
 * Revision 1.13  2002/07/22 16:10:10  rick
 * Fix iteration dependant NI CRC
 *
 * Revision 1.12  2002/07/19 23:10:22  rick
 * Fix iteration dependant NI CRC's
 *
 * Revision 1.11  2002/07/18 23:54:42  rick
 * Fix iteration dependencies in NI CRC
 *
 * Revision 1.10  2002/07/10 19:01:06  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.9  2002/05/29 22:25:01  rick
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
 * Revision 1.5  2002/04/10 19:36:58  rick
 * Fixes to reduce Lite vs. Regular variances in timing
 *
 * Revision 1.4  2002/03/11 22:11:47  rick
 * ITERATIONS, CRC_CHECK, NON_INTRUSIVE TCDef Usage
 *
 * Revision 1.3  2002/02/25 17:15:32  rick
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
#define ITERATIONS 100000	/* required iterations for crc */
#else
#define ITERATIONS 100000	/* recommended iterations for benchmark */
#endif
#endif

#if CRC_CHECK
#define EXPECTED_CRC	0x9b6b
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC	0x0057
#else
#define EXPECTED_CRC	0x0000
#endif

/*******************************************************************************
    Local Data
*******************************************************************************/

TCDef the_tcdef =
{
    "AUT cacheb01   ",
    EEMBC_MEMBER_COMPANY,
    EEMBC_PROCESSOR,
    EEMBC_TARGET,
    "Algorithm #13 --  Cache Buster  V1.0A0 - cacheb01",
    TCDEF_REVISION,
    { EEMBC_TH_MAJOR, EEMBC_TH_MINOR, EEMBC_TH_STEP, EEMBC_TH_REVISION },
    { 0, 0, 0, 0 },   /* Target Hardware Version Number Required( make all zeros to ignore )*/
    { 1, 0, 'A', 0 }, /* The Version number of this Benchmark */
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

n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   failTest;
/* varsize is 32 or 16 bits */
varsize *inputToken ;   /* Pointer to array of input tokens */

/* Table of pointers to source/destination arrays in RAM */
extern varsize *dataPtrTable[] ;
/* Table of pointer to functions */
extern funcPtr funcPtrTable[] ;

varsize *array1 ;       /* Data source or destination in RAM */
varsize *array2 ;       /* Data source or destination in RAM */
varsize *array3 ;       /* Data source or destination in RAM */
varsize *array4 ;       /* Data source or destination in RAM */
varsize *array5 ;       /* Data source or destination in RAM */
varsize *array6 ;       /* Data source or destination in RAM */
varsize *array7 ;       /* Data source or destination in RAM */
varsize *array8 ;       /* Data source or destination in RAM */

varsize func ;          /* Index to selected function */

/* Indicies to source/destination arrays in RAM */
varsize arg1 ;
varsize arg2 ;
varsize arg3 ;
varsize arg4 ;
varsize arg5 ;
varsize arg6 ;

varsize *array1Free;
varsize *array2Free;
varsize *array3Free;
varsize *array4Free;
varsize *array5Free;
varsize *array6Free;
varsize *array7Free;
varsize *array8Free;

n_int   *RAMfileFree;

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
        "\nEEMBC Automotive/Industrial Subcommittee Benchmarks (c)1998-1999\n"
        "Algorithm 13 :  Cache Buster  Rev. 1.0A0 - cacheb01\n" ;
    n_char *szHeader =
        "\n\n, { loop counter }\n" ;
    n_char szDebug[100] ;
#endif /* BMDEBUG */
    n_char szDataSize[40] ;
    /* Input test data table looped */
    n_int isTableLooped = false ;
    n_int i ;
    /* Pointer to executable function */
    funcPtr funcIndirect ;

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
    /* Set size of output file( 1K )*/
    RAMfileSize = MAX_FILESIZE;

    /* Allocate some RAM for output file */
    RAMfile         = (n_int *)th_malloc( RAMfileSize * sizeof(n_int) + sizeof (varsize) ) ;
    /* NOT PART OF BENCHMARK TO FREE RAMfile */
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
    /*
     * Allocate some RAM for source/destination RAM arrays
     * and put array pointers in the table.
     *
     */
    dataPtrTable[0] = array1 = (varsize*)
        th_malloc( ARRAY_SIZE *sizeof( varsize ) ) ;
    dataPtrTable[1] = array2 = (varsize*)
        th_malloc( ARRAY_SIZE *sizeof( varsize ) ) ;
    dataPtrTable[2] = array3 = (varsize*)
        th_malloc( ARRAY_SIZE *sizeof( varsize ) ) ;
    dataPtrTable[3] = array4 = (varsize*)
        th_malloc( ARRAY_SIZE *sizeof( varsize ) ) ;
    dataPtrTable[4] = array5 = (varsize*)
        th_malloc( ARRAY_SIZE *sizeof( varsize ) ) ;
    dataPtrTable[5] = array6 = (varsize*)
        th_malloc( ARRAY_SIZE *sizeof( varsize ) ) ;
    dataPtrTable[6] = array7 = (varsize*)
        th_malloc( ARRAY_SIZE *sizeof( varsize ) ) ;
    dataPtrTable[7] = array8 = (varsize*)
        th_malloc( ARRAY_SIZE *sizeof( varsize ) ) ;

    /* Duplicate some of the array pointers */

    dataPtrTable[8]  = array1 ;
    dataPtrTable[9]  = array1 ;
    dataPtrTable[10] = array1 ;
    dataPtrTable[11] = array1 ;
    dataPtrTable[12] = array1 ;
    dataPtrTable[13] = array1 ;
    dataPtrTable[14] = array1 ;
    dataPtrTable[15] = array1 ;

    /*assign malloced pointer to another arrayxFree NOT PART OF BENCHMARK*/
    array1Free = array1;
    array2Free = array2;
    array3Free = array3;
    array4Free = array4;
    array5Free = array5;
    array6Free = array6;
    array7Free = array7;
    array8Free = array8;


    if( ( array1 == NULL ) || ( array2 == NULL ) || ( array3 == NULL ) ||
        ( array4 == NULL ) || ( array5 == NULL ) || ( array6 == NULL ) ||
        ( array7 == NULL ) || ( array8 == NULL ) )
    {
        th_exit( THE_OUT_OF_MEMORY,
            "Cannot Source/Destination Data Array %s : %d",
            __FILE__, __LINE__ ) ;
    }

    /* Fill source/destination RAM arrays with meaningless data */
    for( i = 0 ; i < ARRAY_SIZE ; i++ )
    {
        *array1++ = (varsize)( i + 0xAAAA ) ;
        *array2++ = (varsize)( i + 0x5555 ) ;
        *array3++ = (varsize)( i + 0xAA55 ) ;
        *array4++ = (varsize)( i + 0x55AA ) ;
        *array5++ = (varsize)( i + 0x5A5A ) ;
        *array6++ = (varsize)( i + 0xA5A5 ) ;
        *array7++ = (varsize)( i + 0x1234 ) ;
        *array8++ = (varsize)( i + 0x4321 ) ;
    }

    /* Tell us the compiled data size */
#if BMDEBUG
    /* th_sprintf( szDataSize, "Data size = %s\n\n",
         ( DATA_SIZE ? "LONG" : "SHORT" ) ) ; */
    xil_printf("Data size = %s\n\r",
         ( DATA_SIZE ? "LONG" : "SHORT" ) ) ;
#else
    szDataSize[0] = (n_char)( DATA_SIZE ? 'L' : 'S' ) ;
    szDataSize[1] = '\0' ;
#endif /* BMDEBUG */

    /* Initialize the test data -- stimuli for the algorithm. */

    /* Allocate for the test input data table */
    if( !GetTestData() )
    {
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
            __FILE__, __LINE__ ) ;
    }

      tcdef->CRC = 0;
    th_signal_start() ;

#if BMDEBUG
    DebugOut( szTitle ) ;    /* Print the title message in RAM file */
    DebugOut( szHeader ) ;   /* Print the test output file hdr in RAM file */
    DebugOut( szDataSize );
    /*DebugOut( "Data size = %s\n\n",
         ( DATA_SIZE ? "LONG" : "SHORT" ) ) ; */ /* ...and the data size */
#endif /* BMDEBUG */

    /*
     * This is the actual benchmark algorithm.
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

        /* Gets 'realData' and 'imagData' values from test data */
        /* Gets function (pointer) and arguments(arg1-arg6) */
        isTableLooped = GetInputValues() ;

        /* Get pointer to the selected function */
        funcIndirect = funcPtrTable[func] ;

        funcIndirect() ;    /* Execute the selected function */

#if BMDEBUG
        /* th_sprintf( szDebug, "%8ld, %8ld, %8ld, %8ld, %8ld, %8ld, %8ld\n",
             (n_long)func, (n_long)arg1, (n_long)arg2, (n_long)arg3, (n_long)arg4,
             (n_long)arg5, (n_long)arg6 ) ; */

        xil_printf("Func, arg1-6 : %8ld, %8ld, %8ld, %8ld, %8ld, %8ld, %8ld\n\r",
            (n_long)func, (n_long)arg1, (n_long)arg2, (n_long)arg3, (n_long)arg4,
            (n_long)arg5, (n_long)arg6 ) ;
        DebugOut( szDebug ) ;
#else
        WriteOut( func ) ;
        WriteOut( arg1 ) ;
        WriteOut( arg2 ) ;
        WriteOut( arg3 ) ;
        WriteOut( arg4 ) ;
        WriteOut( arg5 ) ;
        WriteOut( arg6 ) ;
#endif

#if BMDEBUG
        /* th_sprintf( szDebug, "{ %08lX }\n", (n_long)loop_cnt ) ; */
        xil_printf("Loop number : { %08lX }\n\r", (n_long)loop_cnt ) ;
        /*DebugOut( "{ %08lX }\n", (n_long)loop_cnt  ) ; */
#else

#if DATA_SIZE == 0
        i = (varsize)( loop_cnt & 0x0000FFFF ) ;
        WriteOut( i ) ;
        i = (varsize)( loop_cnt >> 16 ) ;
        WriteOut( i ) ;
#else
        WriteOut(loop_cnt ) ;
#endif
        i = 0xAAAA ;
        WriteOut( i ) ;
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
/*CRC_CHECK OK */
#if		NON_INTRUSIVE_CRC_CHECK
/* Final results are iteration dependent */
	tcdef->CRC=0;
	for (loop_cnt=0;loop_cnt<NUM_TESTS;loop_cnt++){
        /*xil_printf("%8ld",inputToken[loop_cnt]);*/
		tcdef->CRC = Calc_crc32((e_u32)inputToken[loop_cnt],tcdef->CRC);
    }
#elif	CRC_CHECK
/* Iteration dependant results check */
	tcdef->CRC=0;
	tcdef->CRC = Calc_crc32((e_u32)func,tcdef->CRC ) ;
	tcdef->CRC = Calc_crc32((e_u32)arg1,tcdef->CRC ) ;
	tcdef->CRC = Calc_crc32((e_u32)arg2,tcdef->CRC ) ;
	tcdef->CRC = Calc_crc32((e_u32)arg3,tcdef->CRC ) ;
	tcdef->CRC = Calc_crc32((e_u32)arg4,tcdef->CRC ) ;
	tcdef->CRC = Calc_crc32((e_u32)arg5,tcdef->CRC ) ;
	tcdef->CRC = Calc_crc32((e_u32)arg6,tcdef->CRC ) ;
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
    /* Unused */
    argc = argc ;
    argv = argv ;
	/* target specific inititialization */
	al_main(argc, argv);
    xil_printf(">>     Start of Cachebuster...\n\r");

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
                th_free(inputToken);
                th_free(array8Free);
                th_free(array7Free);
                th_free(array6Free);
                th_free(array5Free);
                th_free(array4Free);
                th_free(array3Free);
                th_free(array2Free);
                th_free(array1Free);
                th_free(RAMfileFree);
                xil_printf("%20d\n\r",benchIter++);
			
		}
    }
    xil_printf(">>     Cachebuster test is finished\n\r");
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
            xil_printf("Ram file size has exceeded!\n\r");
            RAMfilePtr = RAMfile;
    }
#else
    /*th_printf( szSrc ) ;*/
    xil_printf( szSrc ) ;
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

n_void
function1( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg4 ] ;
    source2 = dataPtrTable[ arg5 ] ;
    dest    = dataPtrTable[ arg6 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg3 ] = source1[ arg1 ] + source2[ arg2 ] ;

} /* End of 'function1' */


n_void
function2( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg4 ] ;
    source2 = dataPtrTable[ arg5 ] ;
    dest    = dataPtrTable[ arg6 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg3 ] = source1[ arg1 ] - source2[ arg2 ] ;

} /* End of 'function2' */

n_void
function3( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg4 ] ;
    source2 = dataPtrTable[ arg5 ] ;
    dest    = dataPtrTable[ arg6 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg3 ] = source1[ arg1 ] | source2[ arg2 ] ;

} /* End of 'function3' */

n_void
function4( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg4 ] ;
    source2 = dataPtrTable[ arg5 ] ;
    dest    = dataPtrTable[ arg6 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg3 ] = source1[ arg1 ] & source2[ arg2 ] ;

} /* End of 'function4' */

n_void
function5( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg1 ] ;
    source2 = dataPtrTable[ arg2 ] ;
    dest    = dataPtrTable[ arg3 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg6 ] = source1[ arg4 ] + source2[ arg5 ] ;

} /* End of 'function5' */

n_void
function6( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg1 ] ;
    source2 = dataPtrTable[ arg2 ] ;
    dest    = dataPtrTable[ arg3 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg6 ] = source1[ arg4 ] - source2[ arg5 ] ;

} /* End of 'function6' */

n_void
function7( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg1 ] ;
    source2 = dataPtrTable[ arg2 ] ;
    dest    = dataPtrTable[ arg3 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg6 ] = source1[ arg4 ] ^ source2[ arg5 ] ;

} /* End of 'function7' */

n_void
function8( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg1 ] ;
    source2 = dataPtrTable[ arg2 ] ;
    dest    = dataPtrTable[ arg3 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg6 ] = source1[ arg4 ] & source2[ arg5 ] ;

} /* End of 'function8' */

n_void
function9( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg3 ] ;
    source2 = dataPtrTable[ arg5 ] ;
    dest    = dataPtrTable[ arg6 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg4 ] = source1[ arg1 ] + source2[ arg2 ] ;

} /* End of 'function9' */

n_void
function10( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg3 ] ;
    source2 = dataPtrTable[ arg5 ] ;
    dest    = dataPtrTable[ arg6 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg4 ] = source1[ arg1 ] - source2[ arg2 ] ;

} /* End of 'function10' */

n_void
function11( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg3 ] ;
    source2 = dataPtrTable[ arg5 ] ;
    dest    = dataPtrTable[ arg6 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg4 ] = source1[ arg1 ] | source2[ arg2 ] ;

} /* End of 'function11' */

n_void
function12( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg3 ] ;
    source2 = dataPtrTable[ arg5 ] ;
    dest    = dataPtrTable[ arg6 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg4 ] = source1[ arg1 ] ^ source2[ arg2 ] ;

} /* End of 'function12' */

n_void
function13( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg1 ] ;
    source2 = dataPtrTable[ arg2 ] ;
    dest    = dataPtrTable[ arg4 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg6 ] = source1[ arg3 ] + source2[ arg5 ] ;

} /* End of 'function13' */

n_void
function14( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg1 ] ;
    source2 = dataPtrTable[ arg2 ] ;
    dest    = dataPtrTable[ arg4 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg6 ] = source1[ arg3 ] - source2[ arg5 ] ;

} /* End of 'function14' */

n_void
function15( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg1 ] ;
    source2 = dataPtrTable[ arg2 ] ;
    dest    = dataPtrTable[ arg4 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg6 ] = source1[ arg3 ] & source2[ arg5 ] ;

}    /* End of 'function15' */

n_void
function16( n_void )
{
    static varsize *source1 ;
    static varsize *source2 ;
    static varsize *dest ;
    /* Point to the tables which contain the arguments */
    source1 = dataPtrTable[ arg1 ] ;
    source2 = dataPtrTable[ arg2 ] ;
    dest    = dataPtrTable[ arg4 ] ;
    /*
     * Perform the arithmetic on the source arguments,
     * putting result in destination
     *
     */
    dest[ arg6 ] = source1[ arg3 ] | source2[ arg5 ] ;

} /* End of 'function16' */

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
