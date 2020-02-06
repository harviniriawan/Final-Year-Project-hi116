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
 *          $Date: 2002/08/07 22:20:41 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/aiifft01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.15  2002/08/07 22:20:41  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.14  2002/07/22 21:59:12  rick
 * General cleanup Beta 2b
 *
 * Revision 1.13  2002/07/22 16:10:04  rick
 * Fix iteration dependant NI CRC
 *
 * Revision 1.12  2002/07/18 23:33:49  rick
 * Fix iteration dependencies in NI CRC
 *
 * Revision 1.11  2002/07/10 19:00:50  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.10  2002/05/29 22:24:53  rick
 * Set recommended iterations with make
 *
 * Revision 1.9  2002/05/10 23:57:45  rick
 * Fix missing shifts in 16, and 32 bit CRC calculations
 *
 * Revision 1.8  2002/05/10 17:20:36  rick
 * Add al_main to API
 *
 * Revision 1.7  2002/04/25 20:10:43  rick
 * sprintf to th_sprintf
 *
 * Revision 1.6  2002/04/10 19:53:19  rick
 * Fix CRC error
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
 *   5/14/99 pjt     1.0
 *       Initial cleanup
 *   7/4/99  arw     1.0
 *       Added eembc_dt.h datatypes & additional cleanup
 *       "thlib.h" will #include eembc_dt.h for us
 *	7/13/99 arw		1.0
 *		Corrected WriteOut() and 
 *		Changed variable names starting with debugOut* to RAMfile*
 *		Changed various declarations
 *   5/29/01 TC 
 *       Removed all debug related code to debug.c
 *       Removed all intializatio and variable declaration to init.c
 *       Added all extern in algo.h
 *       Removed 'varsize', mostly replaced with n_int 
 *       Combined THResult structure and TCDef structure into one	
 *       Added CRC in TCDef structure - Calculates running CRC of all the
 *       intermediate value
 *       added #if CRC_CHECK, so that number of iterations can be changed
 *       In order to match CRC, you have to use pre-defined number of iterations
 *       Removed all dynamic memory allocation
 *   7/24/01 TC/arw  8/16 Bit Benchmark Suite (V 2.0)
 *       Added optional compile define for all CRC operation with CRC_CHECK.
 *       turn on CRC operation by setting #define CRC_CHECK	TRUE
 *       turn off CRC operation by setting #define CRC_CHECK FALSE 
 *       define for CRC_CHECK is in th\al\thcfg.h file
 *       Please use recommended # of iterations when CRC_CHECK is enabled
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

#define ALGO_GLOBALS    1   /* Next time, we'll skip these */
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
#define ITERATIONS 200	/* required iterations for crc */
#else
#define ITERATIONS 200	/* recommended iterations for benchmark */
#endif
#endif

#if CRC_CHECK
#define EXPECTED_CRC	0x0000
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC	0x7c4c
#else
#define EXPECTED_CRC	0x0000
#endif

TCDef the_tcdef = 
{
    "AUT aiifft01   ",
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Alg #17 -- Inverse Fast Fourier Transform V1.0A0 - aiifft01", 
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
n_int   RAMfileFree; /* NOT PART OF BENCHMARK to free RAMfile */

varsize *inpSignal ;  /* Pointer to array of input signal data values */

varsize realData_1[NUM_POINTS] ;    /* Points to real part of data */
varsize realData_2[NUM_POINTS] ;    /* Points to real part of data */
varsize realData_3[NUM_POINTS] ;    /* Points to real part of data */
varsize imagData_1[NUM_POINTS] ;    /* Points to imaginary part of data */
varsize imagData_2[NUM_POINTS] ;    /* Points to imaginary part of data */
varsize imagData_3[NUM_POINTS] ;    /* Points to imaginary part of data */


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
        "Algorithm 17 :  Inverse Fast Fourier Transform  Rev. 1.0A0 - aiifft01\n" ; 
    char *szHeader = 
        "\n\nrealData, imagData, ( input signal, FFT, power & phase ), "
        "[pass count...], { loop counter }\n" ; 
    char szDebug[100] ; 
#endif /* BMDEBUG */
    n_char szDataSize[40] ; 
    /* Input test data table looped */
    n_int isTableLooped = false ;
    n_int bitRevInd[NUM_POINTS] ; 
    double trigArg ; 
    n_int index ;
    n_int brIndex ; 
    static varsize sineV[NUM_POINTS / 2] ;
    static varsize cosineV[NUM_POINTS / 2] ; 
    static varsize realBitRevData_1[NUM_POINTS] ;
    static varsize imagBitRevData_1[NUM_POINTS] ; 
    static varsize realBitRevData_2[NUM_POINTS] ;
    static varsize imagBitRevData_2[NUM_POINTS] ; 
    static varsize realBitRevData_3[NUM_POINTS] ;
    static varsize imagBitRevData_3[NUM_POINTS] ; 
    static varsize wReal_1 ;
    static varsize wImag_1 ;
    static varsize tRealData_1 ;
    static varsize tImagData_1 ; 
    static varsize wReal_2 ;
    static varsize wImag_2 ;
    static varsize tRealData_2 ;
    static varsize tImagData_2 ; 
    static varsize wReal_3 ;
    static varsize wImag_3 ;
    static varsize tRealData_3 ;
    static varsize tImagData_3 ; 
    static varsize *realLow_1 ;
    static varsize *imagLow_1 ;
    static varsize *realHi_1 ;
    static varsize *imagHi_1 ; 
    static varsize *realLow_2 ;
    static varsize *imagLow_2 ;
    static varsize *realHi_2 ;
    static varsize *imagHi_2 ; 
    static varsize *realLow_3 ;
    static varsize *imagLow_3 ;
    static varsize *realHi_3 ;
    static varsize *imagHi_3 ; 
    static n_long argIndex_1 ;
    static n_long deltaIndex_1 ; 
    static n_long argIndex_2 ;
    static n_long deltaIndex_2 ; 
    static n_long argIndex_3 ;
    static n_long deltaIndex_3 ; 
    static varsize n1_1 ;
    static varsize n2_1 ;
    static varsize l_1 ; 
    static varsize n1_2 ;
    static varsize n2_2 ;
    static varsize l_2 ; 
    static varsize n1_3 ;
    static varsize n2_3 ;
    static varsize l_3 ; 
    static varsize i_1 ;
    static varsize j_1 ;
    static varsize k_1 ;
    static varsize passCount_1 ; 
    static varsize i_2 ;
    static varsize j_2 ;
    static varsize k_2 ; 
    static varsize passCount_2 ; 
    static varsize i_3 ;
    static varsize j_3 ;
    static varsize k_3 ;
    static varsize passCount_3 ; 

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

    /* Populate the sine & cosine tables -- used by all instances */    
    for( i_1 = 0 ; i_1 < ( NUM_POINTS / 2 ) ; i_1++ )
    {        
        trigArg = (double)( i_1 * PI / ( NUM_POINTS / 2 ) ) ; 

        if( sin( trigArg ) == 1.0 )
        {
            sineV[i_1] = (n_long)( TRIG_SCALE_FACTOR - 1 ) ; 
        }
        else
        {

            sineV[i_1] = (n_long)( sin( trigArg ) * TRIG_SCALE_FACTOR ) ; 
        }

        if( cos( trigArg ) == 1.0 )
        {
            cosineV[i_1] = (n_long)( TRIG_SCALE_FACTOR - 1 ) ; 
        }
        else
        {
            cosineV[i_1] = (n_long)( cos( trigArg ) * TRIG_SCALE_FACTOR ) ; 
        }
    }

    /* Compute the bit reversal indicies  -- used by all the instances */    
    for( i_1 = 0 ; i_1 < NUM_POINTS ; i_1++ )
    {        
        index = i_1 ; 
        brIndex = 0 ; 
        for( j_1 = 0 ; j_1 < FFT_LENGTH ; j_1++ )
        {        
            brIndex <<= 1 ; 
            if( 0x01 &index )
            {
                brIndex |= 0x01 ; 
            }
            index >>= 1 ; 
        }
        bitRevInd[i_1] = brIndex ; 
    }

    /* Tell the host that the test has begun */
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
     * Inverse FFT processing in the automotive area might be applied in audio
     * signal processing, specifically noise-cancellation processes.
     *
     * Compute the Radix 2 Decimation in Frequency iFFT on the complex input
     * values stored in the 'realData' and 'imagData' arrays.  Converts
     * frequency-domain data to time-domain.  Builds the sine and cosine
     * twiddle factors prior to execution of the iFFT loop.  Also builds 
     * the bit reversal indicies prior to execution of the loop.
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
        isTableLooped = GetInputValues( realData_1, imagData_1 ) ;

#if BMDEBUG
        DebugOut( "Input signal : \n" ) ; 
        for( i_1 = 0 ; i_1 < NUM_POINTS ; i_1++ )
        { 
            th_sprintf( szDebug, "%8ld, %8ld\n", realData_1[i_1], 
                imagData_1[i_1] ) ; 
            DebugOut( szDebug ) ; 
        }
#endif /* BMDEBUG */

        /* Bit Reversal */    
        for( i_1 = 0 ; i_1 < NUM_POINTS ; i_1++ )
        {        
            realBitRevData_1[i_1] = realData_1[bitRevInd[i_1]] ; 
            imagBitRevData_1[i_1] = imagData_1[bitRevInd[i_1]] ; 
        }

        /* Return bit reversed data to input arrays */    
        for( i_1 = 0 ; i_1 < NUM_POINTS ; i_1++ )
        {        
            realData_1[i_1] = realBitRevData_1[i_1] ; 
            imagData_1[i_1] = imagBitRevData_1[i_1] ; 
        }

        /* iFFT Computation */    

        /* Step through the stages */    

        for( passCount_1 = 0, k_1 = 1 ; 
             k_1 <= FFT_LENGTH ; 
             k_1++, passCount_1++ )
        {        
            n1_1 = 1 << k_1 ; 
            n2_1 = n1_1 >> 1 ; 

            /* Initialize twiddle factor lookup indicies */    
            argIndex_1 = 0 ; 
            deltaIndex_1 = ( NUM_POINTS / 2 ) / n2_1 ; 

            /* Step through the butterflies */    
            for( j_1 = 0 ; j_1 < n2_1 ; j_1++, passCount_1++ )
            {        
                /* Lookup twiddle factors */    
                wReal_1 = cosineV[argIndex_1] ; 
                /* Note iFFT reversal of sign */
                wImag_1 = -sineV[argIndex_1] ;

                /* Process butterflies with the same twiddle factors */    
                for( i_1 = j_1 ; i_1 < NUM_POINTS ; i_1 += n1_1, passCount_1++ )
                {
                    l_1 = i_1 + n2_1 ; 
                    realLow_1 = &realData_1[l_1] ; 
                    imagLow_1 = &imagData_1[l_1] ; 
                    realHi_1 = &realData_1[i_1] ; 
                    imagHi_1 = &imagData_1[i_1] ; 

                    /* Scale each stage to prevent overflow */    
                    *realLow_1 >>= STAGE_SCALE_FACTOR ; 
                    *imagLow_1 >>= STAGE_SCALE_FACTOR ; 
                    *realHi_1 >>= STAGE_SCALE_FACTOR ; 
                    *imagHi_1 >>= STAGE_SCALE_FACTOR ; 

                    tRealData_1 = *realLow_1 * wReal_1 - *imagLow_1 * wImag_1 ; 
                    tImagData_1 = *imagLow_1 * wReal_1 + *realLow_1 * wImag_1 ; 

                    /* Scale twiddle products to accomodate 32-bit accu. */
                    tRealData_1 >>= BUTTERFLY_SCALE_FACTOR ; 
                    tImagData_1 >>= BUTTERFLY_SCALE_FACTOR ; 

                    realData_1[l_1] = *realHi_1 - tRealData_1 ; 
                    imagData_1[l_1] = *imagHi_1 - tImagData_1 ; 
                    realData_1[i_1] += tRealData_1 ; 
                    imagData_1[i_1] += tImagData_1 ; 
                }
                argIndex_1 += deltaIndex_1 ; 
            }
        } /* End of iFFT loop */

#if BMDEBUG
        th_sprintf( szDebug, "[ %d ]\niFFT output : \n", passCount_1 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        for( i_1 = 0 ; i_1 < NUM_POINTS ; i_1++ )
        {
            /* Display iFFT spectrum */
#if BMDEBUG 
            th_sprintf( szDebug, "%8ld, %8ld\n", (n_long)realData_1[i_1], 
                (n_long)imagData_1[i_1] ) ; 
            DebugOut( szDebug ) ; 
#else
            WriteOut( realData_1[i_1] ) ; 
            WriteOut( imagData_1[i_1] ) ; 
#endif
        }

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/

        /* Gets 'signal_in' value from test data*/
        isTableLooped += GetInputValues( realData_2, imagData_2 ) ;

#if BMDEBUG
        DebugOut( "Input signal : \n" ) ; 
        for( i_2 = 0 ; i_2 < NUM_POINTS ; i_2++ )
        { 
            th_sprintf( szDebug, "%8ld, %8ld\n", realData_2[i_2], 
                imagData_2[i_2] ) ; 
            DebugOut( szDebug ) ; 
        }
#endif /* BMDEBUG */

        /* Bit Reversal */    
        for( i_2 = 0 ; i_2 < NUM_POINTS ; i_2++ )
        {        
            realBitRevData_2[i_2] = realData_2[bitRevInd[i_2]] ; 
            imagBitRevData_2[i_2] = imagData_2[bitRevInd[i_2]] ; 
        }

        /* Return bit reversed data to input arrays */    
        for( i_2 = 0 ; i_2 < NUM_POINTS ; i_2++ )
        {        
            realData_2[i_2] = realBitRevData_2[i_2] ; 
            imagData_2[i_2] = imagBitRevData_2[i_2] ; 
        }

        /* iFFT Computation */    

        /* Step through the stages */    

        for( passCount_2 = 0, k_2 = 1 ; 
             k_2 <= FFT_LENGTH ; 
             k_2++, passCount_2++ )
        {        
            n1_2 = 1 << k_2 ; 
            n2_2 = n1_2 >> 1 ; 

            /* Initialize twiddle factor lookup indicies */    
            argIndex_2 = 0 ; 
            deltaIndex_2 = ( NUM_POINTS / 2 ) / n2_2 ; 

            /* Step through the butterflies */    
            for( j_2 = 0 ; j_2 < n2_2 ; j_2++, passCount_2++ )
            {        
                /* Lookup twiddle factors */    
                wReal_2 = cosineV[argIndex_2] ; 
                /* Note iFFT reversal of sign */
                wImag_2 = -sineV[argIndex_2] ;

                /* Process butterflies with the same twiddle factors */    
                for( i_2 = j_2 ; i_2 < NUM_POINTS ; i_2 += n1_2, passCount_2++ )
                {
                    l_2 = i_2 + n2_2 ; 
                    realLow_2 = &realData_2[l_2] ; 
                    imagLow_2 = &imagData_2[l_2] ; 
                    realHi_2 = &realData_2[i_2] ; 
                    imagHi_2 = &imagData_2[i_2] ; 

                    /* Scale each stage to prevent overflow */    
                    *realLow_2 >>= STAGE_SCALE_FACTOR ; 
                    *imagLow_2 >>= STAGE_SCALE_FACTOR ; 
                    *realHi_2 >>= STAGE_SCALE_FACTOR ; 
                    *imagHi_2 >>= STAGE_SCALE_FACTOR ; 

                    tRealData_2 = *realLow_2 * wReal_2 - *imagLow_2 * wImag_2 ; 
                    tImagData_2 = *imagLow_2 * wReal_2 + *realLow_2 * wImag_2 ; 

                    /* Scale twiddle products to accomodate 32-bit accu. */
                    tRealData_2 >>= BUTTERFLY_SCALE_FACTOR ; 
                    tImagData_2 >>= BUTTERFLY_SCALE_FACTOR ; 

                    realData_2[l_2] = *realHi_2 - tRealData_2 ; 
                    imagData_2[l_2] = *imagHi_2 - tImagData_2 ; 
                    realData_2[i_2] += tRealData_2 ; 
                    imagData_2[i_2] += tImagData_2 ; 
                }
                argIndex_2 += deltaIndex_2 ; 
            }
        } /* End of iFFT loop */

#if BMDEBUG
        th_sprintf( szDebug, "[ %d ]\niFFT output : \n", passCount_2 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        for( i_2 = 0 ; i_2 < NUM_POINTS ; i_2++ )
        {
            /* Display iFFT spectrum */
#if BMDEBUG 
            th_sprintf( szDebug, "%8ld, %8ld\n", (n_long)realData_2[i_2], 
                (n_long)imagData_2[i_2] ) ; 
            DebugOut( szDebug ) ; 
#else
            WriteOut( realData_2[i_2] ) ; 
            WriteOut( imagData_2[i_2] ) ; 
#endif
        }

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/

        /* Gets 'signal_in' value from test data*/
        isTableLooped += GetInputValues( realData_3, imagData_3 ) ;

#if BMDEBUG
        DebugOut( "Input signal : \n" ) ; 
        for( i_3 = 0 ; i_3 < NUM_POINTS ; i_3++ )
        { 
            th_sprintf( szDebug, "%8ld, %8ld\n", realData_3[i_3], 
                imagData_3[i_3] ) ; 
            DebugOut( szDebug ) ; 
        }
#endif /* BMDEBUG */

        /* Bit Reversal */    
        for( i_3 = 0 ; i_3 < NUM_POINTS ; i_3++ )
        {        
            realBitRevData_3[i_3] = realData_3[bitRevInd[i_3]] ; 
            imagBitRevData_3[i_3] = imagData_3[bitRevInd[i_3]] ; 
        }

        /* Return bit reversed data to input arrays */    
        for( i_3 = 0 ; i_3 < NUM_POINTS ; i_3++ )
        {        
            realData_3[i_3] = realBitRevData_3[i_3] ; 
            imagData_3[i_3] = imagBitRevData_3[i_3] ; 
        }

        /* iFFT Computation */    

        /* Step through the stages */    

        for( passCount_3 = 0, k_3 = 1 ; 
             k_3 <= FFT_LENGTH ; 
             k_3++, passCount_3++ )
        {        
            n1_3 = 1 << k_3 ; 
            n2_3 = n1_3 >> 1 ; 

            /* Initialize twiddle factor lookup indicies */    
            argIndex_3 = 0 ; 
            deltaIndex_3 = ( NUM_POINTS / 2 ) / n2_3 ; 

            /* Step through the butterflies */    
            for( j_3 = 0 ; j_3 < n2_3 ; j_3++, passCount_3++ )
            {        
                /* Lookup twiddle factors */    
                wReal_3 = cosineV[argIndex_3] ; 
                /* Note iFFT reversal of sign */
                wImag_3 = -sineV[argIndex_3] ;

                /* Process butterflies with the same twiddle factors */    
                for( i_3 = j_3 ; i_3 < NUM_POINTS ; i_3 += n1_3, passCount_3++ )
                {
                    l_3 = i_3 + n2_3 ; 
                    realLow_3 = &realData_3[l_3] ; 
                    imagLow_3 = &imagData_3[l_3] ; 
                    realHi_3 = &realData_3[i_3] ; 
                    imagHi_3 = &imagData_3[i_3] ; 

                    /* Scale each stage to prevent overflow */    
                    *realLow_3 >>= STAGE_SCALE_FACTOR ; 
                    *imagLow_3 >>= STAGE_SCALE_FACTOR ; 
                    *realHi_3 >>= STAGE_SCALE_FACTOR ; 
                    *imagHi_3 >>= STAGE_SCALE_FACTOR ; 

                    tRealData_3 = *realLow_3 * wReal_3 - *imagLow_3 * wImag_3 ; 
                    tImagData_3 = *imagLow_3 * wReal_3 + *realLow_3 * wImag_3 ; 

                    /* Scale twiddle products to accomodate 32-bit accu. */
                    tRealData_3 >>= BUTTERFLY_SCALE_FACTOR ; 
                    tImagData_3 >>= BUTTERFLY_SCALE_FACTOR ; 

                    realData_3[l_3] = *realHi_3 - tRealData_3 ; 
                    imagData_3[l_3] = *imagHi_3 - tImagData_3 ; 
                    realData_3[i_3] += tRealData_3 ; 
                    imagData_3[i_3] += tImagData_3 ; 
                }
                argIndex_3 += deltaIndex_3 ; 
            }
        } /* End of iFFT loop */

#if BMDEBUG
        th_sprintf( szDebug, "[ %d ]\niFFT output : \n", passCount_3 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        for( i_3 = 0 ; i_3 < NUM_POINTS ; i_3++ )
        {
            /* Display iFFT spectrum */
#if BMDEBUG 
            th_sprintf( szDebug, "%8ld, %8ld\n", (n_long)realData_3[i_3], 
                (n_long)imagData_3[i_3] ) ; 
            DebugOut( szDebug ) ; 
#else
            WriteOut( realData_3[i_3] ) ; 
            WriteOut( imagData_3[i_3] ) ; 
#endif
        }

#if BMDEBUG
        th_sprintf( szDebug, "{ %08lX }\n", (n_ulong)loop_cnt ) ; 
        DebugOut( szDebug ) ; 
#else
#if DATA_SIZE == 0
        i_3 = (varsize)( loop_cnt &0x0000FFFF ) ; 
        WriteOut( i_3 ) ;
        i_3 = (varsize)( loop_cnt >> 16 ) ; 
        WriteOut( i_3 ) ; 
#else
        WriteOut( loop_cnt ) ; 
#endif
        i_3 = ( varsize )0xAAAA ; 
        WriteOut( i_3 ) ;
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
    tcdef->v1 = 0;
    tcdef->v2 = 0;
    tcdef->v3 = 0;
    tcdef->v4 = 0;
/* NON_INTRUSIVE_CRC_CHECK ok */
#if NON_INTRUSIVE_CRC_CHECK
	tcdef->CRC=0;
	for( loop_cnt = 0 ; loop_cnt < NUM_POINTS; loop_cnt++ )
	{
		tcdef->CRC = Calc_crc32((e_u32)realData_1[loop_cnt],tcdef->CRC);
		tcdef->CRC = Calc_crc32((e_u32)imagData_1[loop_cnt],tcdef->CRC);
		tcdef->CRC = Calc_crc32((e_u32)realData_2[loop_cnt],tcdef->CRC);
		tcdef->CRC = Calc_crc32((e_u32)imagData_2[loop_cnt],tcdef->CRC);
		tcdef->CRC = Calc_crc32((e_u32)realData_3[loop_cnt],tcdef->CRC);
		tcdef->CRC = Calc_crc32((e_u32)imagData_3[loop_cnt],tcdef->CRC);
	}
#elif	CRC_CHECK
	tcdef->CRC=0;
#else
	tcdef->CRC=0;
#endif

		return th_report_results(tcdef,EXPECTED_CRC);
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
    xil_printf(">>     Start of IFFT...\n\r");
    /* Benchmark Execution */
    while (failTest == 0) {
        failTest = t_run_test(&the_tcdef,argc,argv);
        if (failTest != 0)
        {
            
            xil_printf(">>     Dumping RAMfile information to the log...\n\r");
            for (n_int i = 0 ; i < RAMfileSize ; i++)
            {
                xil_printf("%8d\n\r",*RAMfilePtr++);
            }
        } else {
            th_free(RAMfileFree); /* Free RAMfile for next iteration so no Malloc error */
            th_free(inpSignal);
            xil_printf("%20d\n\r",benchIter++);
            
        }
    }
    xil_printf(">>      FFT test is finished\n\r");
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

