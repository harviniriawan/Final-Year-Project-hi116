/*==============================================================================
 *$RCSfile: bmark_lite.c,v $
 *
 *   DESC : This file contains the Test Main and other TH support functions
 *
 * AUTHOR : ARM, Ltd., Rick Foos, ECL, LLC
 *
 *  EEMBC : Automotive/Industrial Subcommittee 
 *
 *    CVS : $Revision: 1.15 $
 *          $Date: 2002/08/07 22:20:49 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/bitmnp01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.15  2002/08/07 22:20:49  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.14  2002/07/22 21:59:14  rick
 * General cleanup Beta 2b
 *
 * Revision 1.13  2002/07/22 16:10:07  rick
 * Fix iteration dependant NI CRC
 *
 * Revision 1.12  2002/07/18 23:33:52  rick
 * Fix iteration dependencies in NI CRC
 *
 * Revision 1.11  2002/07/10 19:01:02  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.10  2002/05/29 22:24:59  rick
 * Set recommended iterations with make
 *
 * Revision 1.9  2002/05/10 23:57:45  rick
 * Fix missing shifts in 16, and 32 bit CRC calculations
 *
 * Revision 1.8  2002/05/10 17:20:36  rick
 * Add al_main to API
 *
 * Revision 1.7  2002/04/25 20:10:44  rick
 * sprintf to th_sprintf
 *
 * Revision 1.6  2002/04/10 19:36:58  rick
 * Fixes to reduce Lite vs. Regular variances in timing
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

#define ALGO_GLOBALS    1
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
#define ITERATIONS 3000	/* required iterations for crc */
#else
#define ITERATIONS 3000	/* recommended iterations for benchmark */
#endif
#endif

#if CRC_CHECK
#define EXPECTED_CRC	0x8a97
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC	0xee97
#else
#define EXPECTED_CRC	0x0000
#endif

TCDef the_tcdef = 
{
    "AUT bitmnp01   ",
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Algorithm #8 -- Bit manipulation V1.0C0 - bitmnp01", 
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

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/

/* Input stimuli test data table */
extern const varsize inpVariableROM[] ;
extern const varsize digitROM[] ;

n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree; /* NOT PART OF BENCHMARK to free RAMfile */

varsize *inpNumber ;    /* Pointer to array of numbers to paint */
n_int   *inpMode ;      /* Pointer to array of painting modes */
varsize inputNum ;      /* The input argument for computation */
n_int   inverted ;      /* Paint 'inverted' character in display */

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
        "Algorithm 8 :  Bit Manipulation  Rev. 1.0C0 - bitmnp01\n" ; 
    n_char *szHeader = 
        "\n\ninputNum, inverted" "\n            display[0]...display[4]"
        "\n               |            | "
        "\n            display[32nd character], counter\n" ; 
    n_char szDebug[100] ; 
#endif /* BMDEBUG */
    n_char szDataSize[40] ;
    /* Input test data table looped */ 
    n_int isTableLooped = false ;
    /* Temporary pointer to 'inputStrint' */
    n_char *strInput ;    
    static varsize i1 ;
    static varsize i2 ;
    static varsize i3 ;
    static varsize j1 ;
    static varsize j2 ;
    static varsize j3 ; 
    /* Which character in the output display we are painting */
    static int digit1 ;
    static int digit2 ;
    static int digit3 ;
    /* Result array where the character is bit-mapped */
    static varsize display1[MAX_COLUMNS + 1] ;
    static varsize display2[MAX_COLUMNS + 1] ; 
    static varsize display3[MAX_COLUMNS + 1] ; 
    /* Pointer to a character's bitmap */
    static varsize *charset1 ;
    static varsize *charset2 ;
    static varsize *charset3 ;
    /* Array to hold the input BCD string */
    static varsize inputString1[INPUT_CHARS + 1] ;
    static varsize inputString2[INPUT_CHARS + 1] ; 
    static varsize inputString3[INPUT_CHARS + 1] ; 

    /* Unused */
    argc = argc ;
    argv = argv ;

    /*
     * First, initialize the data structures we need for the test
     * and allocate memory as needed.  Report an error if we can't.
     *
     */    

    /* Variable initializations at t=0 */    

    RAMfilePtr = 0 ;   /* Point to beginning of test output text file */
    tableCount = 0 ;    /* Start out at beginning of input test data */
    digit1 = 0 ; 
    digit2 = 0 ; 
    digit3 = 0 ; 

    /* Must clear out the input string( s )before using it */    

    strInput = (n_char*)inputString1 ; 
    for( i1 = 0 ; i1 < ( INPUT_CHARS / 2 ) ; i1++ )
    {        
        *strInput++ = (n_char)0xAA ; 
    }

    strInput = (n_char*)inputString2 ; 
    for( i2 = 0 ; i2 < ( INPUT_CHARS / 2 ) ; i2++ )
    {        
        *strInput++ = (n_char)0xAA ; 
    }

    strInput = (n_char*)inputString3 ; 
    for( i3 = 0 ; i3 < ( INPUT_CHARS / 2 ) ; i3++ )
    {        
        *strInput++ = (n_char)0xAA ; 
    }

    /* If debug output is desired, then must allocate some RAM... */    
    /* Point to beginning of test output file */
    RAMfilePtr = 0 ;
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

    /* Initialize the test data -- stimuli for the algorithm. */    

    if( !GetTestData() ) 
    {
        /* Allocate for the test input data table */
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
            __FILE__, __LINE__ ) ; 
    }

#if BMDEBUG
    DebugOut( szTitle ) ;    /* Print the title message in RAM file */
    DebugOut( szDataSize ) ; /*  and the data size */
    DebugOut( szHeader ) ;   /* Print the test output file hdr in RAM file */
#endif /* BMDEBUG */

    /* This is the actual benchmark algorithm. */    

    /* The Bit Manipulation algorithm presumes that we are receiving input
     * characters in BCD format( four bits )that must be stored in a FIFO
     * and then converted to dot-matrix bitmapped characters on a display.
     *
     * The steps are as follows : 
     *
     * 1.  Get a new character
     * 2.  Shift it into the bottom of a 32-character string of BCD digits.
     * 3.  Pull out each of the 32 digits from the string, starting from the
     *     bottom.
     * 4.  Find the digit in the character set.
     * 5.  Paint the selected character into the display memory (double-high).
     * 6.  Mirror-image the display memory top to bottom, bit by bit.
     * 7.  Go back to( 3 )until all 32 characters are done.
     * 8.  Do it all again from( 1 ).
     *
     * The algorithm is structured so that the bit manipulations could
     * be optimized down to BIT instructions, if available, but are
     * otherwise done in ordinary AND's and OR's. There's also plenty
     * of shifting, done four bits at a time, or multiples thereof.
     *
     */

      tcdef->CRC = 0;
    th_signal_start() ;

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
        th_sprintf( szDebug, "%6ld, %2d", ( long )inputNum, inverted ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Take the new input digit, and slide it into the input string */
        /* ...one nibble at a time, depending on the size of data */    
        
        for( i1 = ((INPUT_CHARS/sizeof(varsize)/2)-1) ; i1 > 0 ; i1-- )
        {        
            inputString1[i1] <<= BITS_PER_DIGIT ; 
            j1 = inputString1[i1 - 1] >> 
                ( sizeof( varsize ) * BITS_PER_BYTE - 4 ) ; 
            inputString1[i1] |= j1 ; 
        }
        /* Don't forget the last nibble */
        inputString1[i1] <<= BITS_PER_DIGIT ;
        /* And pick up the new digit */
        inputString1[i1] |= inputNum ;

        /* Now, copy each digit to the display */    
        for( digit1 = 0 ; digit1 < INPUT_CHARS ; digit1++ )
        {        
            /* Get each digit in the input string */    
            inputNum = inputString1[digit1 / ( sizeof( varsize ) * 2 )] ; 
            j1 = BITS_PER_DIGIT * ( digit1 % ( sizeof( varsize ) * 2 ) ) ; 

            /* Shift down the remainder of bits( if req'd. )*/
            if( j1 > 0 )
            {
                inputNum >>= j1 ; 
            }

            /* Get just one digit to paint */
            inputNum &= DIGIT_MASK ;

            /* Find the digit's bitmap */    
            switch( inputNum )
            {
            /* For the numeric digits... */
            case 0 : 
            case 1 : 
            case 2 : 
            case 3 : 
            case 4 : 
            case 5 : 
            case 6 : 
            case 7 : 
            case 8 : 
            case 9 : 
                charset1 = (varsize*)
                    &digitROM + ( ( inputNum + 1 ) * CHAR_COLUMNS ) ; 
                break ;

            case BLANK : 
                charset1 = (varsize*)&digitROM + BLANK_CHAR;
                break ;

            default :
                charset1 = (varsize*)&digitROM + ERROR_CHAR; 
                break ;
            }

            /* Paint the digit in the display */    
            for( i1 = 0 ; i1 < CHAR_COLUMNS ; i1++ )
            {        
                j1 = digit1 * CHAR_COLUMNS ; 

                /* First, clear out the column in display */
                display1[i1 + j1] = 0 ;

                if( !inverted )
                {
                    /* Normal orientation, not 'inverted' */
                    if( ( charset1[i1] &BIT_0 ) == 0 )
                    {            
                        /* Not a 'descender' */
                        if( BIT_1 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_1 ;
                            display1[i1 + j1] |= DOT_8 ;
                        }
                        if( BIT_2 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_2 ;
                            display1[i1 + j1] |= DOT_9 ;
                        }
                        if( BIT_3 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_3 ; 
                            display1[i1 + j1] |= DOT_10 ;
                        }
                        if( BIT_4 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_4 ; 
                            display1[i1 + j1] |= DOT_11 ;
                        }
                        if( BIT_5 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_5 ; 
                            display1[i1 + j1] |= DOT_12 ;
                        }
                        if( BIT_6 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_6 ; 
                            display1[i1 + j1] |= DOT_13 ;
                        }
                        if( BIT_7 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_7 ; 
                            display1[i1 + j1] |= DOT_14 ;
                        }
                    } /* End of 'normal' orientation, not a 'descender' */
                    else
                    {
                        /* This is 'normal orientation, and 'descender' */
                        if( BIT_1 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_3 ; 
                            display1[i1 + j1] |= DOT_10 ;
                        }
                        if( BIT_2 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_4 ; 
                            display1[i1 + j1] |= DOT_11 ;
                        }
                        if( BIT_3 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_5 ; 
                            display1[i1 + j1] |= DOT_12 ;
                        }
                        if( BIT_4 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_6 ; 
                            display1[i1 + j1] |= DOT_13 ;
                        }
                        if( BIT_5 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_7 ; 
                            display1[i1 + j1] |= DOT_14 ;
                        }
                        if( BIT_6 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_8 ; 
                            display1[i1 + j1] |= DOT_15 ;
                        }
                        if( BIT_7 &charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_9 ; 
                            display1[i1 + j1] |= DOT_16 ;
                        }
                    }
                } /* End of 'normal' orientation' */
                else
                {
                    /* Else, paint 'inverted', not a 'descender' */
                    if( ( charset1[i1] & BIT_0 ) == 0 )
                    {        
                        if( BIT_1 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_9 ; 
                            display1[i1 + j1] |= DOT_16 ;
                        }
                        if( BIT_2 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_8 ; 
                            display1[i1 + j1] |= DOT_15 ;
                        }
                        if( BIT_3 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_7 ; 
                            display1[i1 + j1] |= DOT_14 ;
                        }
                        if( BIT_4 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_6 ; 
                            display1[i1 + j1] |= DOT_13 ;
                        }
                        if( BIT_5 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_5 ; 
                            display1[i1 + j1] |= DOT_12 ;
                        }
                        if( BIT_6 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_4 ; 
                            display1[i1 + j1] |= DOT_11 ;
                        }
                        if( BIT_7 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_3 ; 
                            display1[i1 + j1] |= DOT_10 ;
                        }
                    } /* End of 'inverted' orientation, not a 'descender' */
                    else
                    {
                        /* Else, it's 'inverted', and 'descender' */
                        if( BIT_1 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_7 ; 
                            display1[i1 + j1] |= DOT_14 ;
                        }
                        if( BIT_2 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_6 ; 
                            display1[i1 + j1] |= DOT_13 ;
                        }
                        if( BIT_3 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_5 ; 
                            display1[i1 + j1] |= DOT_12 ;
                        }
                        if( BIT_4 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_4 ; 
                            display1[i1 + j1] |= DOT_11 ;
                        }
                        if( BIT_5 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_3 ; 
                            display1[i1 + j1] |= DOT_10 ;
                        }
                        if( BIT_6 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_2 ; 
                            display1[i1 + j1] |= DOT_9 ;
                        }
                        if( BIT_7 & charset1[i1] )
                        {        
                            display1[i1 + j1] |= DOT_1 ; 
                            display1[i1 + j1] |= DOT_8 ; 
                        }
                    } /* End of 'inverted' and 'descender' */
                } /* End of 'inverted' painting */
            } /* End of all the 'columns' of character */
            
            /* Now, copy the low-order bits to the high-order bits, in
             * reverse order */
            for( i1 = 0 ; i1 < CHAR_COLUMNS ; i1++ )
            {        
                if( display1[i1 + j1] & BIT_0 )
                {
                    display1[i1 + j1] |= DOT_32 ; 
                }
                if( display1[i1 + j1] & BIT_1 )
                {
                    display1[i1 + j1] |= DOT_31 ; 
                }
                if( display1[i1 + j1] & BIT_2 )
                {
                    display1[i1 + j1] |= DOT_30 ; 
                }
                if( display1[i1 + j1] & BIT_3 )
                {
                    display1[i1 + j1] |= DOT_29 ; 
                }
                if( display1[i1 + j1] & BIT_4 )
                {
                    display1[i1 + j1] |= DOT_28 ; 
                }
                if( display1[i1 + j1] & BIT_5 )
                {
                    display1[i1 + j1] |= DOT_27 ; 
                }
                if( display1[i1 + j1] & BIT_6 )
                {
                    display1[i1 + j1] |= DOT_26 ; 
                }
                if( display1[i1 + j1] & BIT_7 )
                {
                    display1[i1 + j1] |= DOT_25 ; 
                }
                if( display1[i1 + j1] & BIT_8 )
                {
                    display1[i1 + j1] |= DOT_24 ; 
                }
                if( display1[i1 + j1] & BIT_9 )
                {
                    display1[i1 + j1] |= DOT_23 ; 
                }
                if( display1[i1 + j1] & BIT_10 )
                {
                    display1[i1 + j1] |= DOT_22 ; 
                }
                if( display1[i1 + j1] & BIT_11 )
                {
                    display1[i1 + j1] |= DOT_21 ; 
                }
                if( display1[i1 + j1] & BIT_12 )
                {
                    display1[i1 + j1] |= DOT_20 ; 
                }
                if( display1[i1 + j1] & BIT_13 )
                {
                    display1[i1 + j1] |= DOT_19 ; 
                }
                if( display1[i1 + j1] & BIT_14 )
                {
                    display1[i1 + j1] |= DOT_18 ; 
                }
                if( display1[i1 + j1] & BIT_15 )
                {
                    display1[i1 + j1] |= DOT_17 ; 
                }
            } /* End of low-to-high bit reversal */
        } /* End of all 'digits' */

        for( i1 = 0 ; i1 < (INPUT_CHARS * CHAR_COLUMNS) ; i1 += CHAR_COLUMNS )
        {        
#if BMDEBUG
            th_sprintf( szDebug, 
                "\n            %08lX, %08lX, %08lX, %08lX, %08lX\n", 
                display1[i1], 
                display1[i1 + 1], 
                display1[i1 + 2], 
                display1[i1 + 3], 
                display1[i1 + 4] ) ; 
            DebugOut( szDebug ) ; 
#else
            WriteOut( display1[i1] ) ; 
            WriteOut( display1[i1 + 1] ) ; 
            WriteOut( display1[i1 + 2] ) ; 
            WriteOut( display1[i1 + 3] ) ; 
            WriteOut( display1[i1 + 4] ) ; 
#endif /* BMDEBUG */
        }

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/

        isTableLooped += GetInputValues() ;

#if BMDEBUG
        th_sprintf( szDebug, "%6ld, %2d", ( n_long )inputNum, inverted ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Take the new input digit, and slide it into the input string */
        /* ...one nibble at a time, depending on the size of data */    
        
        for( i2 = ((INPUT_CHARS/sizeof(varsize)/2)-1) ; i2 > 0 ; i2-- )
        {        
            inputString2[i2] <<= BITS_PER_DIGIT ; 
            j2 = inputString2[i2 - 1] >> 
                ( sizeof( varsize ) * BITS_PER_BYTE - 4 ) ; 
            inputString2[i2] |= j2 ; 
        }
        /* Don't forget the last nibble */
        inputString2[i2] <<= BITS_PER_DIGIT ;
        /* And pick up the new digit */
        inputString2[i2] |= inputNum ;

        /* Now, copy each digit to the display */    
        for( digit2 = 0 ; digit2 < INPUT_CHARS ; digit2++ )
        {        
            /* Get each digit in the input string */    
            inputNum = inputString2[digit2 / ( sizeof( varsize ) * 2 )] ; 
            j2 = BITS_PER_DIGIT * ( digit2 % ( sizeof( varsize ) * 2 ) ) ; 

            /* Shift down the remainder of bits( if req'd. )*/
            if( j2 > 0 )
            {
                inputNum >>= j2 ; 
            }

            /* Get just one digit to paint */
            inputNum &= DIGIT_MASK ;

            /* Find the digit's bitmap */    
            switch( inputNum )
            {
            /* For the numeric digits... */
            case 0 : 
            case 1 : 
            case 2 : 
            case 3 : 
            case 4 : 
            case 5 : 
            case 6 : 
            case 7 : 
            case 8 : 
            case 9 : 
                charset2 = (varsize*)
                    &digitROM + ( ( inputNum + 1 ) * CHAR_COLUMNS ) ; 
                break ;

            case BLANK : 
                charset2 = (varsize*)digitROM + BLANK_CHAR;
                break ;

            default :
                charset2 = (varsize*)&digitROM + ERROR_CHAR;
                break ;
            }

            /* Paint the digit in the display */    
            for( i2 = 0 ; i2 < CHAR_COLUMNS ; i2++ )
            {        
                j2 = digit2 * CHAR_COLUMNS ; 

                /* First, clear out the column in display */
                display2[i2 + j2] = 0 ;

                if( !inverted )
                {
                    /* Normal orientation, not 'inverted' */
                    if( ( charset2[i2] &BIT_0 ) == 0 )
                    {            
                        /* Not a 'descender' */
                        if( BIT_1 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_1 ;
                            display2[i2 + j2] |= DOT_8 ;
                        }
                        if( BIT_2 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_2 ;
                            display2[i2 + j2] |= DOT_9 ;
                        }
                        if( BIT_3 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_3 ; 
                            display2[i2 + j2] |= DOT_10 ;
                        }
                        if( BIT_4 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_4 ; 
                            display2[i2 + j2] |= DOT_11 ;
                        }
                        if( BIT_5 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_5 ; 
                            display2[i2 + j2] |= DOT_12 ;
                        }
                        if( BIT_6 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_6 ; 
                            display2[i2 + j2] |= DOT_13 ;
                        }
                        if( BIT_7 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_7 ; 
                            display2[i2 + j2] |= DOT_14 ;
                        }
                    } /* End of 'normal' orientation, not a 'descender' */
                    else
                    {
                        /* This is 'normal orientation, and 'descender' */
                        if( BIT_1 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_3 ; 
                            display2[i2 + j2] |= DOT_10 ;
                        }
                        if( BIT_2 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_4 ; 
                            display2[i2 + j2] |= DOT_11 ;
                        }
                        if( BIT_3 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_5 ; 
                            display2[i2 + j2] |= DOT_12 ;
                        }
                        if( BIT_4 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_6 ; 
                            display2[i2 + j2] |= DOT_13 ;
                        }
                        if( BIT_5 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_7 ; 
                            display2[i2 + j2] |= DOT_14 ;
                        }
                        if( BIT_6 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_8 ; 
                            display2[i2 + j2] |= DOT_15 ;
                        }
                        if( BIT_7 &charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_9 ; 
                            display2[i2 + j2] |= DOT_16 ;
                        }
                    }
                } /* End of 'normal' orientation' */
                else
                {
                    /* Else, paint 'inverted', not a 'descender' */
                    if( ( charset2[i2] & BIT_0 ) == 0 )
                    {        
                        if( BIT_1 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_9 ; 
                            display2[i2 + j2] |= DOT_16 ;
                        }
                        if( BIT_2 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_8 ; 
                            display2[i2 + j2] |= DOT_15 ;
                        }
                        if( BIT_3 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_7 ; 
                            display2[i2 + j2] |= DOT_14 ;
                        }
                        if( BIT_4 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_6 ; 
                            display2[i2 + j2] |= DOT_13 ;
                        }
                        if( BIT_5 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_5 ; 
                            display2[i2 + j2] |= DOT_12 ;
                        }
                        if( BIT_6 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_4 ; 
                            display2[i2 + j2] |= DOT_11 ;
                        }
                        if( BIT_7 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_3 ; 
                            display2[i2 + j2] |= DOT_10 ;
                        }
                    } /* End of 'inverted' orientation, not a 'descender' */
                    else
                    {
                        /* Else, it's 'inverted', and 'descender' */
                        if( BIT_1 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_7 ; 
                            display2[i2 + j2] |= DOT_14 ;
                        }
                        if( BIT_2 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_6 ; 
                            display2[i2 + j2] |= DOT_13 ;
                        }
                        if( BIT_3 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_5 ; 
                            display2[i2 + j2] |= DOT_12 ;
                        }
                        if( BIT_4 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_4 ; 
                            display2[i2 + j2] |= DOT_11 ;
                        }
                        if( BIT_5 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_3 ; 
                            display2[i2 + j2] |= DOT_10 ;
                        }
                        if( BIT_6 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_2 ; 
                            display2[i2 + j2] |= DOT_9 ;
                        }
                        if( BIT_7 & charset2[i2] )
                        {        
                            display2[i2 + j2] |= DOT_1 ; 
                            display2[i2 + j2] |= DOT_8 ; 
                        }
                    } /* End of 'inverted' and 'descender' */
                } /* End of 'inverted' painting */
            } /* End of all the 'columns' of character */
            
            /* Now, copy the low-order bits to the high-order bits, in
             * reverse order */
            for( i2 = 0 ; i2 < CHAR_COLUMNS ; i2++ )
            {        
                if( display2[i2 + j2] & BIT_0 )
                {
                    display2[i2 + j2] |= DOT_32 ; 
                }
                if( display2[i2 + j2] & BIT_1 )
                {
                    display2[i2 + j2] |= DOT_31 ; 
                }
                if( display2[i2 + j2] & BIT_2 )
                {
                    display2[i2 + j2] |= DOT_30 ; 
                }
                if( display2[i2 + j2] & BIT_3 )
                {
                    display2[i2 + j2] |= DOT_29 ; 
                }
                if( display2[i2 + j2] & BIT_4 )
                {
                    display2[i2 + j2] |= DOT_28 ; 
                }
                if( display2[i2 + j2] & BIT_5 )
                {
                    display2[i2 + j2] |= DOT_27 ; 
                }
                if( display2[i2 + j2] & BIT_6 )
                {
                    display2[i2 + j2] |= DOT_26 ; 
                }
                if( display2[i2 + j2] & BIT_7 )
                {
                    display2[i2 + j2] |= DOT_25 ; 
                }
                if( display2[i2 + j2] & BIT_8 )
                {
                    display2[i2 + j2] |= DOT_24 ; 
                }
                if( display2[i2 + j2] & BIT_9 )
                {
                    display2[i2 + j2] |= DOT_23 ; 
                }
                if( display2[i2 + j2] & BIT_10 )
                {
                    display2[i2 + j2] |= DOT_22 ; 
                }
                if( display2[i2 + j2] & BIT_11 )
                {
                    display2[i2 + j2] |= DOT_21 ; 
                }
                if( display2[i2 + j2] & BIT_12 )
                {
                    display2[i2 + j2] |= DOT_20 ; 
                }
                if( display2[i2 + j2] & BIT_13 )
                {
                    display2[i2 + j2] |= DOT_19 ; 
                }
                if( display2[i2 + j2] & BIT_14 )
                {
                    display2[i2 + j2] |= DOT_18 ; 
                }
                if( display2[i2 + j2] & BIT_15 )
                {
                    display2[i2 + j2] |= DOT_17 ; 
                }
            } /* End of low-to-high bit reversal */
        } /* End of all 'digits' */

        for( i2 = 0 ; i2 < (INPUT_CHARS * CHAR_COLUMNS) ; i2 += CHAR_COLUMNS )
        {        
#if BMDEBUG
            th_sprintf( szDebug, 
                "\n            %08lX, %08lX, %08lX, %08lX, %08lX\n", 
                display2[i2], 
                display2[i2 + 1], 
                display2[i2 + 2], 
                display2[i2 + 3], 
                display2[i2 + 4] ) ; 
            DebugOut( szDebug ) ; 
#else
            WriteOut( display2[i2] ) ; 
            WriteOut( display2[i2 + 1] ) ; 
            WriteOut( display2[i2 + 2] ) ; 
            WriteOut( display2[i2 + 3] ) ; 
            WriteOut( display2[i2 + 4] ) ; 
#endif /* BMDEBUG */
        }

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/

        isTableLooped += GetInputValues() ;

#if BMDEBUG
        th_sprintf( szDebug, "%6ld, %2d", ( n_long )inputNum, inverted ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Take the new input digit, and slide it into the input string */
        /* ...one nibble at a time, depending on the size of data */    
        
        for( i3 = ((INPUT_CHARS/sizeof(varsize)/2)-1) ; i3 > 0 ; i3-- )
        {        
            inputString3[i3] <<= BITS_PER_DIGIT ; 
            j3 = inputString3[i3 - 1] >> 
                ( sizeof( varsize ) * BITS_PER_BYTE - 4 ) ; 
            inputString3[i3] |= j3 ; 
        }
        /* Don't forget the last nibble */
        inputString3[i3] <<= BITS_PER_DIGIT ;
        /* And pick up the new digit */
        inputString3[i3] |= inputNum ;

        /* Now, copy each digit to the display */    
        for( digit3 = 0 ; digit3 < INPUT_CHARS ; digit3++ )
        {        
            /* Get each digit in the input string */    
            inputNum = inputString3[digit3 / ( sizeof( varsize ) * 2 )] ; 
            j3 = BITS_PER_DIGIT * ( digit3 % ( sizeof( varsize ) * 2 ) ) ; 

            /* Shift down the remainder of bits( if req'd. )*/
            if( j3 > 0 )
            {
                inputNum >>= j3 ; 
            }

            /* Get just one digit to paint */
            inputNum &= DIGIT_MASK ;

            /* Find the digit's bitmap */    
            switch( inputNum )
            {
            /* For the numeric digits... */
            case 0 : 
            case 1 : 
            case 2 : 
            case 3 : 
            case 4 : 
            case 5 : 
            case 6 : 
            case 7 : 
            case 8 : 
            case 9 : 
                charset3 = (varsize*)
                    &digitROM + ( ( inputNum + 1 ) * CHAR_COLUMNS ) ; 
                break ;

            case BLANK : 
                charset3 = (varsize*)&digitROM + BLANK_CHAR ;
                break ;

            default :
                charset3 = (varsize*)&digitROM + ERROR_CHAR ;
                break ;
            }

            /* Paint the digit in the display */    
            for( i3 = 0 ; i3 < CHAR_COLUMNS ; i3++ )
            {        
                j3 = digit3 * CHAR_COLUMNS ; 

                /* First, clear out the column in display */
                display3[i3 + j3] = 0 ;

                if( !inverted )
                {
                    /* Normal orientation, not 'inverted' */
                    if( ( charset3[i3] &BIT_0 ) == 0 )
                    {            
                        /* Not a 'descender' */
                        if( BIT_1 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_1 ;
                            display3[i3 + j3] |= DOT_8 ;
                        }
                        if( BIT_2 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_2 ;
                            display3[i3 + j3] |= DOT_9 ;
                        }
                        if( BIT_3 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_3 ; 
                            display3[i3 + j3] |= DOT_10 ;
                        }
                        if( BIT_4 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_4 ; 
                            display3[i3 + j3] |= DOT_11 ;
                        }
                        if( BIT_5 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_5 ; 
                            display3[i3 + j3] |= DOT_12 ;
                        }
                        if( BIT_6 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_6 ; 
                            display3[i3 + j3] |= DOT_13 ;
                        }
                        if( BIT_7 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_7 ; 
                            display3[i3 + j3] |= DOT_14 ;
                        }
                    } /* End of 'normal' orientation, not a 'descender' */
                    else
                    {
                        /* This is 'normal orientation, and 'descender' */
                        if( BIT_1 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_3 ; 
                            display3[i3 + j3] |= DOT_10 ;
                        }
                        if( BIT_2 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_4 ; 
                            display3[i3 + j3] |= DOT_11 ;
                        }
                        if( BIT_3 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_5 ; 
                            display3[i3 + j3] |= DOT_12 ;
                        }
                        if( BIT_4 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_6 ; 
                            display3[i3 + j3] |= DOT_13 ;
                        }
                        if( BIT_5 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_7 ; 
                            display3[i3 + j3] |= DOT_14 ;
                        }
                        if( BIT_6 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_8 ; 
                            display3[i3 + j3] |= DOT_15 ;
                        }
                        if( BIT_7 &charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_9 ; 
                            display3[i3 + j3] |= DOT_16 ;
                        }
                    }
                } /* End of 'normal' orientation' */
                else
                {
                    /* Else, paint 'inverted', not a 'descender' */
                    if( ( charset3[i3] & BIT_0 ) == 0 )
                    {        
                        if( BIT_1 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_9 ; 
                            display3[i3 + j3] |= DOT_16 ;
                        }
                        if( BIT_2 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_8 ; 
                            display3[i3 + j3] |= DOT_15 ;
                        }
                        if( BIT_3 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_7 ; 
                            display3[i3 + j3] |= DOT_14 ;
                        }
                        if( BIT_4 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_6 ; 
                            display3[i3 + j3] |= DOT_13 ;
                        }
                        if( BIT_5 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_5 ; 
                            display3[i3 + j3] |= DOT_12 ;
                        }
                        if( BIT_6 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_4 ; 
                            display3[i3 + j3] |= DOT_11 ;
                        }
                        if( BIT_7 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_3 ; 
                            display3[i3 + j3] |= DOT_10 ;
                        }
                    } /* End of 'inverted' orientation, not a 'descender' */
                    else
                    {
                        /* Else, it's 'inverted', and 'descender' */
                        if( BIT_1 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_7 ; 
                            display3[i3 + j3] |= DOT_14 ;
                        }
                        if( BIT_2 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_6 ; 
                            display3[i3 + j3] |= DOT_13 ;
                        }
                        if( BIT_3 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_5 ; 
                            display3[i3 + j3] |= DOT_12 ;
                        }
                        if( BIT_4 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_4 ; 
                            display3[i3 + j3] |= DOT_11 ;
                        }
                        if( BIT_5 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_3 ; 
                            display3[i3 + j3] |= DOT_10 ;
                        }
                        if( BIT_6 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_2 ; 
                            display3[i3 + j3] |= DOT_9 ;
                        }
                        if( BIT_7 & charset3[i3] )
                        {        
                            display3[i3 + j3] |= DOT_1 ; 
                            display3[i3 + j3] |= DOT_8 ; 
                        }
                    } /* End of 'inverted' and 'descender' */
                } /* End of 'inverted' painting */
            } /* End of all the 'columns' of character */
            
            /* Now, copy the low-order bits to the high-order bits, 
             * in reverse order */
            for( i3 = 0 ; i3 < CHAR_COLUMNS ; i3++ )
            {        
                if( display3[i3 + j3] & BIT_0 )
                {
                    display3[i3 + j3] |= DOT_32 ; 
                }
                if( display3[i3 + j3] & BIT_1 )
                {
                    display3[i3 + j3] |= DOT_31 ; 
                }
                if( display3[i3 + j3] & BIT_2 )
                {
                    display3[i3 + j3] |= DOT_30 ; 
                }
                if( display3[i3 + j3] & BIT_3 )
                {
                    display3[i3 + j3] |= DOT_29 ; 
                }
                if( display3[i3 + j3] & BIT_4 )
                {
                    display3[i3 + j3] |= DOT_28 ; 
                }
                if( display3[i3 + j3] & BIT_5 )
                {
                    display3[i3 + j3] |= DOT_27 ; 
                }
                if( display3[i3 + j3] & BIT_6 )
                {
                    display3[i3 + j3] |= DOT_26 ; 
                }
                if( display3[i3 + j3] & BIT_7 )
                {
                    display3[i3 + j3] |= DOT_25 ; 
                }
                if( display3[i3 + j3] & BIT_8 )
                {
                    display3[i3 + j3] |= DOT_24 ; 
                }
                if( display3[i3 + j3] & BIT_9 )
                {
                    display3[i3 + j3] |= DOT_23 ; 
                }
                if( display3[i3 + j3] & BIT_10 )
                {
                    display3[i3 + j3] |= DOT_22 ; 
                }
                if( display3[i3 + j3] & BIT_11 )
                {
                    display3[i3 + j3] |= DOT_21 ; 
                }
                if( display3[i3 + j3] & BIT_12 )
                {
                    display3[i3 + j3] |= DOT_20 ; 
                }
                if( display3[i3 + j3] & BIT_13 )
                {
                    display3[i3 + j3] |= DOT_19 ; 
                }
                if( display3[i3 + j3] & BIT_14 )
                {
                    display3[i3 + j3] |= DOT_18 ; 
                }
                if( display3[i3 + j3] & BIT_15 )
                {
                    display3[i3 + j3] |= DOT_17 ; 
                }
            } /* End of low-to-high bit reversal */
        } /* End of all 'digits' */

        for( i3 = 0 ; i3 < (INPUT_CHARS * CHAR_COLUMNS) ; i3 += CHAR_COLUMNS )
        {        
#if BMDEBUG
            th_sprintf( szDebug, 
                "\n            %08lX, %08lX, %08lX, %08lX, %08lX\n", 
                display3[i3], 
                display3[i3 + 1], 
                display3[i3 + 2], 
                display3[i3 + 3], 
                display3[i3 + 4] ) ; 
            DebugOut( szDebug ) ; 
#else
            WriteOut( display3[i3] ) ; 
            WriteOut( display3[i3 + 1] ) ; 
            WriteOut( display3[i3 + 2] ) ; 
            WriteOut( display3[i3 + 3] ) ; 
            WriteOut( display3[i3 + 4] ) ; 
#endif /* BMDEBUG */
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %08lX\n", ( n_ulong )loop_cnt ) ; 
        DebugOut( szDebug ) ; 
#else
#if DATA_SIZE == 0
        i3 = (varsize)( loop_cnt &0x0000FFFF ) ; 
        WriteOut( i3 ) ;
        i3 = (varsize)( loop_cnt >> 16 ) ; 
        WriteOut( i3 ) ; 
#else
        WriteOut( loop_cnt ) ; 
#endif
        i3 = (varsize)0xAAAA ; 
        WriteOut( i3 ) ;
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
/* CRC_CHECK ok */
#if NON_INTRUSIVE_CRC_CHECK
/*  final results are iteration dependant */
	tcdef->CRC=0;
	for( i1 = 0 ; i1 < NUM_TESTS ; i1++ )
	{
	tcdef->CRC = Calc_crc32((e_u32)inpVariableROM[i1],tcdef->CRC);
	}
#elif	CRC_CHECK
	tcdef->CRC=0;
	for( i1 = 0 ; i1 < (INPUT_CHARS * CHAR_COLUMNS) ; i1++ )
	{
	tcdef->CRC = Calc_crc32((e_u32)display1[i1],tcdef->CRC);
	tcdef->CRC = Calc_crc32((e_u32)display2[i1],tcdef->CRC);
	tcdef->CRC = Calc_crc32((e_u32)display3[i1],tcdef->CRC);
	}
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
    xil_printf(">>     Start of Bit Manipulation...\n\r");
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
            th_free(inpNumber);
            th_free(inpMode);
            xil_printf("%20d\n\r",benchIter++);

            
        }
    }
    xil_printf(">>      Bit Manipulation test is finished\n\r");
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
     RAMfilePtr += RAMfile_increment;

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
