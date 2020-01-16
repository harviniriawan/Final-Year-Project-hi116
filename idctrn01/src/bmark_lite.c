/*==============================================================================
 *$RCSfile: bmark_lite.c,v $
 *
 *   DESC : This file contains the Test Main and other TH support functions
 *
 * AUTHOR : ARM, Ltd., Rick Foos, ECL, LLC
 *
 *  EEMBC : Automotive/Industrial Subcommittee 
 *
 *    CVS : $Revision: 1.14 $
 *          $Date: 2002/08/07 22:21:17 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/idctrn01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.14  2002/08/07 22:21:17  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.13  2002/07/22 21:59:25  rick
 * General cleanup Beta 2b
 *
 * Revision 1.12  2002/07/19 23:10:25  rick
 * Fix iteration dependant NI CRC's
 *
 * Revision 1.11  2002/07/11 22:13:28  rick
 * Initialize tcdef results
 *
 * Revision 1.10  2002/07/10 19:01:14  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.9  2002/05/29 22:25:09  rick
 * Set recommended iterations with make
 *
 * Revision 1.8  2002/05/10 23:57:45  rick
 * Fix missing shifts in 16, and 32 bit CRC calculations
 *
 * Revision 1.7  2002/05/10 17:20:37  rick
 * Add al_main to API
 *
 * Revision 1.6  2002/04/25 20:10:44  rick
 * sprintf to th_sprintf
 *
 * Revision 1.5  2002/04/23 18:38:40  rick
 * Add anytoi to th reg project files
 *
 * Revision 1.4  2002/03/11 22:11:48  rick
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

#define ALGO_GLOBALS    1
#include "algo.h"


/* Estimate of allocation for NUM_TESTS*( debug test + 2 variables )*/
#define T_BSIZE (MAX_FILESIZE+((NUM_TESTS+1)*VAR_COUNT*4))


/* ======================================================================== */
/*         F U N C T I O N   P R O T O T Y P E S                            */
/* ======================================================================== */
int main(int argc, const char* argv[] );
int t_run_test(struct TCDef *tcdef, int argc, const char* argv[]);
e_u16 Calc_crc16( e_u16 data, e_u16 crc );
/* Define iterations */
#if !defined(ITERATIONS) || CRC_CHECK || ITERATIONS==DEFAULT
#undef ITERATIONS
#if CRC_CHECK
#define ITERATIONS 1500	/* required iterations for crc */
#else
#define ITERATIONS 1500	/* recommended iterations for benchmark */
#endif
#endif

#if CRC_CHECK
#define EXPECTED_CRC	0x310e
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC	0xcffc
#else
#define EXPECTED_CRC	0x0000
#endif

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/

TCDef the_tcdef = 
{
	"AUT idctrn01   ",
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Alg.#14 - iDCT,Inverse Discrete Cosine Transform V1.0E0 ",  
    TCDEF_REVISION, 
    { EEMBC_TH_MAJOR, EEMBC_TH_MINOR, EEMBC_TH_STEP, EEMBC_TH_REVISION }, 
    { 0, 0, 0, 0 },   /* Target Hardware Version Number Required( make all zeros to ignore )*/
    { 1, 0, 'B', 0 }, /* The Version number of this Benchmark */
	ITERATIONS,    /* recomended iterations for benchmark score */
	0,
	0,
	0,
	0,
	0,
	0,
	0
} ; 

/* Input stimuli test data table */
extern const varsize inpSignalROM[] ;

n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree;       /* NOT PART OF BENCHMARK to free RAMfile */
 
/* Input character string representing one 8x8 block */
unsigned char inChar[ROWS + COLS] ;    
/* Points to input data stream */
unsigned char *inpString ;   

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
    char *szTitle = 
        "\nEEMBC Automotive/Industrial Subcommittee Benchmarks (c)1998-1999\n"
        "Algorithm #14 -- Inverse Discrete Cosign Transform (iDCT) V1.0B0\n";
    char *szHeader = 
        "\n\n[inpString], output block, {loop counter}\n" ; 
    char szDebug[100] ; 
#endif /* BMDEBUG */
    char szDataSize[40] ; 
    /* Input test data table looped */
    int isTableLooped = false ;
    /* The cosine tables( used by all )*/
    long cosMatrixA[ROWS][COLS] ;
    long cosMatrixB[ROWS][COLS] ;
    /* Intermediate factors to build the cosine tables */
    double cosine ;
    double cosMultA ;
    double cosMultB ;
    /* The working 8x8 sub-matrix of compressed pixels */
    static signed char f_1[ROWS][COLS] ;
    static signed char f_2[ROWS][COLS] ; 
    static signed char f_3[ROWS][COLS] ; 
    /* The working matricies where pixels are decompressed */
    static long F_1[ROWS][COLS] ;
    static long G_1[ROWS][COLS] ;
    static long F_2[ROWS][COLS] ;
    static long G_2[ROWS][COLS] ; 
    static long F_3[ROWS][COLS] ;
    static long G_3[ROWS][COLS] ; 
    /* Temporary counters/indicies */
    static varsize u_1 ;
    static varsize v_1 ;
    static varsize i_1 ;
    static varsize j_1 ;
    static varsize k_1 ;
    static varsize u_2 ;
    static varsize v_2 ;
    static varsize i_2 ;
    static varsize j_2 ;
    static varsize k_2 ; 
    static varsize u_3 ;
    static varsize v_3 ;
    static varsize i_3 ;
    static varsize j_3 ;
    static varsize k_3 ; 

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
    szDataSize[0] = (char)( '0' + DATA_SIZE ) ; 
    szDataSize[1] = '\0' ; 
#endif /* BMDEBUG */

    /* Initialize the test data -- stimuli for the algorithm */    
    if( !GetTestData() )   /* Allocate for the test input data table */
    {
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
            __FILE__, __LINE__ ) ; 
    }

    /* Build cosine matrix */    
    cosMultA = 2.0 * atan( 1.0 ) / (double)ROWS ; 

    for( u_1 = 0 ; u_1 < ROWS ; u_1++ )
    {        
        cosMultB = u_1 *cosMultA ; 

        for( v_1 = 0 ; v_1 < COLS ; v_1++ )
        {        
            if( u_1 > 0 )
            {
                cosine = (double)( 2.0 * cos( ( 2 * v_1 + 1 ) * cosMultB ) ) ; 
            }
            else
            {
                cosine = (double)( cos( ( 2 * v_1 + 1 ) * cosMultB ) ) ; 
            }
            cosMatrixA[u_1][v_1] = (long)( cosine * COS_SCALE_FACTOR ) ; 
            cosMatrixB[v_1][u_1] = cosMatrixA[u_1][v_1] ; 
        }
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
     * An image file( classic picture of Baboon's face )has been compressed
     * using DCT with 8x8 pixel blocks into 16-byte strings.  Each string is
     * read in, and the first 8 bytes are expanded into an 8x8 matrix, which
     * is then processed by the inverse Discrete Cosine Transform.
     *
     * Prior to execution of the iDCT algorithm, the cosine matrices are
     * generated -- these are created once, normally they would exist as ROM
     * tables in a real embedded application.  The two tables are really one
     * cosine table which has been transposed into a second matrix.
     *
     * The algorithm performs decompression on only a section of the complete
     * image, putting the result back into the same input matrix from which
     * the input pixels came, so this would be typical of an in-place iDCT.
     * All algorithm math is done in 32-bit integer, although the cosine tables
     * are generated in double-precision, then scaled and converted to 32-bit
     * integer (of no consequence to an embedded app, not part of the 
     * algorithm).
     *
     * This algorithm performs the decompression one block at a time, but does
     * not actually reconstruct the image.  We are only interested here in the
     * iDCT performance.  The design of this algorithm is such that it could be
     * expanded to also recreate the finished image if desired.
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
        DebugOut( "\n[ " ) ; 
        for( i_1 = 0 ; i_1 < ROWS + COLS ; i_1++ )
        {        
            th_sprintf( szDebug, "  %02X", inChar[i_1] ) ; 
            DebugOut( szDebug ) ; 
        }
        DebugOut( " ]\n" ) ; 
#endif /* BMDEBUG */

        /* Must clear out the working 8x8 matrix to start */    
        for( i_1 = 0 ; i_1 < ROWS ; i_1++ )
        {        
            for( j_1 = 0 ; j_1 < COLS ; j_1++ )
            {        
                f_1[i_1][j_1] = 0 ; 
            }
        }

        /* Unpack 16 bytes for each 8x8 block */    
        f_1[0][0] = (signed char)inChar[0] ; 
        f_1[0][1] = (signed char)inChar[1] ; 
        f_1[0][2] = 3 * unPack( (unsigned char)( inChar[2] >> 4 ) ) ; 
        f_1[0][3] = 3 * unPack( inChar[2] ) ; 
        f_1[0][4] = 2 * unPack( (unsigned char)( inChar[3] >> 4 ) ) ; 
        f_1[0][5] = 2 * unPack( inChar[3] ) ; 
        f_1[0][6] = (signed char) unPack( (unsigned char)( inChar[4] >> 4 ) ) ; 
        f_1[0][7] = (signed char) unPack( inChar[4] ) ; 

        f_1[1][0] = (signed char)inChar[5] ; 
        f_1[1][1] = (signed char)inChar[6] ; 
        f_1[1][2] = 2 * unPack( (unsigned char)( inChar[7] >> 4 ) ) ; 
        f_1[1][3] = (signed char) unPack( inChar[7] ) ; 
        f_1[1][4] = (signed char) unPack( (unsigned char)( inChar[8] >> 4 ) ) ; 
        f_1[1][5] = (signed char) unPack( inChar[8] ) ; 

        f_1[2][0] = 3 * unPack( (unsigned char)( inChar[9] >> 4 ) ) ; 
        f_1[2][1] = 2 * unPack( inChar[9] ) ; 
        f_1[2][2] = 2 * unPack( (unsigned char)( inChar[10] >> 4 ) ) ; 
        f_1[2][3] = (signed char) unPack( inChar[10] ) ; 

        f_1[3][0] = 3 * unPack( (unsigned char)( inChar[11] >> 4 ) ) ; 
        f_1[3][1] = (signed char) unPack( inChar[11] ) ; 
        f_1[3][2] = (signed char) unPack( (unsigned char)( inChar[12] >> 4 ) ) ; 
        f_1[3][3] = (signed char) unPack( inChar[12] ) ; 

        f_1[4][0] = 2 * unPack( (unsigned char)( inChar[13] >> 4 ) ) ; 
        f_1[4][1] = (signed char) unPack( inChar[13] ) ; 

        f_1[5][0] = 2 * unPack( (unsigned char)( inChar[14] >> 4 ) ) ; 
        f_1[5][1] = (signed char) unPack( inChar[14] ) ; 

        f_1[6][0] = (signed char) unPack( (unsigned char)( inChar[15] >> 4 ) ) ; 

        f_1[7][0] = (signed char) unPack( inChar[15] ) ; 

        /* Now calculate the IDCT by matrix multiplies */    
        for( i_1 = 0 ; i_1 < ROWS ; i_1++ )
        {        
            for( j_1 = 0 ; j_1 < COLS ; j_1++ )
            {        
                F_1[i_1][j_1] = 0 ; 
                G_1[i_1][j_1] = 0 ; 
            }
        }

        for( k_1 = 0 ; k_1 < COLS ; k_1++ )
        {        
            for( i_1 = 0 ; i_1 < ROWS ; i_1++ )
            {        
                for( j_1 = 0 ; j_1 < COLS ; j_1++ )
                {
                    F_1[i_1][j_1] += f_1[i_1][k_1] * cosMatrixA[k_1][j_1] ; 
                }
            }
        }

#if BMDEBUG 
        th_printf( "\nF_1[row][col] : \n" ) ; 

        for( i_1 = 0 ; i_1 < ROWS ; i_1++ )
        {        
            for( j_1 = 0 ; j_1 < COLS ; j_1++ )
            {
                th_printf( "  %8ld", F_1[i_1][j_1] ) ; 
            }
            th_printf( "\n" ) ; 
        }
#endif /* BMDEBUG */

        for( k_1 = 0 ; k_1 < COLS ; k_1++ )
        {        
            for( i_1 = 0 ; i_1 < ROWS ; i_1++ )
            {        
                for( j_1 = 0 ; j_1 < COLS ; j_1++ )
                {
                    G_1[i_1][j_1] += cosMatrixB[i_1][k_1] * F_1[k_1][j_1] ; 
                }
            }
        }

#if BMDEBUG        
        th_printf( "\nG_1[row][col] : \n" ) ; 

        for( i_1 = 0 ; i_1 < ROWS ; i_1++ )
        {        
            for( j_1 = 0 ; j_1 < COLS ; j_1++ )
            {
                th_printf( "  %11ld", G_1[i_1][j_1] ) ; 
            }
            th_printf( "\n" ) ; 
        }
#endif /* BMDEBUG */

        /* Put the result back in the integer input matrix */    
        for( u_1 = 0 ; u_1 < ROWS ; u_1++ )
        {        
            for( v_1 = 0 ; v_1 < COLS ; v_1++ )
            {        
                /* Round off */
                G_1[u_1][v_1] += 1 <<( ( COS_SCALE_EXP * 2 ) - 1 ) ; 
                /* Unscale the resulting pixel, and place back in image */    
                f_1[u_1][v_1] = (signed char)
                    ( G_1[u_1][v_1] >> COS_SCALE_EXP * 2 ) ; 
            }
        }

        for( u_1 = 0 ; u_1 < ROWS ; u_1++ )
        {        
            for( v_1 = 0 ; v_1 < COLS ; v_1++ )
            {        
#if BMDEBUG        
                th_sprintf( szDebug, "  %3d", (unsigned char)f_1[u_1][v_1] ) ; 
                DebugOut( szDebug ) ; 
#else
                WriteOut( f_1[u_1][v_1] ) ;
#endif /* BMDEBUG */
            }
#if BMDEBUG        
        DebugOut( "\n" ) ; 
#endif /* BMDEBUG */
        }

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/

        isTableLooped += GetInputValues() ;

#if BMDEBUG
        DebugOut( "\n[ " ) ; 
        for( i_2 = 0 ; i_2 < ROWS + COLS ; i_2++ )
        {        
            th_sprintf( szDebug, "  %02X", inChar[i_2] ) ; 
            DebugOut( szDebug ) ; 
        }
        DebugOut( " ]\n" ) ; 
#endif /* BMDEBUG */

        /* Must clear out the working 8x8 matrix to start */    
        for( i_2 = 0 ; i_2 < ROWS ; i_2++ )
        {        
            for( j_2 = 0 ; j_2 < COLS ; j_2++ )
            {        
                f_2[i_2][j_2] = 0 ; 
            }
        }

        /* Unpack 16 bytes for each 8x8 block */    
        f_2[0][0] = (signed char)inChar[0] ; 
        f_2[0][1] = (signed char)inChar[1] ; 
        f_2[0][2] = 3 * unPack( (unsigned char)( inChar[2] >> 4 ) ) ; 
        f_2[0][3] = 3 * unPack( inChar[2] ) ; 
        f_2[0][4] = 2 * unPack( (unsigned char)( inChar[3] >> 4 ) ) ; 
        f_2[0][5] = 2 * unPack( inChar[3] ) ; 
        f_2[0][6] = (signed char) unPack( (unsigned char)( inChar[4] >> 4 ) ) ; 
        f_2[0][7] = (signed char) unPack( inChar[4] ) ; 

        f_2[1][0] = (signed char)inChar[5] ; 
        f_2[1][1] = (signed char)inChar[6] ; 
        f_2[1][2] = 2 * unPack( (unsigned char)( inChar[7] >> 4 ) ) ; 
        f_2[1][3] = (signed char) unPack( inChar[7] ) ; 
        f_2[1][4] = (signed char) unPack( (unsigned char)( inChar[8] >> 4 ) ) ; 
        f_2[1][5] = (signed char) unPack( inChar[8] ) ; 

        f_2[2][0] = 3 * unPack( (unsigned char)( inChar[9] >> 4 ) ) ; 
        f_2[2][1] = 2 * unPack( inChar[9] ) ; 
        f_2[2][2] = 2 * unPack( (unsigned char)( inChar[10] >> 4 ) ) ; 
        f_2[2][3] = (signed char) unPack( inChar[10] ) ; 

        f_2[3][0] = 3 * unPack( (unsigned char)( inChar[11] >> 4 ) ) ; 
        f_2[3][1] = (signed char) unPack( inChar[11] ) ; 
        f_2[3][2] = (signed char) unPack( (unsigned char)( inChar[12] >> 4 ) ) ; 
        f_2[3][3] = (signed char) unPack( inChar[12] ) ; 

        f_2[4][0] = 2 * unPack( (unsigned char)( inChar[13] >> 4 ) ) ; 
        f_2[4][1] = (signed char) unPack( inChar[13] ) ; 

        f_2[5][0] = 2 * unPack( (unsigned char)( inChar[14] >> 4 ) ) ; 
        f_2[5][1] = (signed char) unPack( inChar[14] ) ; 

        f_2[6][0] = (signed char) unPack( (unsigned char)( inChar[15] >> 4 ) ) ; 

        f_2[7][0] = (signed char) unPack( inChar[15] ) ; 

        /* Now calculate the IDCT by matrix multiplies */    
        for( i_2 = 0 ; i_2 < ROWS ; i_2++ )
        {        
            for( j_2 = 0 ; j_2 < COLS ; j_2++ )
            {        
                F_2[i_2][j_2] = 0 ; 
                G_2[i_2][j_2] = 0 ; 
            }
        }

        for( k_2 = 0 ; k_2 < COLS ; k_2++ )
        {        
            for( i_2 = 0 ; i_2 < ROWS ; i_2++ )
            {        
                for( j_2 = 0 ; j_2 < COLS ; j_2++ )
                {
                    F_2[i_2][j_2] += f_2[i_2][k_2] * cosMatrixA[k_2][j_2] ; 
                }
            }
        }

#if BMDEBUG 
        th_printf( "\nF_1[row][col] : \n" ) ; 

        for( i_2 = 0 ; i_2 < ROWS ; i_2++ )
        {        
            for( j_2 = 0 ; j_2 < COLS ; j_2++ )
            {
                th_printf( "  %8ld", F_2[i_2][j_2] ) ; 
            }
            th_printf( "\n" ) ; 
        }
#endif /* BMDEBUG */

        for( k_2 = 0 ; k_2 < COLS ; k_2++ )
        {        
            for( i_2 = 0 ; i_2 < ROWS ; i_2++ )
            {        
                for( j_2 = 0 ; j_2 < COLS ; j_2++ )
                {
                    G_2[i_2][j_2] += cosMatrixB[i_2][k_2] * F_2[k_2][j_2] ; 
                }
            }
        }

#if BMDEBUG        
        th_printf( "\nG_1[row][col] : \n" ) ; 

        for( i_2 = 0 ; i_2 < ROWS ; i_2++ )
        {        
            for( j_2 = 0 ; j_2 < COLS ; j_2++ )
            {
                th_printf( "  %11ld", G_2[i_2][j_2] ) ; 
            }
            th_printf( "\n" ) ; 
        }
#endif /* BMDEBUG */

        /* Put the result back in the integer input matrix */    
        for( u_2 = 0 ; u_2 < ROWS ; u_2++ )
        {        
            for( v_2 = 0 ; v_2 < COLS ; v_2++ )
            {        
                /* Round off */
                G_2[u_2][v_2] += 1 <<( ( COS_SCALE_EXP * 2 ) - 1 ) ; 
                /* Unscale the resulting pixel, and place back in image */    
                f_2[u_2][v_2] = (signed char)
                    ( G_2[u_2][v_2] >> COS_SCALE_EXP * 2 ) ; 
            }
        }

        for( u_2 = 0 ; u_2 < ROWS ; u_2++ )
        {        
            for( v_2 = 0 ; v_2 < COLS ; v_2++ )
            {        
#if BMDEBUG        
                th_sprintf( szDebug, "  %3d", (unsigned char)f_2[u_2][v_2] ) ; 
                DebugOut( szDebug ) ; 
#else
                WriteOut( f_2[u_2][v_2] ) ;
#endif /* BMDEBUG */
            }
#if BMDEBUG        
        DebugOut( "\n" ) ; 
#endif /* BMDEBUG */
        }

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/

        isTableLooped += GetInputValues() ;

#if BMDEBUG
        DebugOut( "\n[ " ) ; 
        for( i_3 = 0 ; i_3 < ROWS + COLS ; i_3++ )
        {        
            th_sprintf( szDebug, "  %02X", inChar[i_3] ) ; 
            DebugOut( szDebug ) ; 
        }
        DebugOut( " ]\n" ) ; 
#endif /* BMDEBUG */

        /* Must clear out the working 8x8 matrix to start */    
        for( i_3 = 0 ; i_3 < ROWS ; i_3++ )
        {        
            for( j_3 = 0 ; j_3 < COLS ; j_3++ )
            {        
                f_3[i_3][j_3] = 0 ; 
            }
        }

        /* Unpack 16 bytes for each 8x8 block */    
        f_3[0][0] = (signed char)inChar[0] ; 
        f_3[0][1] = (signed char)inChar[1] ; 
        f_3[0][2] = 3 * unPack( (unsigned char)( inChar[2] >> 4 ) ) ; 
        f_3[0][3] = 3 * unPack( inChar[2] ) ; 
        f_3[0][4] = 2 * unPack( (unsigned char)( inChar[3] >> 4 ) ) ; 
        f_3[0][5] = 2 * unPack( inChar[3] ) ; 
        f_3[0][6] = (signed char)unPack( (unsigned char)( inChar[4] >> 4 ) ) ; 
        f_3[0][7] = (signed char)unPack( inChar[4] ) ; 

        f_3[1][0] = (signed char)inChar[5] ; 
        f_3[1][1] = (signed char)inChar[6] ; 
        f_3[1][2] = 2 * unPack( (unsigned char)( inChar[7] >> 4 ) ) ; 
        f_3[1][3] = (signed char)unPack( inChar[7] ) ; 
        f_3[1][4] = (signed char)unPack( (unsigned char)( inChar[8] >> 4 ) ) ; 
        f_3[1][5] = (signed char)unPack( inChar[8] ) ; 

        f_3[2][0] = 3 * unPack( (unsigned char)( inChar[9] >> 4 ) ) ; 
        f_3[2][1] = 2 * unPack( inChar[9] ) ; 
        f_3[2][2] = 2 * unPack( (unsigned char)( inChar[10] >> 4 ) ) ; 
        f_3[2][3] = (signed char)unPack( inChar[10] ) ; 

        f_3[3][0] = 3 * unPack( (unsigned char)( inChar[11] >> 4 ) ) ; 
        f_3[3][1] = (signed char)unPack( inChar[11] ) ; 
        f_3[3][2] = (signed char)unPack( (unsigned char)( inChar[12] >> 4 ) ) ; 
        f_3[3][3] = (signed char)unPack( inChar[12] ) ; 

        f_3[4][0] = 2 * unPack( (unsigned char)( inChar[13] >> 4 ) ) ; 
        f_3[4][1] = (signed char)unPack( inChar[13] ) ; 

        f_3[5][0] = 2 * unPack( (unsigned char)( inChar[14] >> 4 ) ) ; 
        f_3[5][1] = (signed char)unPack( inChar[14] ) ; 

        f_3[6][0] = (signed char)unPack( (unsigned char)( inChar[15] >> 4 ) ) ; 

        f_3[7][0] = (signed char)unPack( inChar[15] ) ; 

        /* Now calculate the IDCT by matrix multiplies */    
        for( i_3 = 0 ; i_3 < ROWS ; i_3++ )
        {        
            for( j_3 = 0 ; j_3 < COLS ; j_3++ )
            {        
                F_3[i_3][j_3] = 0 ; 
                G_3[i_3][j_3] = 0 ; 
            }
        }

        for( k_3 = 0 ; k_3 < COLS ; k_3++ )
        {        
            for( i_3 = 0 ; i_3 < ROWS ; i_3++ )
            {        
                for( j_3 = 0 ; j_3 < COLS ; j_3++ )
                {
                    F_3[i_3][j_3] += f_3[i_3][k_3] * cosMatrixA[k_3][j_3] ; 
                }
            }
        }

#if BMDEBUG 
        th_printf( "\nF_1[row][col] : \n" ) ; 

        for( i_3 = 0 ; i_3 < ROWS ; i_3++ )
        {        
            for( j_3 = 0 ; j_3 < COLS ; j_3++ )
            {
                th_printf( "  %8ld", F_3[i_3][j_3] ) ; 
            }
            th_printf( "\n" ) ; 
        }
#endif /* BMDEBUG */

        for( k_3 = 0 ; k_3 < COLS ; k_3++ )
        {        
            for( i_3 = 0 ; i_3 < ROWS ; i_3++ )
            {        
                for( j_3 = 0 ; j_3 < COLS ; j_3++ )
                {
                    G_3[i_3][j_3] += cosMatrixB[i_3][k_3] * F_3[k_3][j_3] ; 
                }
            }
        }

#if BMDEBUG        
        th_printf( "\nG_1[row][col] : \n" ) ; 

        for( i_3 = 0 ; i_3 < ROWS ; i_3++ )
        {        
            for( j_3 = 0 ; j_3 < COLS ; j_3++ )
            {
                th_printf( "  %11ld", G_3[i_3][j_3] ) ; 
            }
            th_printf( "\n" ) ; 
        }
#endif /* BMDEBUG */

        /* Put the result back in the integer input matrix */    
        for( u_3 = 0 ; u_3 < ROWS ; u_3++ )
        {        
            for( v_3 = 0 ; v_3 < COLS ; v_3++ )
            {        
                /* Round off */
                G_3[u_3][v_3] += 1 <<( ( COS_SCALE_EXP * 2 ) - 1 ) ; 
                /* Unscale the resulting pixel, and place back in image */    
                f_3[u_3][v_3] = (signed char)
                    ( G_3[u_3][v_3] >> COS_SCALE_EXP * 2 ) ; 
            }
        }

        for( u_3 = 0 ; u_3 < ROWS ; u_3++ )
        {        
            for( v_3 = 0 ; v_3 < COLS ; v_3++ )
            {        
#if BMDEBUG        
                th_sprintf( szDebug, "  %3d", (unsigned char)f_3[u_3][v_3] ) ; 
                DebugOut( szDebug ) ; 
#else
                WriteOut( f_3[u_3][v_3] ) ;
#endif /* BMDEBUG */
            }
#if BMDEBUG        
        DebugOut( "\n" ) ; 
#endif /* BMDEBUG */
        }

#if BMDEBUG        
        th_sprintf( szDebug, "{ %08lX }\n", (unsigned long)loop_cnt ) ; 
        DebugOut( szDebug ) ; 
#else
#if DATA_SIZE == 0
        i_1 = (varsize)( loop_cnt &0x0000FFFF ) ; 
        WriteOut( i_1 ) ;
        i_1 = (varsize)( loop_cnt >> 16 ) ; 
        WriteOut( i_1 ) ; 
#else
        WriteOut( loop_cnt ) ; 
#endif
        i_1 = 0xAAAA ; 
        WriteOut( i_1 ) ;
#endif /* BMDEBUG */

    }

	tcdef->duration = th_signal_finished() ;
    tcdef->iterations = loop_cnt ; 
    tcdef->v1 = 0 ; 
    tcdef->v2 = 0 ; 
    tcdef->v3 = 0 ; 
    tcdef->v4 = 0 ; 

#if NON_INTRUSIVE_CRC_CHECK
/* final values iteration dependant */
	tcdef->CRC=0;
	for( u_3 = 0 ; u_3 < ROWS ; u_3++ )
		for( v_3 = 0 ; v_3 < COLS ; v_3++ ) {
		tcdef->CRC = Calc_crc32((e_u32)cosMatrixA[u_3][v_3],tcdef->CRC);
		tcdef->CRC = Calc_crc32((e_u32)cosMatrixB[u_3][v_3],tcdef->CRC);
		}
#elif	CRC_CHECK
/* Iteration dependant results check */
	tcdef->CRC=0;
	for( u_3 = 0 ; u_3 < ROWS ; u_3++ )
		for( v_3 = 0 ; v_3 < COLS ; v_3++ ) {
		tcdef->CRC = Calc_crc8((e_u8)f_1[u_3][v_3],tcdef->CRC);
		tcdef->CRC = Calc_crc8((e_u8)f_2[u_3][v_3],tcdef->CRC);
		tcdef->CRC = Calc_crc8((e_u8)f_3[u_3][v_3],tcdef->CRC);
		}
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
    xil_printf(">>     Start of iDCT...\n\r");
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
            th_free(inpString);
            xil_printf(">>   Test is working just fine, iteration: %8ld\n\r",benchIter++);
        }
    }
    xil_printf(">>      iDCT test is finished\n\r");
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

/*
*  Function :  unPack
*
*  Unpack function converts lower nibble to a signed int -8 to 7.
*
*/    

short
unPack( unsigned char c )
{        
    /* Only want lower four bit nibble */
    c = c & (unsigned char)0x0F ;  
    
    if( c > 7 ) {
        /* Negative nibble */
        return( ( short )( c - 16 ) ) ;
    }
    else
    {
        /* positive nibble */
        return( ( short )c ) ;       
    }
} /* End of function 'unPack' */
