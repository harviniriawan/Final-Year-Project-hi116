/*==============================================================================
 *$RCSfile: bmark_lite.c,v $
 *
 *   DESC : This file contains the Test Main and other TH support functions
 *
 * AUTHOR : ARM, Ltd., Rick Foos, ECL, LLC
 *
 *  EEMBC : Automotive Subcommittee 
 *
 *    CVS : $Revision: 1.13 $
 *          $Date: 2002/08/12 22:12:27 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/matrix01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.13  2002/08/12 22:12:27  rick
 * INCRC regression fixes
 *
 * Revision 1.12  2002/08/07 22:21:34  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.11  2002/07/22 21:59:27  rick
 * General cleanup Beta 2b
 *
 * Revision 1.10  2002/07/19 23:10:27  rick
 * Fix iteration dependant NI CRC's
 *
 * Revision 1.9  2002/07/10 19:01:22  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.8  2002/05/29 22:25:14  rick
 * Set recommended iterations with make
 *
 * Revision 1.7  2002/05/10 17:20:37  rick
 * Add al_main to API
 *
 * Revision 1.6  2002/04/25 20:10:45  rick
 * sprintf to th_sprintf
 *
 * Revision 1.5  2002/04/01 22:49:52  administrator
 * Fixed NI CRC Algorithm
 *
 * Revision 1.4  2002/03/11 22:11:49  rick
 * ITERATIONS, CRC_CHECK, NON_INTRUSIVE TCDef Usage
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
#include "xil_exception.h"
#include "xscugic.h"
#include "xipipsu.h"
#include "xipipsu_hw.h"
#include "xil_cache.h"

#define ALGO_GLOBALS    1
#include "algo.h"


/* Estimate of allocation for NUM_TESTS*( debug test + 2 variables )*/
#define T_BSIZE (MAX_FILESIZE+((NUM_TESTS+1)*VAR_COUNT*4))

#if WINDOWS_EXAMPLE_CODE 
#include <stdio.h> 
#endif

#include "thlib.h"

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

/* Define iterations */
#if !defined(ITERATIONS) || CRC_CHECK || ITERATIONS==DEFAULT
#undef ITERATIONS
#if CRC_CHECK
#define ITERATIONS 11	/* required iterations for crc */
#else
#define ITERATIONS 110	/* recommended iterations for benchmark */
#endif
#endif

/* Pre-caclculated CRC values */
#if	CRC_CHECK
#define EXPECTED_CRC 0x0000
#elif NON_INTRUSIVE_CRC_CHECK

/* Seems like 0x5137 is the correct value */
#if DATA_SIZE == 1
/*#define EXPECTED_CRC 0x2bf3*/
#define EXPECTED_CRC 0x5137
#else
/*#define EXPECTED_CRC 0x2bf3*/
#define EXPECTED_CRC 0x5137
#endif

#else
#define EXPECTED_CRC 0x0000
#endif

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/


TCDef the_tcdef = 
{
    "AUT matrix01   ", 
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Algorithm #12 -- Matrix Math V1.0E0 - matrix", 
    TCDEF_REVISION, 
    { EEMBC_TH_MAJOR, EEMBC_TH_MINOR, EEMBC_TH_STEP, EEMBC_TH_REVISION }, 
    { 0, 0, 0, 0 },   /* Target Hardware Version Number Required( make all zeros to ignore )*/
    { 1, 0, 'A', 0 }, /* The Version number of this Benchmark */
	ITERATIONS,    /* recomended iterations for benchmark score */
	0,
	0,
	0,
	0,
	0,
	0,
	0
} ; 

int MatrixDecomp( void ) ;  /* LU matrix decomposition */
void FwdBackSubst( void ) ; /* Forward/backward matrix substitution */

#if !HAVE_FABS          /* Does math package have 'fabs()' ? */
double fabs( double ) ; /* If not, better make one */

double
fabs( double a )
{
    /* Return absolute value of arg */
    if( a < 0 )
    {    
        /* Whether arg is negative */
        return( -a ) ; 
    }
    /* ...or arg is positive, */
    return( a ) ;    

} /* End of function 'fabs' */

#endif /* !HAVE_FABS */

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/

n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree;       /* NOT PART OF BENCHMARK, free ramfile after every run */

varsize *inpVariable ;
varsize *inpVariableFree ; /* NOT PART OF BENCHMARK */
varsize **matrixA;     /* The 2D input matrix for LU decomposition */
varsize *rowPtr ;       /* The guts of the matrix 'A' */
varsize *resultB ;      /* The result vector -- solution for 'X' */
n_int   nCount ;        /* Input argument that is value of 'n' for 'n x n' matrix */

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
    size_t		loop_cnt;

#if BMDEBUG
    char *szTitle =
        "\nEEMBC Automotive/Industrial Subcommittee Benchmarks (c)1998-1999\n"
        "Algorithm 12 :  Matrix Math  Rev. 1.0A0\n" ; 
    char *szHeader = 
        "\n\n {input matrix} : [n], a[1, 1], a[1, 2]..., "
        "a[n], b[1], {decomposed matrix}, x[1], x[2]..., "
        "{determinant},  counter\n" ; 
    char szDebug[100] ; 
#endif /* BMDEBUG */
    static char szDataSize[40] ; 
    /* Input test data table looped */
    int isTableLooped = false ;
    /* Matrix row index */
    static int row1 ;
    static int row2 ;
    static int row3 ;
    /* Matrix column index */
    static int col1 ;
    static int col2 ;
    static int col3 ;
    static int k1 ;
    static int k2 ;
    static int k3 ; 
    /* Used to determine row containing max value */
    static varsize bigElmnt1 ;
    static varsize bigElmnt2 ;
    static varsize bigElmnt3 ;
    /* Used to keep track of row containing max value */
    static int maxRow1 ;
    static int maxRow2 ;
    static int maxRow3 ;
    /* Temp holder of a matrix element */
    static varsize element1 ;
    static varsize element2 ;
    static varsize element3 ;
    /* Summation of matrix elements */
    static varsize sum1 ;
    static varsize sum2 ;
    static varsize sum3 ;
    /* The determinant of matrix 'A' */
    static varsize determinant1 ;
    static varsize determinant2 ;
    static varsize determinant3 ;
    /* Points to the matrix scale vector */
    static varsize *scaleVector ;
    /* Used in fwd/backward substitution */
    static int ii1 ;
    static int ii2 ;
    static int ii3 ;
	div_t	mod_11;

    /* Unused */
    argc = argc ;
    argv = argv ;

	/*
	 * Matrix has a period of 11 iterations to cycle through the
	 * data. This effects workload, with all iterations that aren't
	 * a multiple of 11 doing less work. Patrick Webster, ARM
	 * Set iterations to be a multiple of 11, with a minimum of 11,
	 * TechTag.
	 */
	mod_11 = div(tcdef->rec_iterations,11);
	tcdef->rec_iterations = mod_11.quot?mod_11.quot*11:11;

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

      tcdef->CRC = 0;
    th_signal_start() ;    /* Tell the host that the test has begun */

#if BMDEBUG
    DebugOut( szTitle ) ;    /* Print the title message in RAM file */
    DebugOut( szDataSize ) ; /*  and the data size */
    DebugOut( szHeader ) ;   /* Print the test output file hdr in RAM file */
#endif /* BMDEBUG */

    /*
     * This is the actual benchmark algorithm.
     *
     */    

    /* 
     * This kernel is devoted to matrix math, performing an LU decomposition
     * on an 'n x n' input matrix.  This kernel will perform the LU 
     * decomposition algorithm. In addition, this kernel will compute the
     * determinant of the matrix and a cross product with a second matrix.
     * The input matrix data will be loaded into RAM from a test data file
     * to simulate "real-world" input data. The output data file in RAM will
     * contain the result of the matrix manipulation.
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

        /* Gets 'matrixA' and 'nCount' values from test data*/
        isTableLooped = GetInputValues() ;
        
        if( isTableLooped == -1 )
        {
            th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
                __FILE__, __LINE__ ) ; 
        }

#if BMDEBUG
        for( row1 = 0 ; row1 < nCount ; row1++ )
        {        
            /* Display the input row/column count */
            th_sprintf( szDebug, "%4d", row1 ) ;
            DebugOut( szDebug ) ; 

            /* ...and the input matrix 'A' */
            for( col1 = 0 ; col1 < nCount ; col1++ )
            {   th_sprintf( szDebug, " %14e, ", matrixA[row1][col1] ) ; 
                DebugOut( szDebug ) ; 
            }

            th_sprintf( szDebug, " %14e\n", resultB[row1] ) ; 
            DebugOut( szDebug ) ; 
        }
#endif /* BMDEBUG */

        /* First, perform LU decomposition on the input matrix 'A' */    
        maxRow1 = 0 ; 

        /* Allocate for the scaling vector */    
        scaleVector = (varsize *)th_malloc( sizeof( varsize ) * nCount ) ; 
        
        if( scaleVector == NULL )
        {
            th_exit( THE_OUT_OF_MEMORY, 
                "Cannot Allocate 'scaleVector' %s : %d",
                __FILE__, __LINE__ ) ; 
        }

        /* Really just the sign of the determinant( +/-1 )*/
        determinant1 = 1.0 ;
        
        /* Determine the implicit scaling of the input matrix */
        for( row1 = 0 ; row1 < nCount ; row1++ )
        {
            /* ...by finding the largest element */
            bigElmnt1 = 0.0 ;
            
            for( col1 = 0 ; col1 < nCount ; col1++ )
            {        
                element1 = fabs( matrixA[row1][col1] ) ; 
                if( element1 > bigElmnt1 )
                {
                    bigElmnt1 = element1 ; 
                }
            }
            
            /* Save the resulting 'scaleVector' */
            scaleVector[row1] = 1.0 / bigElmnt1 ;
            
        } /* End of 'for' to determine scaling */

        /* This is Crout's method */
        for( col1 = 0 ; col1 < nCount ; col1++ )
        {
            /* First, find the upper triangle */
            for( row1 = 0 ; row1 < col1 ; row1++ )
            {
                sum1 = matrixA[row1][col1] ; 

                for( k1 = 0 ; k1 < row1 ; k1++ )
                {
                    sum1 -= matrixA[row1][k1] * matrixA[k1][col1] ; 
                }

                matrixA[row1][col1] = sum1 ;
            }

            /* Next, search for the largest */
            bigElmnt1 = 0.0 ;
            
            /* ...pivot element */
            for( row1 = col1 ; row1 < nCount ; row1++ )
            {
                sum1 = matrixA[row1][col1] ; 

                for( k1 = 0 ; k1 < col1 ; k1++ )
                {
                    sum1 -= matrixA[row1][k1] * matrixA[k1][col1] ; 
                }

                matrixA[row1][col1] = sum1 ; 
                element1 = scaleVector[row1] * fabs( sum1 ) ; 

                /* Is this the best pivot element ? */
                if( element1 >= bigElmnt1 )
                {
                    /* Then remember it */
                    bigElmnt1 = element1 ;
                    maxRow1 = row1 ; 
                }
            }

            if( col1 != maxRow1 )
            {
                /* Need to interchange rows ? */
                for( k1 = 0 ; k1 < nCount ; k1++ )
                {
                    /* Yes, interchange the rows... */
                    element1 = matrixA[maxRow1][k1] ; 
                    matrixA[maxRow1][k1] = matrixA[col1][k1] ; 
                    matrixA[col1][k1] = element1 ; 
                }

                /* Including the result vector */
                element1 = resultB[maxRow1] ;    
                resultB[maxRow1] = resultB[col1] ; 
                resultB[col1] = element1 ; 
                element1 = scaleVector[maxRow1] ; 
                scaleVector[maxRow1] = scaleVector[col1] ; 
                scaleVector[col1] = element1 ; 
                
                /* Will flip the sign of the determinant */
                determinant1 = -determinant1 ;    
            }   

            if( matrixA[col1][col1] == 0.0 )
            {
                matrixA[col1][col1] = TINY_COEF ; 
            }

            if( col1 != ( nCount - 1 ) )
            {        
                element1 = 1.0 / matrixA[col1][col1] ; 
                for( row1 = ( col1 + 1 ) ; row1 < nCount ; row1++ )
                {
                    matrixA[row1][col1] *= element1 ; 
                }
            }
        } /* end for() Crout's method */

        /* Free up memory from 'scaleVector' */
        th_free( (void*)scaleVector ) ;

        /*
         * End of function 'MatrixDecomp' 
         *
         */    

#if BMDEBUG        
        for( row1 = 0 ; row1 < nCount ; row1++ )
        {        
            /* Display the input row, column count */
            th_sprintf( szDebug, "%4d", row1 ) ;
            DebugOut( szDebug ) ; 
            for( col1 = 0 ; col1 < nCount ; col1++ )
            {        
                /* ...and the decomposed matrix 'A' */
                th_sprintf( szDebug, " %14e, ", matrixA[row1][col1] ) ; 
                DebugOut( szDebug ) ; 
            }
            DebugOut( "\n" ) ; 
        }
#endif /* BMDEBUG */

        /*
         * Now, perform forward/backward substitution on the decomposition
         * of matrix 'A' 
         *
         */    

        ii1 = -1 ;

        /* Do the forward substitution */
        for( row1 = 0 ; row1 < nCount ; row1++ )
        {
            sum1 = resultB[row1] ; 
            if( ii1 != -1 )
            {        
                for( col1 = ii1 ; col1 <= ( row1 - 1 ) ; col1++ )
                {
                    sum1 -= matrixA[row1][col1] * resultB[col1] ; 
                }
            }
            else 
            {        
                if( sum1 != 0.0 )
                {
                    ii1 = row1 ; 
                }
            }
            resultB[row1] = sum1 ; 
        } /* end of forward substitution */

        /* Now do the back-substitution */
        for( row1 = ( nCount - 1 ) ; row1 >= 0 ; row1-- )
        {
            sum1 = resultB[row1] ; 
            if( row1 != ( nCount - 1 ) )
            {        
                for( col1 = ( row1 + 1 ) ; col1 < nCount ; col1++ )
                {
                    sum1 -= matrixA[row1][col1] * resultB[col1] ; 
                }
            }
            /* Store element of solution vector X */
            resultB[row1] = sum1 / matrixA[row1][row1] ;
        }

        /*
         * End of function 'FwdBackSubst'
         *
         */    

#if BMDEBUG    
        DebugOut( "    " ) ;    
        for( row1 = 0 ; row1 < nCount ; row1++ )
        {
            /* Show the resulting matrix 'B' */
            th_sprintf( szDebug, " %14e, ", resultB[row1] ) ; 
            DebugOut( szDebug ) ; 
        }
        DebugOut( "\n" ) ; 
#else
        for( row1 = 0 ; row1 < nCount ; row1++ )
        {
            WriteOut( resultB[row1] ) ;
        }
#endif /* BMDEBUG */

        /*
         * Next, calculate the determinant of matrix 'A' 
         *
         */    

        /* Calculate the determinant from the... */
        for( col1 = 0 ; col1 < nCount ; col1++ )   
        {
            /* ...decomposed matrix 'A' */
            determinant1 *= matrixA[col1][col1] ;
        }

#if BMDEBUG        
        th_sprintf( szDebug, "det= %14e\n", determinant1 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( determinant1 ) ; 
#endif /* BMDEBUG */

        /* Free up memory for the next pass */
        th_free( (void*)matrixA ) ;
        th_free( (void*)rowPtr ) ; 
        th_free( (void*)resultB ) ; 

#if BMDEBUG
        if( isTableLooped )
        {
            DebugOut( "END--END--END\n" ) ;
        }
#endif /* BMDEBUG */

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/

        /* Gets 'matrixA' and 'nCount' values from test data*/
        isTableLooped = GetInputValues() ;
        
        if( isTableLooped == -1 )
        {
            th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
                __FILE__, __LINE__ ) ; 
        }

#if BMDEBUG
        for( row2 = 0 ; row2 < nCount ; row2++ )
        {        
            /* Display the input row/column count */
            th_sprintf( szDebug, "%4d", row2 ) ;
            DebugOut( szDebug ) ; 

            /* ...and the input matrix 'A' */
            for( col2 = 0 ; col2 < nCount ; col2++ )
            {   th_sprintf( szDebug, " %14e, ", matrixA[row2][col2] ) ; 
                DebugOut( szDebug ) ; 
            }

            th_sprintf( szDebug, " %14e\n", resultB[row2] ) ; 
            DebugOut( szDebug ) ; 
        }
#endif /* BMDEBUG */

        /* First, perform LU decomposition on the input matrix 'A' */    
        maxRow2 = 0 ; 

        /* Allocate for the scaling vector */    
        scaleVector = (varsize *)th_malloc( sizeof( varsize ) * nCount ) ; 
        
        if( scaleVector == NULL )
        {
            th_exit( THE_OUT_OF_MEMORY, 
                "Cannot Allocate 'scaleVector' %s : %d",
                __FILE__, __LINE__ ) ; 
        }

        /* Really just the sign of the determinant( +/-1 )*/
        determinant2 = 1.0 ;
        
        /* Determine the implicit scaling of the input matrix */
        for( row2 = 0 ; row2 < nCount ; row2++ )
        {
            /* ...by finding the largest element */
            bigElmnt2 = 0.0 ;
            
            for( col2 = 0 ; col2 < nCount ; col2++ )
            {        
                element2 = fabs( matrixA[row2][col2] ) ; 
                if( element2 > bigElmnt2 )
                {
                    bigElmnt2 = element2 ; 
                }
            }
            
            /* Save the resulting 'scaleVector' */
            scaleVector[row2] = 1.0 / bigElmnt2 ;
            
        } /* End of 'for' to determine scaling */

        /* This is Crout's method */
        for( col2 = 0 ; col2 < nCount ; col2++ )
        {
            /* First, find the upper triangle */
            for( row2 = 0 ; row2 < col2 ; row2++ )
            {
                sum2 = matrixA[row2][col2] ; 

                for( k2 = 0 ; k2 < row2 ; k2++ )
                {
                    sum2 -= matrixA[row2][k2] * matrixA[k2][col2] ; 
                }

                matrixA[row2][col2] = sum2 ;
            }

            /* Next, search for the largest */
            bigElmnt2 = 0.0 ;
            
            /* ...pivot element */
            for( row2 = col2 ; row2 < nCount ; row2++ )
            {
                sum2 = matrixA[row2][col2] ; 

                for( k2 = 0 ; k2 < col2 ; k2++ )
                {
                    sum2 -= matrixA[row2][k2] * matrixA[k2][col2] ; 
                }

                matrixA[row2][col2] = sum2 ; 
                element2 = scaleVector[row2] * fabs( sum2 ) ; 

                /* Is this the best pivot element ? */
                if( element2 >= bigElmnt2 )
                {
                    /* Then remember it */
                    bigElmnt2 = element2 ;
                    maxRow2 = row2 ; 
                }
            }

            if( col2 != maxRow2 )
            {
                /* Need to interchange rows ? */
                for( k2 = 0 ; k2 < nCount ; k2++ )
                {
                    /* Yes, interchange the rows... */
                    element2 = matrixA[maxRow2][k2] ; 
                    matrixA[maxRow2][k2] = matrixA[col2][k2] ; 
                    matrixA[col2][k2] = element2 ; 
                }

                /* Including the result vector */
                element2 = resultB[maxRow2] ;    
                resultB[maxRow2] = resultB[col2] ; 
                resultB[col2] = element2 ; 
                element2 = scaleVector[maxRow2] ; 
                scaleVector[maxRow2] = scaleVector[col2] ; 
                scaleVector[col2] = element2 ; 
                
                /* Will flip the sign of the determinant */
                determinant2 = -determinant2 ;    
            }   

            if( matrixA[col2][col2] == 0.0 )
            {
                matrixA[col2][col2] = TINY_COEF ; 
            }

            if( col2 != ( nCount - 1 ) )
            {        
                element2 = 1.0 / matrixA[col2][col2] ; 
                for( row2 = ( col2 + 1 ) ; row2 < nCount ; row2++ )
                {
                    matrixA[row2][col2] *= element2 ; 
                }
            }
        } /* end for() Crout's method */

        /* Free up memory from 'scaleVector' */
        th_free( (void*)scaleVector ) ;

        /*
         * End of function 'MatrixDecomp' 
         *
         */    

#if BMDEBUG        
        for( row2 = 0 ; row2 < nCount ; row2++ )
        {        
            /* Display the input row, column count */
            th_sprintf( szDebug, "%4d", row2 ) ;
            DebugOut( szDebug ) ; 
            for( col2 = 0 ; col2 < nCount ; col2++ )
            {        
                /* ...and the decomposed matrix 'A' */
                th_sprintf( szDebug, " %14e, ", matrixA[row2][col2] ) ; 
                DebugOut( szDebug ) ; 
            }
            DebugOut( "\n" ) ; 
        }
#endif /* BMDEBUG */

        /*
         * Now, perform forward/backward substitution on the decomposition
         * of matrix 'A' 
         *
         */    

        ii2 = -1 ;

        /* Do the forward substitution */
        for( row2 = 0 ; row2 < nCount ; row2++ )
        {
            sum2 = resultB[row2] ; 
            if( ii2 != -1 )
            {        
                for( col2 = ii2 ; col2 <= ( row2 - 1 ) ; col2++ )
                {
                    sum2 -= matrixA[row2][col2] * resultB[col2] ; 
                }
            }
            else 
            {        
                if( sum2 != 0.0 )
                {
                    ii2 = row2 ; 
                }
            }
            resultB[row2] = sum2 ; 
        } /* end of forward substitution */

        /* Now do the back-substitution */
        for( row2 = ( nCount - 1 ) ; row2 >= 0 ; row2-- )
        {
            sum2 = resultB[row2] ; 
            if( row2 != ( nCount - 1 ) )
            {        
                for( col2 = ( row2 + 1 ) ; col2 < nCount ; col2++ )
                {
                    sum2 -= matrixA[row2][col2] * resultB[col2] ; 
                }
            }
            /* Store element of solution vector X */
            resultB[row2] = sum2 / matrixA[row2][row2] ;
        }

        /*
         * End of function 'FwdBackSubst'
         *
         */    

#if BMDEBUG    
        DebugOut( "    " ) ;    
        for( row2 = 0 ; row2 < nCount ; row2++ )
        {
            /* Show the resulting matrix 'B' */
            th_sprintf( szDebug, " %14e, ", resultB[row2] ) ; 
            DebugOut( szDebug ) ; 
        }
        DebugOut( "\n" ) ; 
#else
        for( row2 = 0 ; row2 < nCount ; row2++ )
        {
            WriteOut( resultB[row2] ) ;
        }
#endif /* BMDEBUG */

        /*
         * Next, calculate the determinant of matrix 'A' 
         *
         */    

        /* Calculate the determinant from the... */
        for( col2 = 0 ; col2 < nCount ; col2++ )   
        {
            /* ...decomposed matrix 'A' */
            determinant2 *= matrixA[col2][col2] ;
        }

#if BMDEBUG        
        th_sprintf( szDebug, "det= %14e\n", determinant2 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( determinant2 ) ; 
#endif /* BMDEBUG */

        /* Free up memory for the next pass */
        th_free( (void*)matrixA ) ;
        th_free( (void*)rowPtr ) ; 
        th_free( (void*)resultB ) ; 

#if BMDEBUG
        if( isTableLooped )
        {
            DebugOut( "END--END--END\n" ) ;
        }
#endif /* BMDEBUG */

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/

        /* Gets 'matrixA' and 'nCount' values from test data*/
        isTableLooped = GetInputValues() ;
        
        if( isTableLooped == -1 )
        {
            th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
                __FILE__, __LINE__ ) ; 
        }

#if BMDEBUG
        for( row3 = 0 ; row3 < nCount ; row3++ )
        {        
            /* Display the input row/column count */
            th_sprintf( szDebug, "%4d", row3 ) ;
            DebugOut( szDebug ) ; 

            /* ...and the input matrix 'A' */
            for( col3 = 0 ; col3 < nCount ; col3++ )
            {   th_sprintf( szDebug, " %14e, ", matrixA[row3][col3] ) ; 
                DebugOut( szDebug ) ; 
            }

            th_sprintf( szDebug, " %14e\n", resultB[row3] ) ; 
            DebugOut( szDebug ) ; 
        }
#endif /* BMDEBUG */

        /* First, perform LU decomposition on the input matrix 'A' */    
        maxRow3 = 0 ; 

        /* Allocate for the scaling vector */    
        scaleVector = (varsize *)th_malloc( sizeof( varsize ) * nCount ) ; 
        
        if( scaleVector == NULL )
        {
            th_exit( THE_OUT_OF_MEMORY, 
                "Cannot Allocate 'scaleVector' %s : %d",
                __FILE__, __LINE__ ) ; 
        }

        /* Really just the sign of the determinant( +/-1 )*/
        determinant3 = 1.0 ;
        
        /* Determine the implicit scaling of the input matrix */
        for( row3 = 0 ; row3 < nCount ; row3++ )
        {
            /* ...by finding the largest element */
            bigElmnt3 = 0.0 ;
            
            for( col3 = 0 ; col3 < nCount ; col3++ )
            {        
                element3 = fabs( matrixA[row3][col3] ) ; 
                if( element3 > bigElmnt3 )
                {
                    bigElmnt3 = element3 ; 
                }
            }
            
            /* Save the resulting 'scaleVector' */
            scaleVector[row3] = 1.0 / bigElmnt3 ;
            
        } /* End of 'for' to determine scaling */

        /* This is Crout's method */
        for( col3 = 0 ; col3 < nCount ; col3++ )
        {
            /* First, find the upper triangle */
            for( row3 = 0 ; row3 < col3 ; row3++ )
            {
                sum3 = matrixA[row3][col3] ; 

                for( k3 = 0 ; k3 < row3 ; k3++ )
                {
                    sum3 -= matrixA[row3][k3] * matrixA[k3][col3] ; 
                }

                matrixA[row3][col3] = sum3 ;
            }

            /* Next, search for the largest */
            bigElmnt3 = 0.0 ;
            
            /* ...pivot element */
            for( row3 = col3 ; row3 < nCount ; row3++ )
            {
                sum3 = matrixA[row3][col3] ; 

                for( k3 = 0 ; k3 < col3 ; k3++ )
                {
                    sum3 -= matrixA[row3][k3] * matrixA[k3][col3] ; 
                }

                matrixA[row3][col3] = sum3 ; 
                element3 = scaleVector[row3] * fabs( sum3 ) ; 

                /* Is this the best pivot element ? */
                if( element3 >= bigElmnt3 )
                {
                    /* Then remember it */
                    bigElmnt3 = element3 ;
                    maxRow3 = row3 ; 
                }
            }

            if( col3 != maxRow3 )
            {
                /* Need to interchange rows ? */
                for( k3 = 0 ; k3 < nCount ; k3++ )
                {
                    /* Yes, interchange the rows... */
                    element3 = matrixA[maxRow3][k3] ; 
                    matrixA[maxRow3][k3] = matrixA[col3][k3] ; 
                    matrixA[col3][k3] = element3 ; 
                }

                /* Including the result vector */
                element3 = resultB[maxRow3] ;    
                resultB[maxRow3] = resultB[col3] ; 
                resultB[col3] = element3 ; 
                element3 = scaleVector[maxRow3] ; 
                scaleVector[maxRow3] = scaleVector[col3] ; 
                scaleVector[col3] = element3 ; 
                
                /* Will flip the sign of the determinant */
                determinant3 = -determinant3 ;    
            }   

            if( matrixA[col3][col3] == 0.0 )
            {
                matrixA[col3][col3] = TINY_COEF ; 
            }

            if( col3 != ( nCount - 1 ) )
            {        
                element3 = 1.0 / matrixA[col3][col3] ; 
                for( row3 = ( col3 + 1 ) ; row3 < nCount ; row3++ )
                {
                    matrixA[row3][col3] *= element3 ; 
                }
            }
        } /* end for() Crout's method */

        /* Free up memory from 'scaleVector' */
        th_free( (void*)scaleVector ) ;

        /*
         * End of function 'MatrixDecomp' 
         *
         */    

#if BMDEBUG        
        for( row3 = 0 ; row3 < nCount ; row3++ )
        {        
            /* Display the input row, column count */
            th_sprintf( szDebug, "%4d", row3 ) ;
            DebugOut( szDebug ) ; 
            for( col3 = 0 ; col3 < nCount ; col3++ )
            {        
                /* ...and the decomposed matrix 'A' */
                th_sprintf( szDebug, " %14e, ", matrixA[row3][col3] ) ; 
                DebugOut( szDebug ) ; 
            }
            DebugOut( "\n" ) ; 
        }
#endif /* BMDEBUG */

        /*
         * Now, perform forward/backward substitution on the decomposition
         * of matrix 'A' 
         *
         */    

        ii3 = -1 ;

        /* Do the forward substitution */
        for( row3 = 0 ; row3 < nCount ; row3++ )
        {
            sum3 = resultB[row3] ; 
            if( ii3 != -1 )
            {        
                for( col3 = ii3 ; col3 <= ( row3 - 1 ) ; col3++ )
                {
                    sum3 -= matrixA[row3][col3] * resultB[col3] ; 
                }
            }
            else 
            {        
                if( sum3 != 0.0 )
                {
                    ii3 = row3 ; 
                }
            }
            resultB[row3] = sum3 ; 
        } /* end of forward substitution */

        /* Now do the back-substitution */
        for( row3 = ( nCount - 1 ) ; row3 >= 0 ; row3-- )
        {
            sum3 = resultB[row3] ; 
            if( row3 != ( nCount - 1 ) )
            {        
                for( col3 = ( row3 + 1 ) ; col3 < nCount ; col3++ )
                {
                    sum3 -= matrixA[row3][col3] * resultB[col3] ; 
                }
            }
            /* Store element of solution vector X */
            resultB[row3] = sum3 / matrixA[row3][row3] ;
        }

        /*
         * End of function 'FwdBackSubst'
         *
         */    

#if BMDEBUG    
        DebugOut( "    " ) ;    
        for( row3 = 0 ; row3 < nCount ; row3++ )
        {
            /* Show the resulting matrix 'B' */
            th_sprintf( szDebug, " %14e, ", resultB[row3] ) ; 
            DebugOut( szDebug ) ; 
        }
        DebugOut( "\n" ) ; 
#else
        for( row3 = 0 ; row3 < nCount ; row3++ )
        {
            WriteOut( resultB[row3] ) ;
        }
#endif /* BMDEBUG */

        /*
         * Next, calculate the determinant of matrix 'A' 
         *
         */    

        /* Calculate the determinant from the... */
        for( col3 = 0 ; col3 < nCount ; col3++ )   
        {
            /* ...decomposed matrix 'A' */
            determinant3 *= matrixA[col3][col3] ;
        }

#if BMDEBUG        
        th_sprintf( szDebug, "det= %14e\n", determinant3 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %08lX\n", ( unsigned long )loop_cnt ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( determinant3 ) ; 
        WriteOut( loop_cnt ) ; 
        k1 = 55555 ; 
        WriteOut( k1 ) ;
#endif /* BMDEBUG */

        /* Free up memory for the next pass */
        th_free( (void*)matrixA ) ;
        th_free( (void*)rowPtr ) ; 
        th_free( (void*)resultB ) ; 

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
/*NON_INTRUSIVE_CRC_CHECK only on stimuli, not ok */
#if NON_INTRUSIVE_CRC_CHECK
/* Final results iteration dependent */
	tcdef->CRC=0;
	/* To Handle benchmark error reported by EEMBC */
	e_u32 toCheck;
	varsize loopVal;

	for (loop_cnt=0;loop_cnt<NUM_TESTS;loop_cnt++) {
        /*toCheck = *(e_u32*)&(inpVariable[loop_cnt*4]);*/
		loopVal = inpVariable[loop_cnt];
		toCheck = *(e_u32*)&loopVal;
		/*tcdef->CRC = Calc_crc32((e_u32)inpVariable[loop_cnt],tcdef->CRC);*/
		tcdef->CRC = Calc_crc32(toCheck,tcdef->CRC);
    }
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
n_int benchIter;
n_int failTest;
int main(int argc, const char* argv[] )
{
    init_platform();

    /*Xil_DCacheDisable();*/
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
    xil_printf(">>     Start of Matrix Mul...\n\r");
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
            th_free(inpVariableFree); /* Free inpVariable in algotst.c */
            xil_printf("%20d\n\r",benchIter++);
            
        }
    }
    xil_printf(">>     Matrix Mul test is finished\n\r");
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
