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
 *          $Date: 2002/08/07 22:21:47 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/tblook01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.14  2002/08/07 22:21:47  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.13  2002/07/22 21:59:33  rick
 * General cleanup Beta 2b
 *
 * Revision 1.12  2002/07/22 16:10:17  rick
 * Fix iteration dependant NI CRC
 *
 * Revision 1.11  2002/07/11 22:13:33  rick
 * Initialize tcdef results
 *
 * Revision 1.10  2002/07/10 19:01:27  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.9  2002/05/29 22:25:34  rick
 * Set recommended iterations with make
 *
 * Revision 1.8  2002/05/10 23:57:47  rick
 * Fix missing shifts in 16, and 32 bit CRC calculations
 *
 * Revision 1.7  2002/05/10 17:20:37  rick
 * Add al_main to API
 *
 * Revision 1.6  2002/04/25 20:10:45  rick
 * sprintf to th_sprintf
 *
 * Revision 1.5  2002/04/19 18:31:38  rick
 * Bug #146: global tablecount uninitialized
 *
 * Revision 1.4  2002/03/12 18:31:05  rick
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
#define ITERATIONS 10000	/* required iterations for crc */
#else
#define ITERATIONS 10000	/* recommended iterations for benchmark */
#endif
#endif

/* Pre-caclculated CRC values */
#if	CRC_CHECK
#define EXPECTED_CRC 0xa469
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC 0x9c19
#else
#define EXPECTED_CRC 0x0000
#endif

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/

TCDef the_tcdef = 
{
    "AUT tblook01   ", 
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Algorithm #1 -- Table Lookup V1.0G0 - TBLOOK01", 
    TCDEF_REVISION, 
    { EEMBC_TH_MAJOR, EEMBC_TH_MINOR, EEMBC_TH_STEP, EEMBC_TH_REVISION }, 
    { 0, 0, 0, 0 },   /* Target Hardware Version Number Required( make all zeros to ignore )*/
    { 1, 0, 'G', 0 }, /* The Version number of this Benchmark */
	ITERATIONS,    /* recomended iterations for benchmark score */
	0,
	0,
	0,
	0,
	0,
	0,
	0
} ; 

extern const varsize engLoadROM[] ;      /* Linear array of 'y' axis */
extern const varsize engSpeedROM[] ;     /* Linear array of 'x' axis */
extern const varsize angleTableROM[] ;   /* 2D array of f( x, y )= 'angle' */

varsize numXEntries ;           /* The table 'x' size spec'd here */
varsize numYEntries ;           /* The table 'y' size spec'd here */
n_int   *RAMfile ;              /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;           /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree; /* NOT PART OF BENCHMARK to free RAMfile */

varsize loadValue ;             /* 'Load' pulled from test data */
varsize speedValue ;            /* 'Speed' pulled from test data */
varsize *inpLoadValue ;         /* Array of 'load' test data */
varsize *inpSpeedValue ;        /* Array of 'speed' test data */

const varsize *engSpeed ;       /* Linear array of 'y' axis */
const varsize *engLoad ;        /* Linear array of 'x' axis */
const varsize *angleTable ;     /* 2D array of f( x, y )= 'angle' */

/* >> IMPORTANT NOTE << 
//
// Since benchmarks can be entered( run )multiple times, the benchmark
// MUST NOT depend on global data being initialized.  E.g. it must
// complelty initialize itself EVERY TIME its t_run_test()function
// is called.
// 
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
        "Algorithm 1 :  Table Lookup & Interpolation Rev. 1.0\n" ; 
    char *szHeader = 
        "\n\nload, speed, engLoadLow, engLoadHi, engSpeedLow, engSpeedHi, "
        " engLoadDelta, engSpeedDelta, angle, counter\n" ; 
    char szDebug[100] ; 
#endif /* BMDEBUG */

    static n_char szDataSize[40] ; 
    varsize i1, i2, i3, j1, j2, j3 ;    /* Local indices and counters */
    static varsize outAngleValue1 ;     /* 'Angle' pulled from map */
    static n_float engSpeedDelta1 ;       /* Intermediate ratios from grid */
    static n_float engLoadDelta1 ;
    static varsize outAngleValue2 ;     /* 'Angle' pulled from map */
    static n_float engSpeedDelta2 ;       /* Intermediate ratios from grid */
    static n_float engLoadDelta2 ;
    static varsize outAngleValue3 ;     /* 'Angle' pulled from map */
    static n_float engSpeedDelta3 ;       /* Intermediate ratios from grid */
    static n_float engLoadDelta3 ;    
    n_int isTableLooped = false ;         /* Input test data table looped */
    tableCount = 0 ;    /* Start out at beginning of input test data */

    /* Unused */
    argc = argc ;
    argv = argv ;

    /*
     * First, initialize the data structures we need for the test
     * and allocate memory as needed.  Report an error if we can't.
     */

    /* If debug output is desired, then must allocate some RAM... */

    RAMfilePtr = 0 ;    /* Point to beginning of test output file */

     /* Set size of output file (1K) */
    RAMfileSize = MAX_FILESIZE; 

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

    /* Initialize the test data -- stimuli for the algorithm. */
    if( !GetTestData() )            /* Allocate for the test input data table */

    th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
        __FILE__, __LINE__ ) ; 

    numXEntries = NUM_X_ENTRIES ;   /* ...specify the num of table entries */
    numYEntries = NUM_Y_ENTRIES ; 
    engSpeed = engSpeedROM ;        /* Point to the constant linear tables */
    engLoad = engLoadROM ; 
    angleTable = angleTableROM ;    /* ...and to the 3D table */

      tcdef->CRC = 0;
    th_signal_start() ;             /* Tell the host that the test has begun */

#if BMDEBUG
    DebugOut( szTitle ) ;           /* Print the title message in RAM file */
    DebugOut( szDataSize ) ;        /* ...and the data size */
    DebugOut( szHeader ) ;          /* Print the output file hdr in RAM file */
#endif /* BMDEBUG */

    /* This is the actual benchmark algorithm.
     *
     * Perform table interpolation using 'bilinear interpolation'
     * technique.  This technique determines the interpolated value by
     * using the four points in a grid which surrounds the desired point, 
     * essentially defining a planar surface, and presumes that the resulting 
     * linear approximation of the point in the plane is close enough to the 
     * actual value to be useful.  However, this technique does not account 
     * for the curvature of a surface and suffers from step-changes at grid 
     * boundaries. This technique is illustrated in 
     *      "Numerical Recipes in C", pp. 104-105.
     */
    for( loop_cnt = 0 ; loop_cnt < tcdef->rec_iterations ; loop_cnt++ ) /* no stopping! */
    {
        /* 'engine load' and 'engine speed' index us into the 'angle' table */
#if BMDEBUG
        if( isTableLooped ) 
        {
            DebugOut( "END--END--END\n" ) ;        
        }
#endif /* BMDEBUG */

        /***********************************************************************
            First Pass                                                          
        ***********************************************************************/
        /* Gets 'loadValue' and 'speedValue' */
        isTableLooped = GetInputValues() ;  

#if BMDEBUG
        th_sprintf( szDebug, "%4ld, %4ld", (long)loadValue, (long)speedValue ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*  First, find the 'x' axis grid coefficients */
        for( i1 = 0 ; i1 < ( numXEntries - 1 ) ; i1++ )
        {
            if( ( loadValue < engLoad[i1+1] ) && ( loadValue >= engLoad[i1] ) ) 
            {
                break ; 
            }
        }

        if( i1 == ( numXEntries - 1 ) ) 
        {
            loadValue = engLoad[i1] ; 
        }

        /*  
         * Use the 'x' coefficients to calculate the proportional delta
         *  such that 'engLoadDelta' is in the range 0..1
         */
        engLoadDelta1 = (n_float)( loadValue - engLoad[i1] ) /
                        (n_float)( engLoad[i1 + 1] - engLoad[i1] ) ; 

#if BMDEBUG
        th_sprintf( szDebug, ", %3ld, %3ld", (n_long)i1, (n_long)i1+1 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*  Next, find the 'y' axis grid coefficients */
        for( j1 = 0 ; j1 < ( numYEntries-1 ) ; j1++ )
        {
            if( ( speedValue < engSpeed[j1+1] ) && 
                ( speedValue >= engSpeed[j1] ) ) 
            {
                break ; 
            }
        }

        if( j1 == ( numYEntries - 1 ) )
        {
            speedValue = engSpeed[j1] ; 
        }

        /*  
         * Use the 'y' coefficients to calculate the proportional grid delta
         * such that 'engSpeedDelta' is in the range 0..1.
         */
        engSpeedDelta1 = (n_float)( speedValue - engSpeed[j1] ) / 
                         (n_float)( engSpeed[j1 + 1] - engSpeed[j1] ) ; 

#if BMDEBUG
        th_sprintf( szDebug, ", %3ld, %3ld, %f, %f, ", (n_long)j1, (n_long)( j1 + 1 ),
            engLoadDelta1, engSpeedDelta1 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* 
         * Now we can determine the interpolated output value for the input 
         * data from the table 'z' values.
         */
        outAngleValue1 = (varsize)( 
            ( ( 1.0 - engLoadDelta1 )*( 1.0 - engSpeedDelta1 ) *
              (n_float)angleTable[ i1 + ( j1 * numXEntries ) ] ) +
            ( engLoadDelta1 * ( 1.0 - engSpeedDelta1 ) * 
              (n_float)angleTable[ ( i1 + 1 ) + ( j1 * numXEntries ) ] ) +
            ( engLoadDelta1 * engSpeedDelta1 * 
              (n_float)angleTable[ ( i1 + 1 ) + ( numXEntries * ( j1 + 1 ) ) ] ) +
            ( ( 1.0 - engLoadDelta1 )* engSpeedDelta1 *
              (n_float)angleTable[ i1 + ( numXEntries * ( j1 + 1 ) ) ] ) ) ; 

        /* Display the 'outAngleValue', or just put it in RAM */
#if BMDEBUG
        th_sprintf( szDebug, " %6ld\n", (n_long)outAngleValue1 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( outAngleValue1 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/
        /* Gets 'loadValue' and 'speedValue' */
        isTableLooped += GetInputValues() ;

#if BMDEBUG
        th_sprintf( szDebug, "%4ld, %4ld", (n_long)loadValue, (n_long)speedValue ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*  First, find the 'x' axis grid coefficients */
        for( i2=0 ; i2 < ( numXEntries - 1 ) ; i2++ )
        {
            if( ( loadValue < engLoad[i2 + 1] ) && 
                ( loadValue >= engLoad[i2] ) ) 
            {
                break ; 
            }
        }

        if( i2 == ( numXEntries-1 ) )
        {
            loadValue = engLoad[i2] ;
        }

        /*
         * Use the 'x' coefficients to calculate the proportional delta
         * such that 'engLoadDelta' is in the range 0..1 
         */
        engLoadDelta2 = (n_float)( loadValue - engLoad[i2] ) /
                        (n_float)( engLoad[i2 + 1] - engLoad[i2] ) ; 

#if BMDEBUG
        th_sprintf( szDebug, ", %3ld, %3ld", (n_long)i2, (n_long)(i2 + 1) ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*  Next, find the 'y' axis grid coefficients */
        for( j2=0 ; j2<( numYEntries-1 ) ; j2++ )
        {
            if( ( speedValue < engSpeed[j2 + 1] ) && 
                ( speedValue >= engSpeed[j2] ) )
            {
                break ; 
            }
        }

        if( j2 == ( numYEntries - 1 ) )
        {
            speedValue = engSpeed[j2] ; 
        }

        /*  
         * Use the 'y' coefficients to calculate the proportional grid delta
         * such that 'engSpeedDelta' is in the range 0..1.
         */
        engSpeedDelta2 = (n_float)( speedValue - engSpeed[j2] ) / 
                         (n_float)( engSpeed[j2 + 1] - engSpeed[j2] ) ; 

#if BMDEBUG
        th_sprintf( szDebug, ", %3ld, %3ld, %f, %f, ", (n_long)j2, (n_long)(j2 + 1),
            engLoadDelta2, engSpeedDelta2 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*
         * Now we can determine the interpolated output value for the input 
         * data from the table 'z' values.
         */
        outAngleValue2 = (varsize)( 
            ( ( 1.0 - engLoadDelta2 ) * ( 1.0 - engSpeedDelta2 ) *
              (n_float)angleTable[ i2 + ( j2 * numXEntries ) ] ) +
            ( engLoadDelta2 * ( 1.0 - engSpeedDelta2 ) * 
              (n_float)angleTable[ ( i2 + 1 ) + ( j2 * numXEntries ) ] ) +
            ( engLoadDelta2 * engSpeedDelta2 *
              (n_float)angleTable[ ( i2 + 1 )+( numXEntries * ( j2+1 ) ) ] ) +
            ( ( 1.0 - engLoadDelta2 ) * engSpeedDelta2 *
              (n_float)angleTable[ i2 + ( numXEntries * ( j2 + 1 ) ) ] ) ) ; 

        /* Display the 'outAngleValue', or just put it in RAM */
#if BMDEBUG
        th_sprintf( szDebug, " %6ld\n", (n_long)outAngleValue2 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( outAngleValue2 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/
        /* Gets 'loadValue' and 'speedValue' */
        isTableLooped += GetInputValues() ;

#if BMDEBUG
        th_sprintf( szDebug, "%4ld, %4ld", (n_long)loadValue, (n_long)speedValue ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*  First, find the 'x' axis grid coefficients */
        for( i3=0 ; i3 < ( numXEntries - 1 ) ; i3++ )
        {
            if( ( loadValue < engLoad[i3 + 1] ) && 
                ( loadValue >= engLoad[i3] ) )
            {
                break ; 
            }
        }

        if( i3 == ( numXEntries-1 ) )
        {
            loadValue = engLoad[i3] ; 
        }

        /*
         * Use the 'x' coefficients to calculate the proportional delta
         * such that 'engLoadDelta' is in the range 0..1 
         */
        engLoadDelta3 = (n_float)( loadValue - engLoad[i3] ) / 
                        (n_float)( engLoad[i3 + 1] - engLoad[i3] ) ; 

#if BMDEBUG
        th_sprintf( szDebug, ", %3ld, %3ld", (n_long)i3, (n_long)i3+1 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*  Next, find the 'y' axis grid coefficients */
        for( j3=0 ; j3 < ( numYEntries - 1 ) ; j3++ )
        {
            if( ( speedValue < engSpeed[j3 + 1] ) && 
                ( speedValue >= engSpeed[j3] ) )
            {
                break ; 
            }
        }

        if( j3 == ( numYEntries - 1 ) )
        {
            speedValue = engSpeed[j3] ; 
        }

        /*
         * Use the 'y' coefficients to calculate the proportional grid delta
         * such that 'engSpeedDelta' is in the range 0..1
         */
        engSpeedDelta3 = (n_float)( speedValue - engSpeed[j3] ) / 
                         (n_float)( engSpeed[j3 + 1] - engSpeed[j3] ) ; 

#if BMDEBUG
        th_sprintf( szDebug, ", %3ld, %3ld, %f, %f, ", (n_long)j3, (n_long)( j3+1 ), 
            engLoadDelta3, engSpeedDelta3 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*
         * Now we can determine the interpolated output value for the input 
         * data from the table 'z' values 
         */
        outAngleValue3 = (varsize)( 
            ( ( 1.0 - engLoadDelta3 ) * ( 1.0 - engSpeedDelta3 ) *
              (n_float)angleTable[ i3 +( j3 * numXEntries ) ] ) +
            ( engLoadDelta3 * ( 1.0 - engSpeedDelta3 ) * 
              (n_float)angleTable[ ( i3+1 ) + ( j3 * numXEntries ) ] ) +
            ( engLoadDelta3 * engSpeedDelta3 *
              (n_float)angleTable[ ( i3 + 1 ) + ( numXEntries * ( j3 + 1 ) ) ] ) +
            ( ( 1.0 - engLoadDelta3 ) * engSpeedDelta3 * 
              (n_float)angleTable[ i3 + ( numXEntries * ( j3 + 1 ) ) ] ) ) ; 

        /* Display the 'outAngleValue', or just put it in RAM */
#if BMDEBUG
        th_sprintf( szDebug, " %6ld", (n_long)outAngleValue3 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %08lX\n", (n_long)loop_cnt ) ; 
        DebugOut( szDebug ) ; 
#else      
        WriteOut( outAngleValue3 ) ; 

#if DATA_SIZE == 0      /* Might break up the loop counter */
        i3 = (varsize)loop_cnt & 0x0000FFFF ; 
        WriteOut( i3 ) ;   /* ...in the output file */
        i3 = (varsize)loop_cnt >> 16 ; 
        WriteOut( i3 ) ; 
#else
        WriteOut( loop_cnt ) ; 
#endif
        i3 = (varsize)0xAAAA ; 
        WriteOut( i3 ) ;  /* Flag the end of data-block */
#endif /* BMDEBUG */

    }
    
    /* Signal that we are finished */
	tcdef->duration = th_signal_finished() ;
    tcdef->iterations = loop_cnt ; 
    tcdef->v1         = 0 ; 
    tcdef->v2         = 0 ; 
    tcdef->v3         = 0 ; 
    tcdef->v4         = 0 ; 

#if NON_INTRUSIVE_CRC_CHECK
/* Final results iteration dependent */
	tcdef->CRC=0;
	for( i1 = 0 ; i1 < ( numXEntries - 1 ) ; i1++ )
		tcdef->CRC = Calc_crc32((e_u32)engLoad[i1],tcdef->CRC);
	for( j1 = 0 ; j1 < ( numYEntries-1 ) ; j1++ )
		tcdef->CRC = Calc_crc32((e_u32)engSpeed[j1],tcdef->CRC);
#elif	CRC_CHECK
/* Iteration dependant results check */
	tcdef->CRC = 0;
	tcdef->CRC = Calc_crc32((e_u32)outAngleValue1,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)outAngleValue2,tcdef->CRC); 
	tcdef->CRC = Calc_crc32((e_u32)outAngleValue3,tcdef->CRC);
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
    xil_printf(">>     Start of tblook...\n\r");
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
            th_free(inpLoadValue);
            th_free(inpSpeedValue);

            xil_printf(">>   Test is working just fine, iteration: %8ld\n\r",benchIter++);
        }
    }
    xil_printf(">>      tblook test is finished\n\r");
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

