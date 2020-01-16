/*==============================================================================
 *$RCSfile: bmark_lite.c,v $
 *
 *   DESC : This file contains the Test Main and other TH support functions
 *
 * AUTHOR : ARM, Ltd., ECL, LLC
 *
 *  EEMBC : Automotive/Industrial Subcommittee 
 *
 *    CVS : $Revision: 1.18 $
 *          $Date: 2002/08/07 22:20:25 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/a2time01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.18  2002/08/07 22:20:25  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.17  2002/07/22 21:58:53  rick
 * General cleanup Beta 2b
 *
 * Revision 1.16  2002/07/22 16:09:49  rick
 * Fix iteration dependant NI CRC
 *
 * Revision 1.15  2002/07/18 23:33:45  rick
 * Fix iteration dependencies in NI CRC
 *
 * Revision 1.14  2002/07/11 22:13:26  rick
 * Initialize tcdef results
 *
 * Revision 1.13  2002/07/10 19:00:42  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.12  2002/05/29 22:24:46  rick
 * Set recommended iterations with make
 *
 * Revision 1.11  2002/05/10 23:57:44  rick
 * Fix missing shifts in 16, and 32 bit CRC calculations
 *
 * Revision 1.10  2002/05/10 17:20:36  rick
 * Add al_main to API
 *
 * Revision 1.9  2002/04/25 20:10:43  rick
 * sprintf to th_sprintf
 *
 * Revision 1.8  2002/04/19 18:31:37  rick
 * Bug #146: global tablecount uninitialized
 *
 * Revision 1.7  2002/03/12 18:31:04  rick
 * ITERATIONS, CRC_CHECK, NON_INTRUSIVE_CRC_CHECK, add standards headers
 *
 * Revision 1.6  2002/03/11 22:11:47  rick
 * ITERATIONS, CRC_CHECK, NON_INTRUSIVE TCDef Usage
 *
 * Revision 1.5  2002/03/08 00:58:51  rick
 * Repair Iterations and CRC ifdef logic
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
#define ITERATIONS 50000	/* required iterations for crc */
#else
#define ITERATIONS 50000	/* recommended iterations for benchmark */
#endif
#endif

/* Pre-caclculated CRC values */
#if	CRC_CHECK
#define EXPECTED_CRC 0x0000
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC 0xc74a
#else
#define EXPECTED_CRC 0x0000
#endif

TCDef the_tcdef = 
{
    "AUT a2time01   ",
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Algorithm #2 -- Angle-To-Time Conversion V1.0E0 - a2time00",
    TCDEF_REVISION, 
    { EEMBC_TH_MAJOR, EEMBC_TH_MINOR, EEMBC_TH_STEP, EEMBC_TH_REVISION }, 
    { 0, 0, 0, 0 },   /* Target Hardware Version Number Required( make all zeros to ignore )*/
    { 1, 0, 'C', 0 }, /* The Version number of this Benchmark */
	ITERATIONS,
	0,
	0,
	0,
	0,
	0,
	0,
	0
} ; 

/*  DECLARATIONS */    
n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree; /* NOT PART OF BENCHMARK to free RAMfile */

varsize angleCounter ;      /* Current 'angleCounter' pulled from test data */
varsize *inpAngleCount ;    /* Array of 'angleCounter' test data values */
varsize tonewheelTeeth ;    /* Number of teeth on the tonewheel */


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
        "Algorithm 2 : Angle-to-Time Conversion  Rev. 1.0E0 a2time00 \n" ; 
    char *szHeader = 
        "\n\n   angle  pulse          tdc   engine "
        "tooth  delta        fire   test\n"
        "   Count, DeltaT, TDC,   Time, Speed, "
        "Count, TimeAvg,     Time   counter\n" ; 
    char szDebug[100] ; 
#else
    varsize i ;
#endif /* BMDEBUG */

    static n_char szDataSize[40] ; 
    n_int isTableLooped = false ;    /* Input test data table looped */
    varsize window ; 
    static varsize pulseDeltaTime1 ;
    static varsize pulseDeltaTime2 ;
    static varsize pulseDeltaTime3 ;
    static varsize angle1 ;
    static varsize angle2 ;
    static varsize angle3 ; 
    static varsize angleCounterLast1 ;
    static varsize angleCounterLast2 ;
    static varsize angleCounterLast3 ; 
    static varsize toothCount1 ;
    static varsize toothCount2 ;
    static varsize toothCount3 ; 
    static varsize deltaTimeAccum1 ;
    static varsize deltaTimeAccum2 ;
    static varsize deltaTimeAccum3 ; 
    static varsize deltaTimeAvg1 ;
    static varsize deltaTimeAvg2 ;
    static varsize deltaTimeAvg3 ; 
    static varsize firingTime1 ;
    static varsize firingTime2 ;
    static varsize firingTime3 ; 
    static n_ulong tdcTime1 ;
    static n_ulong tdcTime2 ;
    static n_ulong tdcTime3 ; 
    static n_ulong engineSpeed1 ;
    static n_ulong engineSpeed2 ;
    static n_ulong engineSpeed3 ; 
    static n_ulong rotationTime1 ;
    static n_ulong rotationTime2 ;
    static n_ulong rotationTime3 ; 
    static n_int isTopDeadCenter1 ; /* TRUE/FALSE flag when TDC occurs */
    static n_int isTopDeadCenter2 ; /* TRUE/FALSE flag when TDC occurs */
    static n_int isTopDeadCenter3 ; /* TRUE/FALSE flag when TDC occurs */

    /* Unused */
    argc = argc ;
    argv = argv ;
    
    /*
     * First, initialize the data structures we need for the test
     * and allocate memory as needed.  Report an error if we can't.
     *
     */    

    /* Variable initializations at t=0 */    
    toothCount1 = 0 ;       /* Don't know which pulse we start on */
    toothCount2 = 0 ; 
    toothCount3 = 0 ; 
    deltaTimeAccum1 = 0 ;   /* ...and haven't accumulated for filter... */
    deltaTimeAccum2 = 0 ; 
    deltaTimeAccum3 = 0 ; 
    deltaTimeAvg1 = 32767 ; /* ...and not gotten an average... */
    deltaTimeAvg2 = 32767 ; 
    deltaTimeAvg3 = 32767 ; 
    tdcTime1 = 0 ;          /* ...and don't know when TDC occurs */
    tdcTime2 = 0 ; 
    tdcTime3 = 0 ; 
    angleCounterLast1 = 0 ; 
    angleCounterLast2 = 0 ; 
    angleCounterLast3 = 0 ; 
    engineSpeed1 = 0 ; 
    engineSpeed2 = 0 ; 
    engineSpeed3 = 0 ; 
    rotationTime1 = 0 ; 
    rotationTime2 = 0 ; 
    rotationTime3 = 0 ; 
    firingTime1 = 0 ; 
    firingTime2 = 0 ; 
    firingTime3 = 0 ; 
/* ALGO_GLOBALS */
    tableCount		= 0;
	angleCounter	= 0;   /* Current 'angleCounter' pulled from  data */
	inpAngleCount	= NULL; /* Array of 'angleCounter' test data values */
	tonewheelTeeth	= 0; /* Number of teeth on the tonewheel */

    /* Set size of output file (1K) */    
    RAMfileSize = MAX_FILESIZE ;

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
    /* Allocate for the test input data table */
    if( !GetTestData() ) 
    {
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d", 
            __FILE__, __LINE__ ) ; 
    }

    th_signal_start() ;    /* Tell the host that the test has begun */

#if BMDEBUG
    DebugOut( szTitle ) ;    /* Print the title message in RAM file */
    DebugOut( szDataSize ) ; /*  and the data size */
    DebugOut( szHeader ) ;   /* Print the test output file header in RAM file */
#endif     /* BMDEBUG */

    window = TENTH_DEGREES / tonewheelTeeth ; /* Only need to do this once */

    /* This is the actual benchmark algorithm. */    

    for( loop_cnt = 0 ; loop_cnt < tcdef->rec_iterations ; loop_cnt++ ) /* no stopping! */
    {

#if BMDEBUG
        if( isTableLooped )
        {
            DebugOut( "END--END--END\n" ) ;    /* Flag end-of-testfile */
        }
#endif /* BMDEBUG */

        /***********************************************************************
            First Pass                                                          
        ***********************************************************************/

        /* Gets 'angleCounter' value from test data */
        isTableLooped = GetInputValues() ;

#if BMDEBUG
        th_sprintf( szDebug, "%8ld", (n_long)angleCounter ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*
         * Compute 'pulseDeltaTime' -- the difference in counter 
         * readings from the last pulse to the current one.  Note that
         * the realtime counter may roll over, so the elapsed time may
         * need to be corrected when this occurs.
         *
         */    
        if( angleCounterLast1 > angleCounter ) 
        {
            pulseDeltaTime1 = angleCounter + 
                ( (varsize)MAX_VARIABLE - angleCounterLast1 + 1 ) ; 
        }
        else
        {
            pulseDeltaTime1 = angleCounter - angleCounterLast1 ; 
        }

        /* Update timer history... */
        angleCounterLast1 = angleCounter ;
        /* ...and elapsed time for a revolution */
        rotationTime1 += pulseDeltaTime1 ; 

#if BMDEBUG
        th_sprintf( szDebug, ", %4ld", (n_long)pulseDeltaTime1 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*
         * Must detect TDC reference by noticing that the period between this
         * pulse and the last one is two or three times normal.  We'll set a
         * flag when TDC reference occurs, and clear it on all other pulses.
         * We also keep count of which pulse we're on relative to TDC reference.
         *
         */    

        if( pulseDeltaTime1 > ( TDC_TEETH *deltaTimeAvg1 *TDC_MARGIN ) )
        {
            isTopDeadCenter1 = true ; 
            pulseDeltaTime1 /= TDC_TEETH ; 
    
            /*
             * Compute engine speed every TDC.  Engine speed will be
             * the inverse of 'tdcTime', which is the period( in CPU
             * time )between TDC's.  Engine speed is also scaled by
             * an arbitrary constant to make it useful elsewhere in
             * the engine controller.
             *
             */    
            tdcTime1 = rotationTime1 ; 
            rotationTime1 = 0 ; 
            engineSpeed1 = RPM_SCALE_FACTOR / tdcTime1 ; 
            toothCount1 = 0 ; 
        }
        else 
        {
            toothCount1++ ; 
            isTopDeadCenter1 = false ; 
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %4d, %8ld, %5ld, %4ld", ( isTopDeadCenter1 & 1 ),
            (n_long)tdcTime1, (n_long)engineSpeed1, (n_long)toothCount1 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( (varsize)engineSpeed1 ) ; 
#endif /* BMDEBUG */

        /*
         * Compute an average delta-T for TDC pulse discrimination.
         * The delta-T will be filtered by averaging over the period
         * of one cylinder( several pulses ).
         *
         */    

        deltaTimeAccum1 += pulseDeltaTime1 ; 
        if( ( toothCount1 > 0 ) && 
            ( toothCount1 %( tonewheelTeeth / CYLINDERS ) == 0 ) )
        {        
            deltaTimeAvg1 = deltaTimeAccum1 / ( tonewheelTeeth / CYLINDERS ) ; 
            deltaTimeAccum1 = 0 ; 
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %6ld, ", (n_long)deltaTimeAvg1 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        angle1 = ( TENTH_DEGREES * toothCount1 / tonewheelTeeth ) ; 

        /*
         * Now, output a value for the firing angle timer( a one-shot )
         * only if we're on the tooth which precedes the firing
         * angle for one of the cylinders.  We presume that there is
         * always a tooth which precedes each cylinder's firing angle.
         * The value which is output presumably goes to a "capture/compare"
         * timer which generates an interrupt used to fire that cylinder.
         * Note the special treatment for the last cylinder( #4, #6, or #8 ), 
         * we don't subtract the 'angle' because the 360th degree of rotation
         * is the same as the 0th degree of the next rotation.
         *
         */    

        /* CYLINDER 1 */    
        if( ( angle1 >= ( ( CYL1 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle1 < ( CYL1 * TENTH_DEGREES / CYLINDERS ) ) )
        {
            firingTime1 = 
                ( ( FIRE1_ANGLE - angle1 ) * tdcTime1 / TENTH_DEGREES ) +
                angleCounter ; 
        }

        /* CYLINDER 2 */    
        if( ( angle1 >= ( ( CYL2 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle1 < ( CYL2 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime1 = 
                ( ( FIRE2_ANGLE - angle1 ) * tdcTime1 / TENTH_DEGREES ) +
                angleCounter ; 
        }

        /* CYLINDER 3 */    
        if( ( angle1 >= ( ( CYL3 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle1 < ( CYL3 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime1 = 
                ( ( FIRE3_ANGLE - angle1 ) * tdcTime1 / TENTH_DEGREES ) +
                angleCounter ; 
        }

        /* CYLINDER 4 */    
        if( ( angle1 >= ( ( CYL4 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle1 < ( CYL4 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime1 = 
                ( ( FIRE4_ANGLE - angle1 ) * tdcTime1 / TENTH_DEGREES ) +
                angleCounter ; 
        }

#if( CYLINDERS > 4 )

        /* CYLINDER 5 */    
        if( ( angle1 >= ( ( CYL5 * TENTH_DEGREES / CYLINDERS ) - window ) ) &&
            ( angle1 < ( CYL5 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime1 = 
                ( ( FIRE5_ANGLE - angle1 ) * tdcTime1 / TENTH_DEGREES ) +
                angleCounter ; 
        }

        /* CYLINDER 6 */    
        if( ( angle1 >= ( ( CYL6 * TENTH_DEGREES / CYLINDERS ) - window ) ) &&
            ( angle1 < ( CYL6 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime1 = 
                ( ( FIRE6_ANGLE - angle1 ) * tdcTime1 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

#if( CYLINDERS > 6 )

        /* CYLINDER 7 */    
        if( ( angle1 >= ( ( CYL7 * TENTH_DEGREES / CYLINDERS ) - window ) ) &&
            ( angle1 < ( CYL7 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime1 = 
                ( ( FIRE7_ANGLE - angle1 ) * tdcTime1 / TENTH_DEGREES ) +
                angleCounter ; 
        }

        /* CYLINDER 8 */    
        if( ( angle1 >= ( ( CYL8 * TENTH_DEGREES / CYLINDERS ) - window ) ) &&
            ( angle1 < ( CYL8 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime1 = 
                ( ( FIRE8_ANGLE - angle1 ) * tdcTime1 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

#endif /* 6 cylinders */
#endif /* 4 cylinders */

        if( firingTime1 > MAX_VARIABLE )
        {
            firingTime1 -= MAX_VARIABLE ; 
        }

        /* Output the 'firingTime result */    

#if BMDEBUG
        th_sprintf( szDebug, " %10ld\n", (n_long)firingTime1 ) ; 
        DebugOut( szDebug ) ; 
#else      
        WriteOut( (varsize)firingTime1 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/
        /* Gets 'angleCounter' value from test data*/
        isTableLooped += GetInputValues() ;

#if BMDEBUG
        /* Output some debug info, if needed */
        th_sprintf( szDebug, "%8ld", (n_long)angleCounter ) ;
        DebugOut( szDebug ) ; 
#endif     /* BMDEBUG */


        /*
         * Compute 'pulseDeltaTime' -- the difference in counter 
         * readings from the last pulse to the current one.  Note that
         * the realtime counter may roll over, so the elapsed time may
         * need to be corrected when this occurs.
         *
         */    

        if( angleCounterLast2 > angleCounter )
        {
            pulseDeltaTime2 = angleCounter +
                ( (varsize)MAX_VARIABLE - angleCounterLast2 + 2 ) ; 
        }
        else
        {
            pulseDeltaTime2 = angleCounter - angleCounterLast2 ; 
        }

        /* Update timer history... */
        angleCounterLast2 = angleCounter ;    
        /* ...and elapsed time for a revolution */
        rotationTime2 += pulseDeltaTime2 ;    

#if BMDEBUG
        th_sprintf( szDebug, ", %4ld", (n_long)pulseDeltaTime2 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*
         * Must detect TDC reference by noticing that the period between this
         * pulse and the last one is two or three times normal.  We'll set a
         * flag when TDC reference occurs, and clear it on all other pulses.
         * We also keep count of which pulse we're on relative to TDC reference.
         *
         */    

        if( pulseDeltaTime2 > ( TDC_TEETH * deltaTimeAvg2 * TDC_MARGIN ) )
        {        
            isTopDeadCenter2 = true ; 
            pulseDeltaTime2 /= TDC_TEETH ; 

            /*
             * Compute engine speed every TDC.  Engine speed will be
             * the inverse of 'tdcTime', which is the period( in CPU
             * time )between TDC's.  Engine speed is also scaled by
             * an arbitrary constant to make it useful elsewhere in
             * the engine controller.
             *
             */    


            tdcTime2 = rotationTime2 ; 
            rotationTime2 = 0 ; 
            engineSpeed2 = RPM_SCALE_FACTOR / tdcTime2 ; 
            toothCount2 = 0 ; 
        }
        else {        
            toothCount2++ ; 
            isTopDeadCenter2 = false ; 
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %4d, %8ld, %5ld, %4ld", ( isTopDeadCenter2 & 1 ),
            (n_long)tdcTime2, (n_long)engineSpeed2, (n_long)toothCount2 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( (varsize)engineSpeed2 ) ; 
#endif /* BMDEBUG */

        /*
         * Compute an average delta-T for TDC pulse discrimination.
         * The delta-T will be filtered by averaging over the period
         * of one cylinder( several pulses ).
         *
         */    
        deltaTimeAccum2 += pulseDeltaTime2 ; 

        if( ( toothCount2 > 0 ) && 
            ( toothCount2 % ( tonewheelTeeth / CYLINDERS ) == 0 ) )
        {        
            deltaTimeAvg2 = deltaTimeAccum2 / ( tonewheelTeeth / CYLINDERS ) ; 
            deltaTimeAccum2 = 0 ; 
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %6ld, ", (n_long)deltaTimeAvg2 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        angle2 = ( TENTH_DEGREES * toothCount2 / tonewheelTeeth ) ; 

        /*
         * Now, output a value for the firing angle timer( a one-shot )
         * only if we're on the tooth which precedes the firing
         * angle for one of the cylinders.  We presume that there is
         * always a tooth which precedes each cylinder's firing angle.
         * The value which is output presumably goes to a "capture/compare"
         * timer which generates an interrupt used to fire that cylinder.
         * Note the special treatment for the last cylinder( #4, #6, or #8 ), 
         * we don't subtract the 'angle' because the 360th degree of rotation
         * is the same as the 0th degree of the next rotation.
         *
         */    

        /* CYLINDER 1 */    
        if( ( angle2 >= ( ( CYL1 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle2 < ( CYL1 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime2 = 
                ( ( FIRE1_ANGLE - angle2 ) * tdcTime2 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

        /* CYLINDER 2 */    
        if( ( angle2 >= ( ( CYL2 * TENTH_DEGREES / CYLINDERS ) - window ) ) &&
            ( angle2 < ( CYL2 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime2 = 
                ( ( FIRE2_ANGLE - angle2 ) * tdcTime2 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

        /* CYLINDER 3 */    
        if( ( angle2 >= ( ( CYL3 * TENTH_DEGREES / CYLINDERS ) - window ) ) &&
            ( angle2 < ( CYL3 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime2 = 
                ( ( FIRE3_ANGLE - angle2 ) * tdcTime2 / TENTH_DEGREES ) +
                angleCounter ; 
        }

        /* CYLINDER 4 */    
        if( ( angle2 >= ( ( CYL4 * TENTH_DEGREES / CYLINDERS ) - window ) ) &&
            ( angle2 < ( CYL4 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime2 = 
                ( ( FIRE4_ANGLE - angle2 ) * tdcTime2 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

#if( CYLINDERS > 4 )

        /* CYLINDER 5 */    
        if( ( angle2 >= ( ( CYL5 * TENTH_DEGREES / CYLINDERS )- window ) ) &&
            ( angle2 < ( CYL5 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime2 = 
                ( ( FIRE5_ANGLE - angle2 ) * tdcTime2 / TENTH_DEGREES ) +
                angleCounter ; 
        }

        /* CYLINDER 6 */    
        if( ( angle2 >= ( ( CYL6 * TENTH_DEGREES / CYLINDERS ) - window ) ) &&
            ( angle2 < ( CYL6 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime2 = 
                ( ( FIRE6_ANGLE - angle2 ) * tdcTime2 / TENTH_DEGREES ) +
                angleCounter ; 
        }

#if( CYLINDERS > 6 )

        /* CYLINDER 7 */    
        if( ( angle2 >= ( ( CYL7 * TENTH_DEGREES / CYLINDERS ) - window ) ) &&
            ( angle2 < ( CYL7 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime2 = 
                ( ( FIRE7_ANGLE - angle2 ) * tdcTime2 / TENTH_DEGREES ) +
                angleCounter ; 
        }

        /* CYLINDER 8 */    
        if( ( angle2 >= ( ( CYL8 * TENTH_DEGREES / CYLINDERS ) - window ) ) &&
            ( angle2 < ( CYL8 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime2 = 
                ( ( FIRE8_ANGLE - angle2 ) * tdcTime2 / TENTH_DEGREES ) +
                angleCounter ; 
        }

#endif /* 6 cylinders */
#endif /* 4 cylinders */

        if( firingTime2 > MAX_VARIABLE )
        {
            firingTime2 -= MAX_VARIABLE ; 
        }

#if BMDEBUG
        th_sprintf( szDebug, " %10ld\n", (n_long)firingTime2 ) ; 
        DebugOut( szDebug ) ; 
#else 
        WriteOut( (varsize)firingTime2 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/
        /* Gets 'angleCounter' value from test data*/
        isTableLooped += GetInputValues() ;

#if BMDEBUG
        th_sprintf( szDebug, "%8ld", (n_long)angleCounter ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*
         * Compute 'pulseDeltaTime' -- the difference in counter 
         * readings from the last pulse to the current one.  Note that
         * the realtime counter may roll over, so the elapsed time may
         * need to be corrected when this occurs.
         *
         */    

        if( angleCounterLast3 > angleCounter )
        {
            pulseDeltaTime3 = angleCounter +
                ( (varsize)MAX_VARIABLE - angleCounterLast3 + 3 ) ; 
        }
        else
        {
            pulseDeltaTime3 = angleCounter - angleCounterLast3 ; 
        }
        /* Update timer history */
        angleCounterLast3 = angleCounter ;    
        /* and elapsed time for a revolution */
        rotationTime3 += pulseDeltaTime3 ;    

#if BMDEBUG
        th_sprintf( szDebug, ", %4ld", (n_long)pulseDeltaTime3 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /*
         * Must detect TDC reference by noticing that the period between this
         * pulse and the last one is two or three times normal.  We'll set a
         * flag when TDC reference occurs, and clear it on all other pulses.
         * We also keep count of which pulse we're on relative to TDC reference.
         *
         */    

        if( pulseDeltaTime3 > ( TDC_TEETH * deltaTimeAvg3 * TDC_MARGIN ) )
        {        
            isTopDeadCenter3 = true ; 
            pulseDeltaTime3 /= TDC_TEETH ; 

            /*
             * Compute engine speed every TDC.  Engine speed will be
             * the inverse of 'tdcTime', which is the period( in CPU
             * time )between TDC's.  Engine speed is also scaled by
             * an arbitrary constant to make it useful elsewhere in
             * the engine controller.
             *
             */    

            tdcTime3 = rotationTime3 ; 
            rotationTime3 = 0 ; 
            engineSpeed3 = RPM_SCALE_FACTOR / tdcTime3 ; 
            toothCount3 = 0 ; 
        }
        else 
        {        
            toothCount3++ ; 
            isTopDeadCenter3 = false ; 
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %4d, %8ld, %5ld, %4ld", ( isTopDeadCenter3 & 1 ),
            (n_long)tdcTime3, (n_long)engineSpeed3, (n_long)toothCount3 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( (varsize)engineSpeed3 ) ; 
#endif /* BMDEBUG */

        /*
         * Compute an average delta-T for TDC pulse discrimination.
         * The delta-T will be filtered by averaging over the period
         * of one cylinder( several pulses ).
         *
         */    

        deltaTimeAccum3 += pulseDeltaTime3 ; 

        if( ( toothCount3 > 0 ) && 
            ( toothCount3 % ( tonewheelTeeth / CYLINDERS ) == 0 ) )
        {        
            deltaTimeAvg3 = deltaTimeAccum3 / ( tonewheelTeeth / CYLINDERS ) ; 
            deltaTimeAccum3 = 0 ; 
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %6ld, ", (n_long)deltaTimeAvg3 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        angle3 = ( TENTH_DEGREES * toothCount3 / tonewheelTeeth ) ; 

        /*
         * Now, output a value for the firing angle timer( a one-shot )
         * only if we're on the tooth which precedes the firing
         * angle for one of the cylinders.  We presume that there is
         * always a tooth which precedes each cylinder's firing angle.
         * The value which is output presumably goes to a "capture/compare"
         * timer which generates an interrupt used to fire that cylinder.
         * Note the special treatment for the last cylinder( #4, #6, or #8 ), 
         * we don't subtract the 'angle' because the 360th degree of rotation
         * is the same as the 0th degree of the next rotation.
         *
         */    

        /* CYLINDER 1 */    
        if( ( angle3 >= ( ( CYL1 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle3 < ( CYL1 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime3 = 
                ( ( FIRE1_ANGLE - angle3 ) * tdcTime3 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

        /* CYLINDER 2 */    
        if( ( angle3 >= ( ( CYL2 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle3 < ( CYL2 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime3 = 
                ( ( FIRE2_ANGLE - angle3 ) * tdcTime3 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

        /* CYLINDER 3 */    
        if( ( angle3 >= ( ( CYL3 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle3 <( CYL3 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime3 = 
                ( ( FIRE3_ANGLE - angle3 ) * tdcTime3 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

        /* CYLINDER 4 */    
        if( ( angle3 >= ( ( CYL4 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle3 < ( CYL4 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime3 = 
                ( ( FIRE4_ANGLE - angle3 ) * tdcTime3 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

#if( CYLINDERS > 4 )

        /* CYLINDER 5 */    
        if( ( angle3 >= ( ( CYL5 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle3 < ( CYL5 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime3 = 
                ( ( FIRE5_ANGLE - angle3 ) * tdcTime3 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

        /* CYLINDER 6 */    
        if( ( angle3 >= ( ( CYL6 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle3 < ( CYL6 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime3 = 
                ( ( FIRE6_ANGLE - angle3 ) * tdcTime3 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

#if( CYLINDERS > 6 )

        /* CYLINDER 7 */    
        if( ( angle3 >= ( ( CYL7 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle3 < ( CYL7 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime3 = 
                ( ( FIRE7_ANGLE - angle3 ) * tdcTime3 / TENTH_DEGREES ) + 
                angleCounter ; 
        }

        /* CYLINDER 8 */    
        if( ( angle3 >= ( ( CYL8 * TENTH_DEGREES / CYLINDERS ) - window ) ) && 
            ( angle3 <( CYL8 * TENTH_DEGREES / CYLINDERS ) ) )
        {        
            firingTime3 = 
                ( ( FIRE8_ANGLE - angle3 ) * tdcTime3 / TENTH_DEGREES  )+ 
                angleCounter ; 
        }

#endif /* 6 cylinders */
#endif /* 4 cylinders */

        if( firingTime3 > MAX_VARIABLE )
        {
            firingTime3 -= MAX_VARIABLE ; 
        }

#if BMDEBUG
        th_sprintf( szDebug, " %10ld", (n_long)firingTime3 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %08lX\n", ( n_ulong )loop_cnt ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( (varsize)firingTime3 ) ; 

#if DATA_SIZE == 0 /* Might break up the loop counter */
        i = (varsize)( loop_cnt &0x0000FFFF ) ; 
        WriteOut( i ) ; /*  ...in the output file */
        i = (varsize)( loop_cnt >> 16 ) ; 
        WriteOut( i ) ; 
#else
        WriteOut( (varsize)loop_cnt ) ; 
#endif
        i = (varsize)0xAAAA ; 
        WriteOut( i ) ; /* Flag the end of data-block */
#endif /* BMDEBUG */

#if BMDEBUG
        if( !th_harness_poll() )
        {
            break ; 
        }
#endif
    }

    /* Signal that we are finished */
	tcdef->duration = th_signal_finished() ;

    tcdef->iterations = loop_cnt;
    tcdef->v1 = 0 ; 
    tcdef->v2 = 0 ; 
    tcdef->v3 = 0 ; 
    tcdef->v4 = 0 ; 


#if NON_INTRUSIVE_CRC_CHECK
	tcdef->CRC=0;
/* varsize is n_short or n_long, calc crc based on e_u32 */
/* final answers are iteration dependent */
	tcdef->CRC = Calc_crc32((e_u32)*inpAngleCount,tcdef->CRC ) ; 
	tcdef->CRC = Calc_crc32((e_u32)tonewheelTeeth,tcdef->CRC ) ; 
#elif	CRC_CHECK
	tcdef->CRC=0;
#else
	tcdef->CRC=0;
#endif

	return	th_report_results(tcdef,EXPECTED_CRC);
} /* End of function 't_run_test' */


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
    xil_printf(">>     Start of angle-to-time...\n\r");
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
            th_free(inpAngleCount);
            xil_printf(">>   Test is working just fine, iteration: %8ld\n\r",benchIter++);
        }
    }
    xil_printf(">>      angle-to-time test is finished\n\r");
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

