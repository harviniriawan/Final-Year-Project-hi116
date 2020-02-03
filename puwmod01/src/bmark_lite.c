/*==============================================================================
 *$RCSfile: bmark_lite.c,v $
 *
 *   DESC : This file contains the Test Main and other TH support functions
 *
 * AUTHOR : ARM, Ltd., Rick Foos ECL, LLC
 *
 *  EEMBC : Automotive Subcommittee 
 *
 *    CVS : $Revision: 1.14 $
 *          $Date: 2002/08/07 22:21:40 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/puwmod01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.14  2002/08/07 22:21:40  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.13  2002/07/22 21:59:30  rick
 * General cleanup Beta 2b
 *
 * Revision 1.12  2002/07/19 23:10:30  rick
 * Fix iteration dependant NI CRC's
 *
 * Revision 1.11  2002/07/10 19:01:24  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.10  2002/05/29 22:25:22  rick
 * Set recommended iterations with make
 *
 * Revision 1.9  2002/05/10 23:57:46  rick
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
 * Revision 1.4  2002/03/12 17:36:37  rick
 * ITERATIONS, CRC_CHECK, NON_INTRUSIVE_CRC_CHECK
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
#define EXPECTED_CRC 0xccc1
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC 0xed1d
#else
#define EXPECTED_CRC 0x0000
#endif

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/

TCDef the_tcdef = 
{
    "AUT puwmod01   ", 
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Algorithm #3 -- Pulse-Width Modulation V1.0F0 - puwmod01", 
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

/*  DECLARATIONS */
n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree; /* NOT PART OF BENCHMARK to free RAMfile */

varsize commandPos ;        /* Commanded position from external world */
varsize *inpCmdPosition ;   /* Array of 'cmdPosition' test data values */

/*
* FUNC   : t_run_test
*
* DESC   : called to run the test
*
*          This function is called to start the test.  It does not return
*          until after the test is completed( finished ).  Note, th_finished()
*          and th_report_results()MUST be called before this function
*          returns if results are to be report.  If these are not called
*          then no results will be reported for the test.
*         
* NOTE   : after this function returns, no other functions in the test
*          will be called.  EG, returning from this function is equivelent
*          to returning from a main()or calling th_exit()
*
* RETURNS : Success if the test ran fine.  If th_finished()and
*          th_report_results()were not called, then the test finished
*          successfully, but there were no results and the host will
*          not be able to measure the test's duration.
*/
int t_run_test( struct TCDef *tcdef,int argc, const char *argv[] )
{    
    size_t		loop_cnt = tcdef->rec_iterations;

#if BMDEBUG
    n_char *szTitle = 
        "\nEEMBC Automotive/Industrial Subcommittee Benchmarks ( c )1998-1999\n"
        "Algorithm 3 :  Pulse-Width Modulation  Rev. 1.0F0 - puwmod01\n" ; 
    n_char *szHeader = 
        "\n\npwmCounter, commandPos, internalPos, upDir, incA, incB, "
        " pwmA, pwmB, dirA, enbA, dirB, enbB, counter\n" ; 
    n_char szDebug[100] ; 
#else
    varsize i ; 
#endif /* BMDEBUG */
    static n_char szDataSize[40] ; 
    int isTableLooped = false ;     /* Input test data table looped */
    static n_int upDirection1 ;       /* Initial direction is 'up' */
    static n_int upDirection2 ; 
    static n_int upDirection3 ; 
    static n_int lastDirection1 ;     /* Memory of direction to detect change */
    static n_int lastDirection2 ; 
    static n_int lastDirection3 ; 
    static n_int enbPhaseA1 ;         /* Enable for the coil A driver */
    static n_int enbPhaseA2 ;     
    static n_int enbPhaseA3 ; 
    static n_int dirPhaseA1 ;         /* Dir. control for the coil A driver */
    static n_int dirPhaseA2 ; 
    static n_int dirPhaseA3 ; 
    static n_int enbPhaseB1 ;         /* Enable for the coil B driver */
    static n_int enbPhaseB2 ; 
    static n_int enbPhaseB3 ; 
    static n_int dirPhaseB1 ;         /* Dir. control for the coil B driver */
    static n_int dirPhaseB2 ; 
    static n_int dirPhaseB3 ; 
    static n_int incrementA1 ;        /* Incr./decrement coil A's duty cycle */
    static n_int incrementA2 ; 
    static n_int incrementA3 ; 
    static n_int incrementB1 ;        /* Incr./decrement coil B's duty cycle */
    static n_int incrementB2 ; 
    static n_int incrementB3 ; 
    static varsize internalPos1 ;   /* Alg.'s internal electrical position */
    static varsize internalPos2 ; 
    static varsize internalPos3 ; 
    static n_int pwmCounter1 ;        /* The PWM phase counter */
    static n_int pwmCounter2 ; 
    static n_int pwmCounter3 ; 
    static n_int pwmPhaseA1 ;         /* The counter-comparator for coil A */
    static n_int pwmPhaseA2 ; 
    static n_int pwmPhaseA3 ; 
    static n_int pwmPhaseB1 ;         /* The counter-comparator for coil B */
    static n_int pwmPhaseB2 ; 
    static n_int pwmPhaseB3 ; 

    /* Unused */
    argc = argc ;
    argv = argv ;

    /*
     * First, initialize the data structures we need for the test
     * and allocate memory as needed.  Report an error if we can't.
     *
     */    

    /* Variable initializations at t=0 */    
    pwmCounter1 = 0 ;   /* Initialize the PWM phase counter... */
    pwmCounter2 = 0 ; 
    pwmCounter3 = 0 ; 
    dirPhaseA1 = 0 ;    /* ...and the coil driver bits */
    dirPhaseA2 = 0 ; 
    dirPhaseA3 = 0 ; 
    dirPhaseB1 = 0 ; 
    dirPhaseB2 = 0 ; 
    dirPhaseB3 = 0 ; 
    enbPhaseA1 = 1 ; 
    enbPhaseA2 = 1 ; 
    enbPhaseA3 = 1 ; 
    enbPhaseB1 = 1 ; 
    enbPhaseB2 = 1 ; 
    enbPhaseB3 = 1 ; 
    pwmPhaseA1 = MID_PHASE ;    /* ...and set 50% duty cycle */
    pwmPhaseA2 = MID_PHASE ; 
    pwmPhaseA3 = MID_PHASE ; 
    pwmPhaseB1 = MID_PHASE ; 
    pwmPhaseB2 = MID_PHASE ; 
    pwmPhaseB3 = MID_PHASE ; 
    upDirection1 = true ;       /* ...and presume we'll move 'up' */
    upDirection2 = true ; 
    upDirection3 = true ; 
    lastDirection1 = true ; 
    lastDirection2 = true ; 
    lastDirection3 = true ; 
    incrementA1 = true ;        /* ...which means we increase */
    incrementA2 = true ;        /* coil A duty cycle */
    incrementA3 = true ; 
    incrementB1 = false ;       /* ...and decrease coil B duty cycle */
    incrementB2 = false ; 
    incrementB3 = false ; 

    internalPos1 = 0 ;          /* Start out at zero position */
    internalPos2 = 0 ; 
    internalPos3 = 0 ; 
    tableCount = 0 ;    /* Start out at beginning of input test data */

    /* If debug output is desired, then must allocate some RAM... */    
    RAMfilePtr = 0 ;    /* Point to beginning of test output file */
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
    th_sprintf( szDataSize, "Data size = %s\n\n", ( DATA_SIZE ? "LONG" : "SHORT" ) ) ; 
#else
    szDataSize[0] = ( n_char )( '0' + DATA_SIZE ) ; 
    szDataSize[1] = '\0' ; 
#endif /* BMDEBUG */
 
    /* Initialize the test data -- stimuli for the algorithm. */    

    if( !GetTestData() )
    {
        /* Allocate for the test input data table */
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
            __FILE__, __LINE__ ) ; 
    }

      tcdef->CRC = 0;
    th_signal_start() ;      /* Tell the host that the test has begun */

#if BMDEBUG
    DebugOut( szTitle ) ;    /* Print the title message in RAM file */
    DebugOut( szDataSize ) ; /*  and the data size */
    DebugOut( szHeader ) ;   /* Print the test output file hdr in RAM file */
#endif /* BMDEBUG */

    /* This is the actual benchmark algorithm. */    

    /*
     * The Pulse-Width Modulation algorithm will take an input variable, 
     * presumed to be a position command for a stepper motor controlled
     * fuel-control valve, and will move the motor to that position using
     * micro-stepping control.  Micro-stepping requires that a proportional
     * current be applied in each motor coil to cause the armature to sit
     * at fractional positions within the fixed step angle of the motor.
     * This requires a PWM signal to be developed for each motor coil.
     *
     * Normally, in a two-phase stepper motor, each coil is energized in
     * either of two polarities of current flow, the four combinations of
     * which causing the stepping action when applied in the correct
     * sequence : 
     *
     * Motor coil :    A    B
     *                +    -        1    \
     *                +    +        2     |__   one cycle, 4 steps
     *                -    +        3     |
     *                -    -        4    /
     *                +    -        5
     *
     * By applying a proportional current in each phase, as well as the
     * major step polarity shown above, we can "micro-step" to positions
     * between steps 1 and 2.  This will be the purpose of the PWM
     * algorithm.  Micro-stepping not only provides more resolution for
     * motor position, but also results in smoother positioning.
     *
     * The following algorithm will presume that each pass through the loop
     * is caused by an interrupt which occurs at a fixed rate, which is
     * substantially higher than the maximum step rate applied to the
     * motor, so that a loop counter can be used to perform the timing
     * for the PWM.  If an 8-bit counter is used, then the algorithm will
     * provide up to 256 micro-step positions within each major motor step.
     *
     * For maximum torque, both motor coils will be energized at
     * 50% duty cycle in each coil applied at the major-step( motor
     * resolution )positions, while 100% duty cycle and 0% duty cycle, 
     * respectively, will be applied to the coils at the middle 
     * micro-step position between two major-step positions.  The PWM duty-
     * cycle will then increase from 0% on the de-energized coil, in the
     * reverse polarity, and decrease proportionally from 100% on the other
     * coil, until 50%/50% is reached again at the next major-step position.
     *
     *
     */    

    for( loop_cnt = 0 ; loop_cnt < tcdef->rec_iterations  ; loop_cnt++ )
    {
        /* no stopping! */

#if BMDEBUG
        if( isTableLooped )
        {
            DebugOut( "END--END--END\n" ) ;    /* Flag end-of-testfile */
        }
#endif /* BMDEBUG */

        /***********************************************************************
            First Pass                                                          
        ***********************************************************************/

        /* 
         * First, check the PWM phase, and drive the two motor coils depending
         * on their duty-cycle( DC )setpoints.
         */    

        pwmCounter1++ ;                 /* Bump the PWM phase counter */

        if( pwmCounter1 == MAX_COUNT )  /* Counter overflowed ? */
        {
            pwmCounter1 = 0 ;           /* Yes, reset the counter */
        }

#if BMDEBUG
        th_sprintf( szDebug, "%4d", pwmCounter1 ) ; 
        DebugOut( szDebug ) ; 
#endif     /* BMDEBUG */

        if( ( pwmCounter1 == pwmPhaseA1 ) &&    /* Reached end of phase A */
            ( pwmPhaseA1 < MAX_PHASE ) )        /*  and not 100% DC ? */
        {
            enbPhaseA1 = 0 ;    /* Yes, turn phase A off now */
        }
        if( ( pwmCounter1 == pwmPhaseB1 ) &&    /* Reached end of phase B */
            ( pwmPhaseB1 < MAX_PHASE ) )        /*  and not 100% DC ? */
        {
            enbPhaseB1 = 0 ;    /* Yes, turn phase B off now */
        }

        /* Now, check to see if the counter rolled over, indicating the end 
         * of a PWM cycle, and maybe time to change the motor position. We 
         * change the motor position if the commanded position is different 
         * from the current internal position. If so, we will increment the
         * internal position by one micro-step, which will require a new
         * proportional duty-cycle to be applied to each coil.
         *
         * When one phase is at 0% duty-cycle, and is incremented by one
         * micro-step, then the polarity for that phase must also be switched,
         * per the major-step table. Note that we may need to be decrementing 
         * motor position, depending on the sign of the difference between 
         * commanded and internal position.
         *
         */    

        if( pwmCounter1 == 0 ) /* Sync with pwm counter rollover */
        {
            /* Get 'commandPos' value from test data*/
            isTableLooped = GetInputValues() ;
            
            /* Are we commanded to move 'up'? */
            if( commandPos > internalPos1 ) 
            {
                /* Yes, remember that */
                upDirection1 = true ;
            }

            /* Are we commanded to move 'down' ? */
            if( commandPos < internalPos1 ) 
            {
                /* Yes, remember that */
                upDirection1 = false ;
            }

            if( upDirection1 != lastDirection1 )
            {        
                /* Change in direction ? */
                incrementA1 = !incrementA1 ;
                /* ...then, flip the duty-cycle bits */
                incrementB1 = !incrementB1 ;
            }

            /* Remember current direction for next pass */
            lastDirection1 = upDirection1 ;

            /* Are we commanded to move ? */
            if( commandPos != internalPos1 )
            {
                /* Yes, so micro-step per the duty-cycle bits */    
                if( incrementA1 )
                {
                    pwmPhaseA1++ ; 
                }
                else
                {
                    pwmPhaseA1-- ; 
                }

                if( ( pwmPhaseA1 == MAX_COUNT ) && incrementA1 )
                {        
                    pwmPhaseA1 = MAX_PHASE - 1 ; 
                    incrementA1 = false ; 
                }

                if( ( pwmPhaseA1 < 0 ) && !incrementA1 )
                {        
                    pwmPhaseA1 = 1 ; 
                    incrementA1 = true ; 
                }

                if( incrementB1 )
                {
                    pwmPhaseB1++ ; 
                }
                else
                {
                    pwmPhaseB1-- ; 
                }
                
                if( ( pwmPhaseB1 == MAX_COUNT ) && incrementB1 )
                {        
                    pwmPhaseB1 = MAX_PHASE - 1 ; 
                    incrementB1 = false ; 
                }

                if( ( pwmPhaseB1 < 0 ) && !incrementB1 )
                {        
                    pwmPhaseB1 = 1 ; 
                    incrementB1 = true ; 
                }

                /* If we reach the mid-point between major motor step 
                 * positions, then we must flip the coil driver polarity
                 *
                 */
                if( pwmPhaseA1 == 0 )
                {
                    if( dirPhaseA1 == 1 )
                    {
                        dirPhaseA1 = 0 ; 
                    }
                    else
                    {
                        dirPhaseA1 = 1 ; 
                    }
                }

                /* If we reach the mid-point between major motor step 
                 * positions, then we must flip the coil driver polarity
                 *
                 */
                if( pwmPhaseB1 == 0 )
                {
                    if( dirPhaseB1 == 1 )   
                    {
                        dirPhaseB1 = 0 ; 
                    }
                    else
                    {
                        dirPhaseB1 = 1 ; 
                    }
                }

                /* Depending on direction, bump the internal position */
                if( upDirection1 )   
                {
                    internalPos1++ ;
                }
                else
                {
                    internalPos1-- ; 
                }
            } /* End of 'commandPos != internalPos' */

            if( pwmPhaseA1 > 0 )   /* If >0% DC, turn A on again */
            {
                enbPhaseA1 = 1 ; 
            }

            if( pwmPhaseB1 > 0 )   /* And if >0% DC, turn B on again */
            {
                enbPhaseB1 = 1 ; 
            }
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %6ld, %6ld", (n_long)commandPos, 
            (n_long)internalPos1 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %2d, %2d, %2d", upDirection1, incrementA1, 
            incrementB1 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %4d, %4d", pwmPhaseA1, pwmPhaseB1 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

#if BMDEBUG
        th_sprintf( szDebug, ", %2d, %2d, %2d, %2d\n", dirPhaseA1, enbPhaseA1, 
            dirPhaseB1, enbPhaseB1 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( dirPhaseA1 ) ; 
        WriteOut( enbPhaseA1 ) ; 
        WriteOut( dirPhaseB1 ) ; 
        WriteOut( enbPhaseB1 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/

        /* 
         * First, check the PWM phase, and drive the two motor coils depending
         * on their duty-cycle( DC )setpoints.
         */    

        pwmCounter2++ ;                 /* Bump the PWM phase counter */

        if( pwmCounter2 == MAX_COUNT )  /* Counter overflowed ? */
        {
            pwmCounter2 = 0 ;           /* Yes, reset the counter */
        }

#if BMDEBUG
        th_sprintf( szDebug, "%4d", pwmCounter2 ) ; 
        DebugOut( szDebug ) ; 
#endif     /* BMDEBUG */

        if( ( pwmCounter2 == pwmPhaseA2 ) &&    /* Reached end of phase A */
            ( pwmPhaseA2 < MAX_PHASE ) )        /*  and not 100% DC ? */
        {
            enbPhaseA2 = 0 ;    /* Yes, turn phase A off now */
        }
        if( ( pwmCounter2 == pwmPhaseB2 ) &&    /* Reached end of phase B */
            ( pwmPhaseB2 < MAX_PHASE ) )        /*  and not 100% DC ? */
        {
            enbPhaseB2 = 0 ;    /* Yes, turn phase B off now */
        }

        /* Now, check to see if the counter rolled over, indicating the end 
         * of a PWM cycle, and maybe time to change the motor position. We 
         * change the motor position if the commanded position is different 
         * from the current internal position. If so, we will increment the
         * internal position by one micro-step, which will require a new
         * proportional duty-cycle to be applied to each coil.
         *
         * When one phase is at 0% duty-cycle, and is incremented by one
         * micro-step, then the polarity for that phase must also be switched,
         * per the major-step table. Note that we may need to be decrementing 
         * motor position, depending on the sign of the difference between 
         * commanded and internal position.
         *
         */    

        if( pwmCounter2 == 0 ) /* Sync with pwm counter rollover */
        {
            /* Get 'commandPos' value from test data*/
            isTableLooped = GetInputValues() ;
            
            /* Are we commanded to move 'up'? */
            if( commandPos > internalPos2 ) 
            {
                /* Yes, remember that */
                upDirection2 = true ;
            }

            /* Are we commanded to move 'down' ? */
            if( commandPos < internalPos2 ) 
            {
                /* Yes, remember that */
                upDirection2 = false ;
            }

            if( upDirection2 != lastDirection2 )
            {        
                /* Change in direction ? */
                incrementA2 = !incrementA2 ;
                /* ...then, flip the duty-cycle bits */
                incrementB2 = !incrementB2 ;
            }

            /* Remember current direction for next pass */
            lastDirection2 = upDirection2 ;

            /* Are we commanded to move ? */
            if( commandPos != internalPos2 )
            {
                /* Yes, so micro-step per the duty-cycle bits */    
                if( incrementA2 )
                {
                    pwmPhaseA2++ ; 
                }
                else
                {
                    pwmPhaseA2-- ; 
                }

                if( ( pwmPhaseA2 == MAX_COUNT ) && incrementA2 )
                {        
                    pwmPhaseA2 = MAX_PHASE - 1 ; 
                    incrementA2 = false ; 
                }

                if( ( pwmPhaseA2 < 0 ) && !incrementA2 )
                {        
                    pwmPhaseA2 = 1 ; 
                    incrementA2 = true ; 
                }

                if( incrementB2 )
                {
                    pwmPhaseB2++ ; 
                }
                else
                {
                    pwmPhaseB2-- ; 
                }
                
                if( ( pwmPhaseB2 == MAX_COUNT ) && incrementB2 )
                {        
                    pwmPhaseB2 = MAX_PHASE - 1 ; 
                    incrementB2 = false ; 
                }

                if( ( pwmPhaseB2 < 0 ) && !incrementB2 )
                {        
                    pwmPhaseB2 = 1 ; 
                    incrementB2 = true ; 
                }

                /* If we reach the mid-point between major motor step 
                 * positions, then we must flip the coil driver polarity
                 *
                 */
                if( pwmPhaseA2 == 0 )
                {
                    if( dirPhaseA2 == 1 )
                    {
                        dirPhaseA2 = 0 ; 
                    }
                    else
                    {
                        dirPhaseA2 = 1 ; 
                    }
                }

                /* If we reach the mid-point between major motor step 
                 * positions, then we must flip the coil driver polarity
                 *
                 */
                if( pwmPhaseB2 == 0 )
                {
                    if( dirPhaseB2 == 1 )   
                    {
                        dirPhaseB2 = 0 ; 
                    }
                    else
                    {
                        dirPhaseB2 = 1 ; 
                    }
                }

                /* Depending on direction, bump the internal position */
                if( upDirection2 )   
                {
                    internalPos2++ ;
                }
                else
                {
                    internalPos2-- ; 
                }
            } /* End of 'commandPos != internalPos' */

            if( pwmPhaseA2 > 0 )   /* If >0% DC, turn A on again */
            {
                enbPhaseA2 = 1 ; 
            }

            if( pwmPhaseB2 > 0 )   /* And if >0% DC, turn B on again */
            {
                enbPhaseB2 = 1 ; 
            }
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %6ld, %6ld", (n_long)commandPos, 
            (n_long)internalPos2 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %2d, %2d, %2d", upDirection2, incrementA2, 
            incrementB2 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %4d, %4d", pwmPhaseA2, pwmPhaseB2 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

#if BMDEBUG
        th_sprintf( szDebug, ", %2d, %2d, %2d, %2d\n", dirPhaseA2, enbPhaseA2, 
            dirPhaseB2, enbPhaseB2 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( dirPhaseA2 ) ; 
        WriteOut( enbPhaseA2 ) ; 
        WriteOut( dirPhaseB2 ) ; 
        WriteOut( enbPhaseB2 ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/

        /* 
         * First, check the PWM phase, and drive the two motor coils depending
         * on their duty-cycle( DC )setpoints.
         */    

        pwmCounter3++ ;                 /* Bump the PWM phase counter */

        if( pwmCounter3 == MAX_COUNT )  /* Counter overflowed ? */
        {
            pwmCounter3 = 0 ;           /* Yes, reset the counter */
        }

#if BMDEBUG
        th_sprintf( szDebug, "%4d", pwmCounter3 ) ; 
        DebugOut( szDebug ) ; 
#endif     /* BMDEBUG */

        if( ( pwmCounter3 == pwmPhaseA3 ) &&    /* Reached end of phase A */
            ( pwmPhaseA3 < MAX_PHASE ) )        /*  and not 100% DC ? */
        {
            enbPhaseA3 = 0 ;    /* Yes, turn phase A off now */
        }
        if( ( pwmCounter3 == pwmPhaseB3 ) &&    /* Reached end of phase B */
            ( pwmPhaseB3 < MAX_PHASE ) )        /*  and not 100% DC ? */
        {
            enbPhaseB3 = 0 ;    /* Yes, turn phase B off now */
        }

        /* Now, check to see if the counter rolled over, indicating the end 
         * of a PWM cycle, and maybe time to change the motor position. We 
         * change the motor position if the commanded position is different 
         * from the current internal position. If so, we will increment the
         * internal position by one micro-step, which will require a new
         * proportional duty-cycle to be applied to each coil.
         *
         * When one phase is at 0% duty-cycle, and is incremented by one
         * micro-step, then the polarity for that phase must also be switched,
         * per the major-step table. Note that we may need to be decrementing 
         * motor position, depending on the sign of the difference between 
         * commanded and internal position.
         *
         */    

        if( pwmCounter3 == 0 ) /* Sync with pwm counter rollover */
        {
            /* Get 'commandPos' value from test data*/
            isTableLooped = GetInputValues() ;
            
            /* Are we commanded to move 'up'? */
            if( commandPos > internalPos3 ) 
            {
                /* Yes, remember that */
                upDirection3 = true ;
            }

            /* Are we commanded to move 'down' ? */
            if( commandPos < internalPos3 ) 
            {
                /* Yes, remember that */
                upDirection3 = false ;
            }

            if( upDirection3 != lastDirection3 )
            {        
                /* Change in direction ? */
                incrementA3 = !incrementA3 ;
                /* ...then, flip the duty-cycle bits */
                incrementB3 = !incrementB3 ;
            }

            /* Remember current direction for next pass */
            lastDirection3 = upDirection3 ;

            /* Are we commanded to move ? */
            if( commandPos != internalPos3 )
            {
                /* Yes, so micro-step per the duty-cycle bits */    
                if( incrementA3 )
                {
                    pwmPhaseA3++ ; 
                }
                else
                {
                    pwmPhaseA3-- ; 
                }

                if( ( pwmPhaseA3 == MAX_COUNT ) && incrementA3 )
                {        
                    pwmPhaseA3 = MAX_PHASE - 1 ; 
                    incrementA3 = false ; 
                }

                if( ( pwmPhaseA3 < 0 ) && !incrementA3 )
                {        
                    pwmPhaseA3 = 1 ; 
                    incrementA3 = true ; 
                }

                if( incrementB3 )
                {
                    pwmPhaseB3++ ; 
                }
                else
                {
                    pwmPhaseB3-- ; 
                }
                
                if( ( pwmPhaseB3 == MAX_COUNT ) && incrementB3 )
                {        
                    pwmPhaseB3 = MAX_PHASE - 1 ; 
                    incrementB3 = false ; 
                }

                if( ( pwmPhaseB3 < 0 ) && !incrementB3 )
                {        
                    pwmPhaseB3 = 1 ; 
                    incrementB3 = true ; 
                }

                /* If we reach the mid-point between major motor step 
                 * positions, then we must flip the coil driver polarity
                 *
                 */
                if( pwmPhaseA3 == 0 )
                {
                    if( dirPhaseA3 == 1 )
                    {
                        dirPhaseA3 = 0 ; 
                    }
                    else
                    {
                        dirPhaseA3 = 1 ; 
                    }
                }

                /* If we reach the mid-point between major motor step 
                 * positions, then we must flip the coil driver polarity
                 *
                 */
                if( pwmPhaseB3 == 0 )
                {
                    if( dirPhaseB3 == 1 )   
                    {
                        dirPhaseB3 = 0 ; 
                    }
                    else
                    {
                        dirPhaseB3 = 1 ; 
                    }
                }

                /* Depending on direction, bump the internal position */
                if( upDirection3 )   
                {
                    internalPos3++ ;
                }
                else
                {
                    internalPos3-- ; 
                }
            } /* End of 'commandPos != internalPos' */

            if( pwmPhaseA3 > 0 )   /* If >0% DC, turn A on again */
            {
                enbPhaseA3 = 1 ; 
            }

            if( pwmPhaseB3 > 0 )   /* And if >0% DC, turn B on again */
            {
                enbPhaseB3 = 1 ; 
            }
        }

#if BMDEBUG
        th_sprintf( szDebug, ", %6ld, %6ld", (n_long)commandPos, 
            (n_long)internalPos3 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %2d, %2d, %2d", upDirection3, incrementA3, 
            incrementB3 ) ; 
        DebugOut( szDebug ) ; 
        th_sprintf( szDebug, ", %4d, %4d", pwmPhaseA3, pwmPhaseB3 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

#if BMDEBUG
        th_sprintf( szDebug, ", %2d, %2d, %2d, %2d\n", dirPhaseA3, enbPhaseA3, 
            dirPhaseB3, enbPhaseB3 ) ; 
        DebugOut( szDebug ) ; 
#else
        WriteOut( dirPhaseA3 ) ; 
        WriteOut( enbPhaseA3 ) ; 
        WriteOut( dirPhaseB3 ) ; 
        WriteOut( enbPhaseB3 ) ; 

#if DATA_SIZE == 0 /* Might break up the loop counter */
        i = (varsize )loop_cnt &0x0000FFFF ; 
        WriteOut( i ) ; /* ...in the output file */
        i = (varsize)loop_cnt >> 16 ; 
        WriteOut( i ) ; 
#else
        WriteOut( loop_cnt ) ; 
#endif
        i = 0xAAAA ; 
        WriteOut( i ) ;  /* Flag the end of data-block */
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
/* CRC_CHECK ok */
#if NON_INTRUSIVE_CRC_CHECK
/* Final results iteration dependent */
	tcdef->CRC=0;
	for (loop_cnt=0;loop_cnt<NUM_TESTS;loop_cnt++) 
		tcdef->CRC = Calc_crc32((e_u32)inpCmdPosition[loop_cnt],tcdef->CRC);
#elif	CRC_CHECK
/* Iteration dependant results check */
	tcdef->CRC=0;
	tcdef->CRC = Calc_crc32((e_u32) dirPhaseA1,tcdef->CRC) ; 
	tcdef->CRC = Calc_crc32((e_u32) enbPhaseA1,tcdef->CRC) ; 
	tcdef->CRC = Calc_crc32((e_u32) dirPhaseB1,tcdef->CRC) ; 
	tcdef->CRC = Calc_crc32((e_u32) enbPhaseB1,tcdef->CRC) ; 
	tcdef->CRC = Calc_crc32((e_u32) dirPhaseA2,tcdef->CRC) ; 
	tcdef->CRC = Calc_crc32((e_u32) enbPhaseA2,tcdef->CRC) ; 
	tcdef->CRC = Calc_crc32((e_u32) dirPhaseB2,tcdef->CRC) ; 
	tcdef->CRC = Calc_crc32((e_u32) enbPhaseB2,tcdef->CRC) ; 
	tcdef->CRC = Calc_crc32((e_u32) dirPhaseA3,tcdef->CRC) ; 
	tcdef->CRC = Calc_crc32((e_u32) enbPhaseA3,tcdef->CRC) ; 
	tcdef->CRC = Calc_crc32((e_u32) dirPhaseB3,tcdef->CRC) ; 
	tcdef->CRC = Calc_crc32((e_u32) enbPhaseB3,tcdef->CRC) ; 
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
    xil_printf(">>     Start of puwmod...\n\r");
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
            th_free(inpCmdPosition);

            xil_printf(">>   Test is working just fine, iteration: %8ld\n\r",benchIter++);
        }
    }
    xil_printf(">>      puwmod test is finished\n\r");
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

