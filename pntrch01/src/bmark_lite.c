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
 *          $Date: 2002/08/07 22:21:37 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/pntrch01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.14  2002/08/07 22:21:37  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.13  2002/07/22 21:59:29  rick
 * General cleanup Beta 2b
 *
 * Revision 1.12  2002/07/19 23:10:28  rick
 * Fix iteration dependant NI CRC's
 *
 * Revision 1.11  2002/07/11 22:13:31  rick
 * Initialize tcdef results
 *
 * Revision 1.10  2002/07/10 19:01:23  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.9  2002/05/29 22:25:20  rick
 * Set recommended iterations with make
 *
 * Revision 1.8  2002/05/10 23:57:46  rick
 * Fix missing shifts in 16, and 32 bit CRC calculations
 *
 * Revision 1.7  2002/05/10 17:20:37  rick
 * Add al_main to API
 *
 * Revision 1.6  2002/04/25 20:10:45  rick
 * sprintf to th_sprintf
 *
 * Revision 1.5  2002/03/14 21:47:15  rick
 * Remove whitespace preceeding ifdef's, compiler problem resolution.
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
#define ITERATIONS 2500	/* required iterations for crc */
#else
#define ITERATIONS 2500	/* recommended iterations for benchmark */
#endif
#endif

/* Pre-caclculated CRC values */
#if	CRC_CHECK
#define EXPECTED_CRC 0xd739
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC 0x04a6
#else
#define EXPECTED_CRC 0x0000
#endif

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/


TCDef the_tcdef = 
{
	"AUT pntrch01   ",
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
   "Algorithm #11 --  Pointer Chasing V1.0E0 ",
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

/*  DECLARATIONS */    
/* ------------------------------------------------------------------------------
 * Local Data
 *
 */


n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree;       /* NOT PART OF BENCHMARK, free ramfile after every run */

varsize *inputToken;			/* Pointer to table of input tokens to look for */
varsize token;					/* The input token to match */
varsize *listData;				/* The doubly-linked list data table */
varsize *leftPtr;				/* The doubly-linked list 'left' pointer table */
varsize *rightPtr;				/* The doubly-linked list 'right' pointer table */

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
	char *szTitle = "\nEEMBC Automotive/Industrial Subcommittee Benchmarks  (c)1998-1999\n"
					"Algorithm 11:  Pointer Chasing Rev. 1.0A0\n";
	char *szHeader = "\n\n[token], listData, direction, leftPtr, rightPtr"
					 "\ncount, lastData { loop counter }\n";
 	char szDebug[100];
#else
	int i;
#endif /* BMDEBUG */
 	char szDataSize[40];
	int isTableLooped = false;								/* Input test data table looped */
	static varsize positionNow_1;							/* Current position in the list */
	static varsize positionNow_2;
	static varsize positionNow_3;
	static varsize positionNow_4;
	static varsize positionNow_5;
	static varsize startPosition_1;							/* Beginning position of the current search */
	static varsize startPosition_2;
	static varsize startPosition_3;
	static varsize startPosition_4;
	static varsize startPosition_5;
	static varsize searchCount_1;							/* How many items in the list were searched */
	static varsize searchCount_2;
	static varsize searchCount_3;
	static varsize searchCount_4;
	static varsize searchCount_5;
	static n_int direction_1;									/* Current direction of the search */
	static n_int direction_2;
	static n_int direction_3;
	static n_int direction_4;
	static n_int direction_5;
	static n_int oldDirection_1;								/* Beginning direction of the current search */
	static n_int oldDirection_2;
	static n_int oldDirection_3;
	static n_int oldDirection_4;
	static n_int oldDirection_5;


   /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    * First, initialize the data structures we need for the test
    * and allocate memory as needed.  Report an error if we can't.
	*
	*/

	/* Variable initializations at t=0 */

	tableCount = 0;								/* Start out at beginning of input test data */
	positionNow_1 = 0;							/* At the base of the table */
	positionNow_2 = 0;
	positionNow_3 = 0;
	positionNow_4 = 0;
	positionNow_5 = 0;
	direction_1 = RIGHT;						/* Start out searching to the 'right' */
	direction_2 = RIGHT;
	direction_3 = RIGHT;
	direction_4 = RIGHT;
	direction_5 = RIGHT;

	/* Set size of output file (1K) */    
    RAMfileSize = MAX_FILESIZE ;

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
	th_sprintf( szDataSize, "Data size = %s\n\n", ( DATA_SIZE ? "LONG" : "SHORT" ) );
#else
	szDataSize[0] = (char) ( DATA_SIZE ? 'L' : 'S' );
	szDataSize[1] = '\0';
#endif /* BMDEBUG */


   /* Initialize the test data -- stimuli for the algorithm.
    *   
	*/

	if ( !GetTestData() )					/* Allocate for the test input data table */
		th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s:%d", __FILE__, __LINE__ );

      tcdef->CRC = 0;
	th_signal_start();						 /* Tell the host that the test has begun */

#if BMDEBUG
	DebugOut( szTitle );					/* Print the title message in RAM file */
	DebugOut( szHeader );					/* Print the test output file header in RAM file */
	DebugOut( szDataSize );					/*  and the data size */
#endif /* BMDEBUG */


   /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    * This is the actual benchmark algorithm.
	*
	*/

	/*
	 * Do a search for a token in a doubly-linked list.  The list is comprised
	 * of the data list, 'left' pointer list, and 'right' pointer list.  This is
	 * a simple linear list structure.  Each data item is associated with a pointer
	 * to the next item to the left in the data sequence, and a pointer to the next
	 * item to the right in the data sequence.  This is not a circular list, its
	 * ends are bounded (with -1's).  Therefore, when the search hits one end of
	 * the list, the search direction is reversed to continue searching in the other
	 * direction.  When the search direction changes, the search commences from the
	 * same starting point as the initial search, so that the already searched items
	 * in the list are skipped over.  Therefore, the search will always complete in
	 * LIST_SIZE steps, where LIST_SIZE is the size of the list.
	 *
	 * When the search produces a match between 'inputToken' and 'listData', the
	 * search is over, and we break out of the search loop.  We report the matched
	 * data, and the count of search steps required to find the data.  Note that the
	 * search always starts out wherever the last search left off.
	 *
	 * In case no match is found, then 'searchCount' (count of steps) will equal
	 * the LIST_SIZE+1 (one step required to hit list boundary).  This is the only
	 * indication of no match.
	 *
	 */

   for ( loop_cnt = 0; loop_cnt < tcdef->rec_iterations; loop_cnt++ )  /* no stopping! */
      {
#if BMDEBUG
		if ( isTableLooped ) 							/* Might reset pointer to beginning of test output text file */
			DebugOut( "END--END--END\n" );				/* Flag end-of-testfile */
#endif /* BMDEBUG */

		/* DO IT ALL ONCE */

		isTableLooped = GetInputValues();				/* Gets 'inputToken' from test data */

#if BMDEBUG										/* Output some debug info, if needed */
		th_sprintf( szDebug, "[%6ld]\n", token );
		DebugOut( szDebug );
#endif /* BMDEBUG */

		startPosition_1 = positionNow_1;				/* Remember where we started the search */
		oldDirection_1 = direction_1;						/*  and in which direction the search began */

		for ( searchCount_1 = 0; searchCount_1 <= LIST_SIZE; searchCount_1++ )
		{
#if BMDEBUG										/* Output some debug info, if needed */
			th_sprintf( szDebug, "%6ld, %2d, %6ld, %6ld\n",
				(long) listData[positionNow_1], direction_1,
				(long) leftPtr[positionNow_1], (long) rightPtr[positionNow_1] );
			DebugOut( szDebug );
#endif /* BMDEBUG */

			if ( token == listData[positionNow_1] )		/* See if we've found the desired token */
				break;									/* Match!  We're outta here... */

			if ( direction_1 == LEFT )					/* Depending on direction of search, */
				positionNow_1 = leftPtr[positionNow_1];		/*  get the pointer to next data to left */
			else
				positionNow_1 = rightPtr[positionNow_1];	/*  or next data to the right */

			if ( positionNow_1 == END_LIST )				/* Else, did we hit the end of the list ? */
			{
				if ( oldDirection_1 != direction_1 )		/* Yes, did we already do this ? */
				{
					searchCount_1 = LIST_SIZE+1;				/* Yes, sorry, no match anywhere... */
					break;
				}

				positionNow_1 = startPosition_1;			/* No, so start over where this search */
				direction_1 = !direction_1;					/*  started, and go the other direction */
			}

		} /* End of search loop */

#if BMDEBUG										/* Output some debug info, if needed */
		th_sprintf( szDebug, "\n%6ld, %6ld", (long) searchCount_1,
			(long) listData[positionNow_1] );
		DebugOut( szDebug );
#else
		WriteOut( searchCount_1 );
		WriteOut( listData[positionNow_1] );
#endif /* BMDEBUG */


		/* DO IT ALL TWICE */

		isTableLooped += GetInputValues();				/* Gets 'inputToken' from test data */

#if BMDEBUG										/* Output some debug info, if needed */
		th_sprintf( szDebug, "[%6ld]\n", token );
		DebugOut( szDebug );
#endif /* BMDEBUG */

		startPosition_2 = positionNow_2;				/* Remember where we started the search */
		oldDirection_2 = direction_2;						/*  and in which direction the search began */

		for ( searchCount_2 = 0; searchCount_2 <= LIST_SIZE; searchCount_2++ )
		{
#if BMDEBUG										/* Output some debug info, if needed */
			th_sprintf( szDebug, "%6ld, %2d, %6ld, %6ld\n",
				(long) listData[positionNow_2], direction_2,
				(long) leftPtr[positionNow_2], (long) rightPtr[positionNow_2] );
			DebugOut( szDebug );
#endif /* BMDEBUG */

			if ( token == listData[positionNow_2] )		/* See if we've found the desired token */
				break;									/* Match!  We're outta here... */

			if ( direction_2 == LEFT )					/* Depending on direction of search, */
				positionNow_2 = leftPtr[positionNow_2];		/*  get the pointer to next data to left */
			else
				positionNow_2 = rightPtr[positionNow_2];	/*  or next data to the right */

			if ( positionNow_2 == END_LIST )				/* Else, did we hit the end of the list ? */
			{
				if ( oldDirection_2 != direction_2 )		/* Yes, did we already do this ? */
				{
					searchCount_2 = LIST_SIZE+1;				/* Yes, sorry, no match anywhere... */
					break;
				}

				positionNow_2 = startPosition_2;			/* No, so start over where this search */
				direction_2 = !direction_2;					/*  started, and go the other direction */
			}

		} /* End of search loop */

#if BMDEBUG										/* Output some debug info, if needed */
		th_sprintf( szDebug, "\n%6ld, %6ld", (long) searchCount_2,
			(long) listData[positionNow_2] );
		DebugOut( szDebug );
#else
		WriteOut( searchCount_2 );
		WriteOut( listData[positionNow_2] );
#endif /* BMDEBUG */


		/* DO IT ALL A THIRD TIME */

		isTableLooped += GetInputValues();				/* Gets 'inputToken' from test data */

#if BMDEBUG										/* Output some debug info, if needed */
		th_sprintf( szDebug, "[%6ld]\n", token );
		DebugOut( szDebug );
#endif /* BMDEBUG */

		startPosition_3 = positionNow_3;				/* Remember where we started the search */
		oldDirection_3 = direction_3;						/*  and in which direction the search began */

		for ( searchCount_3 = 0; searchCount_3 <= LIST_SIZE; searchCount_3++ )
		{
#if BMDEBUG										/* Output some debug info, if needed */
			th_sprintf( szDebug, "%6ld, %2d, %6ld, %6ld\n",
				(long) listData[positionNow_3], direction_3,
				(long) leftPtr[positionNow_3], (long) rightPtr[positionNow_3] );
			DebugOut( szDebug );
#endif /* BMDEBUG */

			if ( token == listData[positionNow_3] )		/* See if we've found the desired token */
				break;									/* Match!  We're outta here... */

			if ( direction_3 == LEFT )					/* Depending on direction of search, */
				positionNow_3 = leftPtr[positionNow_3];		/*  get the pointer to next data to left */
			else
				positionNow_3 = rightPtr[positionNow_3];	/*  or next data to the right */

			if ( positionNow_3 == END_LIST )				/* Else, did we hit the end of the list ? */
			{
				if ( oldDirection_3 != direction_3 )		/* Yes, did we already do this ? */
				{
					searchCount_3 = LIST_SIZE+1;				/* Yes, sorry, no match anywhere... */
					break;
				}

				positionNow_3 = startPosition_3;			/* No, so start over where this search */
				direction_3 = !direction_3;					/*  started, and go the other direction */
			}

		} /* End of search loop */

#if BMDEBUG										/* Output some debug info, if needed */
		th_sprintf( szDebug, "\n%6ld, %6ld", (long) searchCount_3,
			(long) listData[positionNow_3] );
		DebugOut( szDebug );
#else
		WriteOut( searchCount_3 );
		WriteOut( listData[positionNow_3] );
#endif /* BMDEBUG */

		/* DO IT ALL A FOURTH TIME */

		isTableLooped += GetInputValues();				/* Gets 'inputToken' from test data */

#if BMDEBUG										/* Output some debug info, if needed */
		th_sprintf( szDebug, "[%6ld]\n", token );
		DebugOut( szDebug );
#endif /* BMDEBUG */

		startPosition_4 = positionNow_4;				/* Remember where we started the search */
		oldDirection_4 = direction_4;						/*  and in which direction the search began */

		for ( searchCount_4 = 0; searchCount_4 <= LIST_SIZE; searchCount_4++ )
		{
#if BMDEBUG										/* Output some debug info, if needed */
			th_sprintf( szDebug, "%6ld, %2d, %6ld, %6ld\n",
				(long) listData[positionNow_4], direction_4,
				(long) leftPtr[positionNow_4], (long) rightPtr[positionNow_4] );
			DebugOut( szDebug );
#endif /* BMDEBUG */

			if ( token == listData[positionNow_4] )		/* See if we've found the desired token */
				break;									/* Match!  We're outta here... */

			if ( direction_4 == LEFT )					/* Depending on direction of search, */
				positionNow_4 = leftPtr[positionNow_4];		/*  get the pointer to next data to left */
			else
				positionNow_4 = rightPtr[positionNow_4];	/*  or next data to the right */

			if ( positionNow_4 == END_LIST )				/* Else, did we hit the end of the list ? */
			{
				if ( oldDirection_4 != direction_4 )		/* Yes, did we already do this ? */
				{
					searchCount_4 = LIST_SIZE+1;				/* Yes, sorry, no match anywhere... */
					break;
				}

				positionNow_4 = startPosition_4;			/* No, so start over where this search */
				direction_4 = !direction_4;					/*  started, and go the other direction */
			}

		} /* End of search loop */

#if BMDEBUG										/* Output some debug info, if needed */
		th_sprintf( szDebug, "\n%6ld, %6ld", (long) searchCount_4,
			(long) listData[positionNow_4] );
		DebugOut( szDebug );
#else
		WriteOut( searchCount_4 );
		WriteOut( listData[positionNow_4] );
#endif /* BMDEBUG */

		/* DO IT ALL A FIFTH TIME */

		isTableLooped += GetInputValues();				/* Gets 'inputToken' from test data */

#if BMDEBUG										/* Output some debug info, if needed */
		th_sprintf( szDebug, "[%6ld]\n", token );
		DebugOut( szDebug );
#endif /* BMDEBUG */

		startPosition_5 = positionNow_5;				/* Remember where we started the search */
		oldDirection_5 = direction_5;						/*  and in which direction the search began */

		for ( searchCount_5 = 0; searchCount_5 <= LIST_SIZE; searchCount_5++ )
		{
#if BMDEBUG										/* Output some debug info, if needed */
			th_sprintf( szDebug, "%6ld, %2d, %6ld, %6ld\n",
				(long) listData[positionNow_5], direction_5,
				(long) leftPtr[positionNow_5], (long) rightPtr[positionNow_5] );
			DebugOut( szDebug );
#endif /* BMDEBUG */

			if ( token == listData[positionNow_5] )		/* See if we've found the desired token */
				break;									/* Match!  We're outta here... */

			if ( direction_5 == LEFT )					/* Depending on direction of search, */
				positionNow_5 = leftPtr[positionNow_5];		/*  get the pointer to next data to left */
			else
				positionNow_5 = rightPtr[positionNow_5];	/*  or next data to the right */

			if ( positionNow_5 == END_LIST )				/* Else, did we hit the end of the list ? */
			{
				if ( oldDirection_5 != direction_5 )		/* Yes, did we already do this ? */
				{
					searchCount_5 = LIST_SIZE+1;				/* Yes, sorry, no match anywhere... */
					break;
				}

				positionNow_5 = startPosition_5;			/* No, so start over where this search */
				direction_5 = !direction_5;					/*  started, and go the other direction */
			}

		} /* End of search loop */

#if BMDEBUG										/* Output some debug info, if needed */
		th_sprintf( szDebug, "\n%6ld, %6ld", (long) searchCount_5,
			(long) listData[positionNow_5] );
		DebugOut( szDebug );
#else
		WriteOut( searchCount_5 );
		WriteOut( listData[positionNow_5] );
#endif /* BMDEBUG */

#if BMDEBUG										/* Output some debug info, if needed */
		th_sprintf( szDebug, " { %08lX }\n", (unsigned long) loop_cnt );
		DebugOut( szDebug );
#else											/* Or write value directly to RAM file */
#if DATA_SIZE == 0								/* Might break up the loop counter */
		i = (varsize) ( loop_cnt & 0x0000FFFF );
		WriteOut( i );								/*  in the output file */
		i = (varsize) ( loop_cnt >> 16 );
		WriteOut( i );
#else
		WriteOut(loop_cnt );
#endif
		i = 0xAAAA;
		WriteOut( i );				/* Flag the end of data-block */
#endif /* BMDEBUG */

#if BMDEBUG
      if ( !th_harness_poll() )
         break;
#endif
      }

	tcdef->duration = th_signal_finished() ;
    tcdef->iterations = loop_cnt ; 
	tcdef->v1         = 0;
	tcdef->v2         = 0;
	tcdef->v3         = 0;
	tcdef->v4         = 0;

#if		NON_INTRUSIVE_CRC_CHECK
/* Final results iteration dependent */
	tcdef->CRC=0;
	for (loop_cnt=0;loop_cnt<NUM_TESTS;loop_cnt++) {
		tcdef->CRC = Calc_crc32((e_u32)inputToken[loop_cnt],tcdef->CRC);
	}
#elif	CRC_CHECK
/* Iteration dependant results check */
	tcdef->CRC = 0;
	tcdef->CRC = Calc_crc32((e_u32)searchCount_1,tcdef->CRC);
	tcdef->CRC = Calc_crc32((e_u32)searchCount_2,tcdef->CRC);
	tcdef->CRC = Calc_crc32((e_u32)searchCount_3,tcdef->CRC);
	tcdef->CRC = Calc_crc32((e_u32)searchCount_4,tcdef->CRC);
	tcdef->CRC = Calc_crc32((e_u32)searchCount_5,tcdef->CRC);
#else
	tcdef->CRC=0;
#endif

	return	th_report_results(tcdef,EXPECTED_CRC);
} 


/***************************************************************************/
n_int benchIter;
n_int failTest;
int main(int argc, const char* argv[] )
{
    /* initialise variable to 0 */
    failTest = 0;
    benchIter = 0;
	init_platform();
	/* target specific inititialization */
	al_main(argc, argv);
    xil_printf(">>     Start of Pointer Chasing...\n\r");
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
        	/* Free allocated memory to avoid runtime malloc error, NOT PART OF BENCHMARK */
            th_free(RAMfileFree);
            th_free(inputToken);
            th_free(listData);
            th_free(leftPtr);
            th_free(rightPtr);
            xil_printf(">>   Test is working just fine, iteration: %8ld\n\r",benchIter++);
        }
    }
    xil_printf(">>     Pointer Chasing test is finished\n\r");
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

