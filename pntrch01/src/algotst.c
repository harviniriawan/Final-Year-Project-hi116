/*==============================================================================
 *$RCSfile: algotst.c,v $
 *
 *   DESC : Pointer Chasing
 *
 * AUTHOR : dt
 *
 *  EEMBC : Automotive/Industrial Subcommittee 
 *
 *    CVS : $Revision: 1.3 $
 *          $Date: 2002/04/23 18:38:40 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/pntrch01/algotst.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: algotst.c,v $
 * Revision 1.3  2002/04/23 18:38:40  rick
 * Add anytoi to th reg project files
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

/* INCLUDE FILES */

#include "algo.h"


/*  DECLARATIONS  */

/*
 * The following test data contains indicies to functions and their corresponding arguments.
 *
 */

const varsize inpTokenROM[] = {

	10, 20, 100, 200, 300, 399, 1, 398, 2, 397, 3, 396, 4, 395,
	5, 394, 6, 393, 7, 392, 8, 391, 9, 390, 10, 389, 11, 388,
	12, 387, 13, 386, 14, 385, 15, 384, 16, 383, 17, 382, 18, 381,
	19, 380, 20, 379, 21, 378, 22, 377, 23, 376, 24, 375, 25, 374,
	26, 373, 27, 372, 28, 371, 29, 370, 30, 369, 31, 368, 32, 367,
	33, 365, 34, 364, 35, 363, 36, 362, 37, 361, 38, 360, 39, 359,
	40, 358, 41, 357, 42, 356, 43, 355, 44, 354, 45, 353, 46, 352,
	12, 287, 13, 386, 114, 385, 15, 184, 16, 383, 17, 382, 218, 381,
	19, 280, 20, 379, 121, 378, 22, 177, 23, 376, 24, 375, 225, 374,
	26, 273, 27, 372, 128, 371, 29, 170, 30, 369, 31, 368, 232, 367,
	33, 265, 34, 364, 135, 363, 36, 162, 37, 361, 38, 360, 239, 359,
	40, 258, 41, 357, 142, 356, 43, 155, 44, 354, 45, 353, 246, 352,
	
};  /* End of test values:  inpTokenROM[] */



/*
 * The following table contains the data portion of a doubly-linked list.
 * The data is completely arbitrary, all that matters is that all tokens
 * are different, no repeats.  There is not an implied sequence of these
 * values.  This data will be searched via the 'left' and 'right' pointers
 * (really just indicies into this table) to match the input token.
 *
 */

const varsize dataTableROM[] = {

	1,		3,		5,		7,		9,		11,		13,		15,		17,		19,
	2,		4,		6,		8,		10,		12,		14,		16,		18,		20,
	21,		23,		25,		27,		29,		31,		33,		35,		37,		39,
	22,		24,		26,		28,		30,		32,		34,		36,		38,		40,
	41,		43,		45,		47,		49,		51,		53,		55,		57,		59,
	42,		44,		46,		48,		50,		52,		54,		56,		58,		50,
	61,		63,		65,		67,		69,		71,		73,		75,		77,		79,
	62,		64,		66,		68,		70,		72,		74,		76,		78,		70,
	81,		83,		85,		87,		89,		91,		93,		95,		97,		99,
	82,		84,		86,		88,		90,		92,		94,		96,		98,		100,

	101,	103,	105,	107,	109,	111,	113,	115,	117,	119,
	102,	104,	106,	108,	110,	112,	114,	116,	118,	120,
	121,	123,	125,	127,	129,	131,	133,	135,	137,	139,
	122,	124,	126,	128,	130,	132,	134,	136,	138,	140,
	141,	143,	145,	147,	149,	151,	153,	155,	157,	159,
	142,	144,	146,	148,	150,	152,	154,	156,	158,	160,
	161,	163,	165,	167,	169,	171,	173,	175,	177,	179,
	162,	164,	166,	168,	170,	172,	174,	176,	178,	180,
	181,	183,	185,	187,	189,	191,	193,	195,	197,	199,
	182,	184,	186,	188,	190,	192,	194,	196,	198,	200,

	201,	203,	205,	207,	209,	211,	213,	215,	217,	219,
	202,	204,	206,	208,	210,	212,	214,	216,	218,	220,
	221,	223,	225,	227,	229,	231,	233,	235,	237,	239,
	222,	224,	226,	228,	230,	232,	234,	236,	238,	240,
	241,	243,	245,	247,	249,	251,	253,	255,	257,	259,
	242,	244,	246,	248,	250,	252,	254,	256,	258,	260,
	261,	263,	265,	267,	269,	271,	273,	275,	277,	279,
	262,	264,	266,	268,	270,	272,	274,	276,	278,	280,
	281,	283,	285,	287,	289,	291,	293,	295,	297,	299,
	282,	284,	286,	288,	290,	292,	294,	296,	298,	300,

	301,	303,	305,	307,	309,	311,	313,	315,	317,	319,
	302,	304,	306,	308,	310,	312,	314,	316,	318,	320,
	321,	323,	325,	327,	329,	331,	333,	335,	337,	339,
	322,	324,	326,	328,	330,	332,	334,	336,	338,	340,
	341,	343,	345,	347,	349,	351,	353,	355,	357,	359,
	342,	344,	346,	348,	350,	352,	354,	356,	358,	360,
	361,	363,	365,	367,	369,	371,	373,	375,	377,	379,
	362,	364,	366,	368,	370,	372,	374,	376,	378,	380,
	381,	383,	385,	387,	389,	391,	393,	395,	397,	399,
	382,	384,	386,	388,	390,	392,	394,	396,	398,	400

}; /* End of table of data 'dataTableROM' */


/*
 * The following table contains the 'left' pointers of a doubly-linked list.
 * These pointers are the indicies to the 'dataTable' above, indicating the 
 * index value of the token which is to the left of the current token in a
 * linear list.  The 'END_LIST' value indicates the left end of the list
 * (i.e., this is not a circular list ).
 *
 */

const varsize leftPtrTableROM[] = {
	END_LIST,	0,	1,	2,	3,	4,	5,	6,	7,	8,
	9,	10,	11,	12,	13,	14,	15,	16,	17,	18,
	19,	30,	31,	32,	33,	34,	35,	36,	37,	38,
	39,	20,	21,	22,	23,	24,	25,	26,	27,	28,
	29,	50,	51,	52,	53,	54,	55,	56,	57,	58,
	59,	40,	41,	42,	43,	44,	45,	46,	47,	48,
	49,	60,	61,	62,	63,	64,	65,	66,	67,	68,
	69,	70,	71,	72,	73,	74,	75,	76,	77,	78,
	79,	90,	91,	92,	93,	94,	95,	96,	97,	98,
	99,	80,	81,	82,	83,	84,	85,	86,	87,	88,

	89,	100,	101,	102,	103,	104,	105,	106,	107,	108,
	109,	110,	111,	112,	113,	114,	115,	116,	117,	118,
	119,	130,	131,	132,	133,	134,	135,	136,	137,	138,
	139,	120,	121,	122,	123,	124,	125,	126,	127,	128,
	129,	150,	151,	152,	153,	154,	155,	156,	157,	158,
	159,	140,	141,	142,	143,	144,	145,	146,	147,	148,
	149,	160,	161,	162,	163,	164,	165,	166,	167,	168,
	169,	170,	171,	172,	173,	174,	175,	176,	177,	178,
	179,	190,	191,	192,	193,	194,	195,	196,	197,	198,
	199,	180,	181,	182,	183,	184,	185,	186,	187,	188,

	189,	200,	201,	202,	203,	204,	205,	206,	207,	208,
	209,	210,	211,	212,	213,	214,	215,	216,	217,	218,
	219,	230,	231,	232,	233,	234,	235,	236,	237,	238,
	239,	220,	221,	222,	223,	224,	225,	226,	227,	228,
	229,	250,	251,	252,	253,	254,	255,	256,	257,	258,
	259,	240,	241,	242,	243,	244,	245,	246,	247,	248,
	249,	260,	261,	262,	263,	264,	265,	266,	267,	268,
	269,	270,	271,	272,	273,	274,	275,	276,	277,	278,
	279,	290,	291,	292,	293,	294,	295,	296,	297,	298,
	299,	280,	281,	282,	283,	284,	285,	286,	287,	288,

	289,	300,	301,	302,	303,	304,	305,	306,	307,	308,
	309,	310,	311,	312,	313,	314,	315,	316,	317,	318,
	319,	330,	331,	332,	333,	334,	335,	336,	337,	338,
	339,	320,	321,	322,	323,	324,	325,	326,	327,	328,
	329,	350,	351,	352,	353,	354,	355,	356,	357,	358,
	359,	340,	341,	342,	343,	344,	345,	346,	347,	348,
	349,	360,	361,	362,	363,	364,	365,	366,	367,	368,
	369,	370,	371,	372,	373,	374,	375,	376,	377,	378,
	379,	390,	391,	392,	393,	394,	395,	396,	397,	398,
	399,	380,	381,	382,	383,	384,	385,	386,	387,	388

}; /* End of table of data pointers 'leftPtrTableROM' */


/*
 * The following table contains the 'right' pointers of a doubly-linked list.
 * These pointers are the indicies to the 'dataTable' above, indicating the 
 * index value of the token which is to the right of the current token.  The
 * 'END_LIST' indicates the right end of the list (i.e., this is not a circular list ).
 *
 */

const varsize rightPtrTableROM[] = {
	1,		2,		3,		4,		5,		6,		7,		8,		9,		10,
	11,		12,		13,		14,		15,		16,		17,		18,		19,		20,
	31,		32,		33,		34,		35,		36,		37,		38,		39,		40,
	21,		22,		23,		24,		25,		26,		27,		28,		29,		30,
	51,		52,		53,		54,		55,		56,		57,		58,		59,		60,
	41,		42,		43,		44,		45,		46,		47,		48,		49,		50,
	61,		62,		63,		64,		65,		66,		67,		68,		69,		70,
	71,		72,		73,		74,		75,		76,		77,		78,		79,		80,
	91,		92,		93,		94,		95,		96,		97,		98,		99,		100,
	81,		82,		83,		84,		85,		86,		87,		88,		89,		90,

	101,	102,	103,	104,	105,	106,	107,	108,	109,	110,
	111,	112,	113,	114,	115,	116,	117,	118,	119,	120,
	131,	132,	133,	134,	135,	136,	137,	138,	139,	140,
	121,	122,	123,	124,	125,	126,	127,	128,	129,	130,
	151,	152,	153,	154,	155,	156,	157,	158,	159,	160,
	141,	142,	143,	144,	145,	146,	147,	148,	149,	150,
	161,	162,	163,	164,	165,	166,	167,	168,	169,	170,
	171,	172,	173,	174,	175,	176,	177,	178,	179,	180,
	191,	192,	193,	194,	195,	196,	197,	198,	199,	200,
	181,	182,	183,	184,	185,	186,	187,	188,	189,	190,

	201,	202,	203,	204,	205,	206,	207,	208,	209,	210,
	211,	212,	213,	214,	215,	216,	217,	218,	219,	220,
	231,	232,	233,	234,	235,	236,	237,	238,	239,	240,
	221,	222,	223,	224,	225,	226,	227,	228,	229,	230,
	251,	252,	253,	254,	255,	256,	257,	258,	259,	260,
	241,	242,	243,	244,	245,	246,	247,	248,	249,	250,
	261,	262,	263,	264,	265,	266,	267,	268,	269,	270,
	271,	272,	273,	274,	275,	276,	277,	278,	279,	280,
	291,	292,	293,	294,	295,	296,	297,	298,	299,	300,
	281,	282,	283,	284,	285,	286,	287,	288,	289,	290,

	301,	302,	303,	304,	305,	306,	307,	308,	309,	310,
	311,	312,	313,	314,	315,	316,	317,	318,	319,	320,
	331,	332,	333,	334,	335,	336,	337,	338,	339,	340,
	321,	322,	323,	324,	325,	326,	327,	328,	329,	330,
	351,	352,	353,	354,	355,	356,	357,	358,	359,	360,
	341,	342,	343,	344,	345,	346,	347,	348,	349,	350,
	361,	362,	363,	364,	365,	366,	367,	368,	369,	370,
	371,	372,	373,	374,	375,	376,	377,	378,	379,	380,
	391,	392,	393,	394,	395,	396,	397,	398,	399,	END_LIST,
	381,	382,	383,	384,	385,	386,	387,	388,	389,	390

}; /* End of table of data pointers 'rightPtrTableROM' */


/*	FUNCTIONS  */


/*
 *  Function:  GetTestData
 *
 * Builds the tables of input data (which represents the input token to
 * be matched), the doubly-linked data list, and left and right token pointers.
 * This is the "real world" data stream which drives the algorithm.
 *
 */


n_int GetTestData( n_void )
{
	n_int i;

	inputToken = (varsize *) th_malloc( (NUM_TESTS+1) * sizeof(varsize) );

	if ( inputToken == NULL )
		return( false );

	/*  Copy the test values from ROM to RAM  */

	for ( i = 0; i < NUM_TESTS; i++ )
		inputToken[i] = inpTokenROM[i];

	listData = (varsize *) th_malloc( (LIST_SIZE) * sizeof(varsize) );
	leftPtr = (varsize *) th_malloc( (LIST_SIZE) * sizeof(varsize) );
	rightPtr = (varsize *) th_malloc( (LIST_SIZE) * sizeof(varsize) );

	if ( ( listData == NULL ) || ( leftPtr == NULL ) || ( rightPtr == NULL ) )
		return( false );

	/* Copy the data table, left and right pointers from ROM to RAM */

	for ( i = 0; i < LIST_SIZE; i++ )
	{
		listData[i] = dataTableROM[i];
		leftPtr[i] = leftPtrTableROM[i];
		rightPtr[i] = rightPtrTableROM[i];
	}

	return true;

	}  /* End of function 'GetTestData' */




/*
 *  Function:  GetInputValues
 *
 *	On each pass of the table lookup, a value must be input for the 'inputToken'
 *	Each time this function is called, the next input value is
 *	pulled from the table in RAM.  The table wraps around, so that input data is
 *	continuous.	 A flag is returned TRUE whenever the table wraps around.
 *
 */

n_int GetInputValues( n_void )
{

	token = inputToken[ tableCount++ ];			/* Get the current test token */

	if ( tableCount < NUM_TESTS )				/* If not the end of the input test data */
		return( 0 );							/*  then we're done... */

	tableCount = 0;								/* Else, reset the input test data pointer */
	return( 1 );

}  /* End of function 'GetInputValues' */





