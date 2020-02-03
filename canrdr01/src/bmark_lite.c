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
 *          $Date: 2002/08/07 22:21:07 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/canrdr01/bmark_lite.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: bmark_lite.c,v $
 * Revision 1.14  2002/08/07 22:21:07  rick
 * Add NI CRC to TH Regular
 *
 * Revision 1.13  2002/07/22 21:59:21  rick
 * General cleanup Beta 2b
 *
 * Revision 1.12  2002/07/19 23:10:23  rick
 * Fix iteration dependant NI CRC's
 *
 * Revision 1.11  2002/07/10 19:01:08  rick
 * Always initialize tcdef->CRC
 *
 * Revision 1.10  2002/05/29 22:25:02  rick
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
 * Revision 1.6  2002/04/25 16:24:53  rick
 * Bug #102: Increment missing off pointer? (Experimantal)
 *
 * Revision 1.5  2002/04/23 18:38:13  rick
 * Remove unused variables
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
#define ITERATIONS 200000	/* required iterations for crc */
#else
#define ITERATIONS 200000	/* recommended iterations for benchmark */
#endif
#endif

#if CRC_CHECK
#define EXPECTED_CRC	0x0000
#elif NON_INTRUSIVE_CRC_CHECK
#define EXPECTED_CRC 0xcb20
#else
#define EXPECTED_CRC	0x0000
#endif

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/

TCDef the_tcdef = 
{
	"AUT canrdr01    ",
    EEMBC_MEMBER_COMPANY, 
    EEMBC_PROCESSOR, 
    EEMBC_TARGET, 
    "Alg. #4 - Response to Remote Request(CAN)V1.0E0 - canrdr00", 
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

/*******************************************************************************
    Local Data                                                                  
*******************************************************************************/

n_int   *RAMfile ;          /* Pointer to test output RAM file */
n_int   *RAMfilePtr ;       /* Pointer to position in output RAM file */
n_int   RAMfileSize ;       /* Size of the debug output RAM file */
n_int   tableCount ;        /* Number of passes through table */
n_int   *RAMfileEOF;        /* points to end of RAM file */
n_int   RAMfile_increment;  /* difference between varsize and n_int */
n_int   *RAMfileFree;        /* NOT PART OF BENCHMARK to free RAMfile */

varsize inputData ;     /* The input data from the CAN message stream */
varsize *inpVariable ;  /* Pointer to array of input data values */

varsize engineTemp ;    /* Phony engine data to xmit */
varsize engineLoad ;    /* Phony engine data to xmit */
varsize engineSpeed ;   /* Phony engine data to xmit */
varsize batteryVolts ;  /* Phony engine data to xmit */

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
        "\nEEMBC Automotive/Industrial Subcommittee Benchmarks ( c )1998\n"
        "Algorithm 4 :  Response to Remote Request( CAN )Rev. 1.0B0\n" ; 
    char *szHeader = 
        "\n\naddr, ext addr, EXTID, IDE, RTR, length, data1...8, counter\n" ; 
    char szDebug[100] ; 
#endif /* BMDEBUG */
    static char szDataSize[40] ; 
    int isTableLooped = false ; /* Input test data table looped */
    varsize xmitBuffer ;        /* The simulated xmit buffer register */
    varsize i1, i2, i3 ; 
    char *msgPtr1 ;             /* Pointer to the CAN message block */
    char *msgPtr2 ; 
    char *msgPtr3 ; 
    int length1 = 0 ;           /* A message length counter */
    int length2 = 0 ; 
    int length3 = 0 ; 
    int msgState1 = LOOKING ;   /* CAN message processor state machine index */
    int msgState2 = LOOKING ; 
    int msgState3 = LOOKING ; 
    CAN_MESSAGE message1 ;      /* The received CAN message block */
    CAN_MESSAGE message2 ; 
    CAN_MESSAGE message3 ; 
    CAN_MESSAGE xmit1 ;         /* The xmit CAN message block */
    CAN_MESSAGE xmit2 ; 
    CAN_MESSAGE xmit3 ; 
    int isAddressMatch1 = false ; 
/* unused variables
    int isAddressMatch2 = false ; 
    int isAddressMatch3 = false ; 
*/
    /* Unused */
    argc = argc ;
    argv = argv ;

    /* Variable initializations at t=0 */    
    /* Start out at beginning of input test data */
    tableCount = 0 ;
    /* Initialize just to eliminate compiler warnings */
    msgPtr1 = (char*)&message1 ;    
    msgPtr2 = (char*)&message2 ; 
    msgPtr3 = (char*)&message3 ; 

    /* If debug output is desired, then must allocate some RAM... */    
    RAMfilePtr = 0 ;    /* Point to beginning of test output file */
    /* Set size of output file( 1K )*/    
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
    szDataSize[0] = (char)( '0' + DATA_SIZE ) ; 
    szDataSize[1] = '\0' ; 
#endif /* BMDEBUG */


    /* Initialize the test data -- stimuli for the algorithm. */   
    /* Allocate for the test input data table */
    if( !GetTestData() )    
    {
        th_exit( THE_OUT_OF_MEMORY, "Cannot Allocate Memory %s : %d",
            __FILE__, __LINE__ ) ; 
    }

    /* Tell the host that the test has begun */
      tcdef->CRC = 0;
    th_signal_start() ;    

#if BMDEBUG
    DebugOut( szTitle ) ;    /* Print the title message in RAM file */
    DebugOut( szDataSize ) ; /*  and the data size */
    DebugOut( szHeader ) ;   /* Print the test output file hdr in RAM file */
#endif /* BMDEBUG */

    /* This is the actual benchmark algorithm. */    

    /* 
    * This kernel will perform a simulation of a portion of the CAN
    * protocol, presuming some minimal hardware support for CAN.
    * Input data will be in the form of CAN remote request messages
    * which must be recognized and responded to.  The input data will
    * be taken from the test data file.  The input data will be comprised
    * of random CAN messages, some of which will be addressed to this
    * "node", while others are to be ignored.
    *
    * This kernel presumes that we are in a "Basic CAN" environment, 
    * that is, only the basic I/O functions are performed in hardware.
    * The basic hardware functions include generation of the CAN bit-
    * stream, bit-stream error checking, and reception of the bit-stream.
    * Presumably there may be some receive or transmit buffering, but
    * that is all transparent to this kernel.
    *
    * Decision of whether a received message is to be accepted by this
    * node must be made in this kernel, that is, there is no hardware
    * support assumed for this function( called "acceptance filtering" ).
    * All message management functions are embodied here, without hardware
    * support.
    *
    * Normally CAN data transmission is autonomous, with the data source
    * node sending out status messages whenever it wants.  "Response to
    * Remote Request" refers to the fact that destination( receiving )nodes
    * may instead request data from the source, via a "Remote Frame".  The
    * Remote Frame contains the address of the desired source in the Identifier
    * Field.  The Remote Frame differs from a Data Frame( that which is sent
    * by the data source )in that the state of the RTR bit( Remote Transmission
    * Request )signifies "receiving" and there is no Data Field in the message.
    *
    * For our "Response to Remote Request" algorithm, we must check all
    * incoming messages to see if the Identifier Field matches any of a number
    * of addresses which this node must respond to.  All incoming messages will
    * have to be processed at the front end, even those that will be rejected, 
    * because this is a Basic CAN environment.  So we must look at the message
    * Identifier Field, decide if it is for us, if not we must ignore the rmndr
    * of data bytes in the message and wait for the next message.  If the msg
    * has this node's address( a block of addresses may be assigned tothis node 
    * then we must store the message( Remote Frames will not have data though),
    * and spill through a decision tree to see what we're going to do about the
    * request.
    *
    * Finally, a response message must be built up with the appropriate Data 
    * Field to be fed to the transmitter.  Since there's no real hardware here,
    * the response message will be the output data for this kernel.  We will
    * presume that this node is some kind of Engine Control Unit, and that
    * we're going to send out various parameters like Engine Temperature, Load,
    * Engine Speed( RPM ), Battery Volts, etc.  Data structures will be used to
    * define each accepted message as far as the corresponding Message
    * Identifier( address ), number of bytes in that message's Data Field,
    * and pointers to the sources for each of the bytes in the
    * Data Field( i.e., the engine parameters mentioned above ).
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

        /* Gets 'inputData' value from test data*/
        isTableLooped = GetInputValues() ;    

#if BMDEBUG        
        th_sprintf( szDebug, "[%8ld]%s", (long)inputData, 
            msgState1 ? " " : "*" ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Process the CAN message strings : */    
        switch( msgState1 )
        {        
        case LOOKING :
        
            /* Point to the message block */
            msgPtr1 = (char*)&message1 ;    

            /* Clear it out */
            for( i1 = 0 ; i1 < (varsize) sizeof( message1 ) ; i1++ ) {
                *msgPtr1++ = 0 ; 
            }
        
            /* Save the message ID */
            message1.address = (unsigned short)inputData ;   

            /* Look at the incoming message ID */
            switch( message1.address )
            {        
            /* Is it one of ours ? */
            case ENG_TEMPERATURE :    
            case LOAD : 
            case ENG_SPEED : 
            case BATTERY_VOLTS : 
                /* Yes, remember we got ours */
                isAddressMatch1 = true ;
                break ; 

            default :
                /* Else, it's not our message,  so remember that */
                isAddressMatch1 = false ;   
                break ; 
            } /* End of 'switch( inputData )' */

            /* We received the message ID */
            msgState1 = GOT_ID ;    
            break ; 

        case GOT_ID : 
            /* Save the extended ID( if any )*/
            message1.ext_address = inputData ; 
            /* We received the extended ID */
            msgState1 = GOT_EXTID ;    
            break ; 

        case GOT_EXTID : 
            /* Save the state of IDE */
            message1.ide = inputData ;    
            /* ...and remember we did */
            msgState1 = GOT_IDE ;    
            break ; 

        case GOT_IDE : 
            /* Save the state of RTR */
            message1.rtr = inputData ;    
            /* ...and remember we did */
            msgState1 = GOT_RTR ;   
            break ; 

        case GOT_RTR : 
            /* Save the message length */
            message1.length = inputData ;    

            /* Did we get a valid Remote Request ? */    
            if( message1.rtr && isAddressMatch1 && ( message1.length == 0 ) )
            {        
                /* Point to the xmit message block and clear it */
                msgPtr1 = (char*)&xmit1 ;    
                for( i1 = 0 ; i1 < (varsize) sizeof( message1 ) ; i1++ ) 
                {
                    *msgPtr1++ = 0 ;
                }

                /* Respond with same message ID */
                xmit1.address = message1.address ;    
                /* Depending on which message we got, */
                switch( message1.address )
                {        
                case ENG_TEMPERATURE :    
                    /* If it's Engine Temperature, then */
                    /* ...set the message length */
                    xmit1.length = ENG_TEMP_LENGTH ;    
                    /* ...and set the selected value */
                    xmit1.data1 = (char)( engineTemp &0x00FF ) ;
                    xmit1.data2 = (char)( engineTemp >> 8 ) ; 
                    break ; 

                case LOAD :   
                    /* If it's Engine Load, then */
                    /* ...set the message length */
                    xmit1.length = LOAD_LENGTH ;    
                    /* ...and set the selected value */
                    xmit1.data1 = (char)( engineLoad &0x00FF ) ;    
                    xmit1.data2 = (char)( engineLoad >> 8 ) ; 
                    break ; 

                case ENG_SPEED :    
                    /* If it's Engine Speed, then */
                    /* ...set the message length */
                    xmit1.length = ENG_SPEED_LENGTH ;    
                    /* ...and set the selected value */
                    xmit1.data1 = (char)( engineSpeed &0x00FF ) ;    
                    xmit1.data2 = (char)( engineSpeed >> 8 ) ; 
                    break ; 

                default :    
                    /* Else, it must be Battery Voltage, */
                    /* ...set the message length */
                    xmit1.length = BATTERY_LENGTH ;
                    /* ...and set the selected value */
                    xmit1.data1 = (n_char)batteryVolts ;    
                    break ; 

                } /* End of 'switch( inputData )' */

                /* We only allow standard CAN addressing */
                xmit1.ext_address = 0 ;    

                /* Now, will trasmit the response to Remote Request : */    
                /* Xmit the address( ID )*/
                xmitBuffer = xmit1.address ; 

#if BMDEBUG        
                /* Display the output to 'xmitBuffer' */
                th_sprintf( szDebug, "  << %5ld", (long)xmitBuffer ) ;
                DebugOut( szDebug ) ; 
#else           /* Or write value directly to RAM file */
                WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */

                /* ...and the extended address */
                xmitBuffer = xmit1.ext_address ;    

#if BMDEBUG        
                /* Display the output to 'xmitBuffer' */
                th_sprintf( szDebug, ", %5ld", (long)xmitBuffer ) ;    
                DebugOut( szDebug ) ; 
#else           /* Or write value directly to RAM file */
                WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */

                /*  and the Data Field length */
                xmitBuffer = xmit1.length ;    

#if BMDEBUG        
                /* Display the output to 'xmitBuffer' */
                th_sprintf( szDebug, ", %5ld", (long)xmitBuffer ) ;    
                DebugOut( szDebug ) ; 
#else           /* Or write value directly to RAM file */
                WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */

                /* Point to the Data Field */
                msgPtr1 = &xmit1.data1 ;    

                /* Xmit all the data */
                for( i1 = 0 ; i1 < xmit1.length ; i1++ )
                {        
                    xmitBuffer = *msgPtr1++ ; 
#if BMDEBUG        
                    /* Xmit all the data */
                    th_sprintf( szDebug, ", %5ld", (long)xmitBuffer ) ;    
                    DebugOut( szDebug ) ; 
#else               /* Or write value directly to RAM file */
                    WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */
                }
#if BMDEBUG        
                DebugOut( " >>\n           " ) ; 
#endif /* BMDEBUG */

            } /* End of processing Remote Request */

            /* Did received message have any data ? */
            if( message1.length == 0 )   
            {
                /* No, back to LOOKING... */
                msgState1 = LOOKING ;    
            }
            else 
            {        
                /* Yes, remember how much data */
                length1 = message1.length ;    
                /* Point to message data storage */
                msgPtr1 = &message1.data1 ;    
                /* ...and will copy the data */
                msgState1 = GET_DATA ;    
            }
            break ; 

        case GET_DATA :
            /* Receiving message data... */
            /* Store the data */
            *msgPtr1++ = (n_char)inputData ;    
            /* When we're done storing data, */
            if( --length1 == 0 )  
            {
                /* ...then start LOOKING again */
                msgState1 = LOOKING ;    
            }
            break ; 
        }    /* End of 'switch( msgState )' */


#if BMDEBUG        
        th_sprintf( szDebug, "%6ld, %6ld, %2d, %2d, %2d", 
            (long)message1.address, 
            (long)message1.ext_address,
            message1.ide,
            message1.rtr, 
            message1.length ) ; 
        DebugOut( szDebug ) ; 

        th_sprintf( szDebug, ", %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d\n",
            message1.data1,
            message1.data2,
            message1.data3,
            message1.data4,
            message1.data5,
            message1.data6,
            message1.data7,
            message1.data8 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Second Pass                                                         
        ***********************************************************************/
    
        /* Gets 'inputData' value from test data*/
        isTableLooped += GetInputValues() ;

#if BMDEBUG        
        th_sprintf( szDebug, "[%8ld]%s", (long)inputData, 
            msgState2 ? " " : "*" ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Process the CAN message strings : */    
        switch( msgState2 )
        {        
        case LOOKING :
        
            /* Point to the message block */
            msgPtr2 = (char*)&message2 ;    

            /* Clear it out */
            for( i2 = 0 ; i2 < (varsize) sizeof( message2 ) ; i2++ ) {
                *msgPtr2++ = 0 ; 
            }
        
            /* Save the message ID */
            message2.address = (unsigned short)inputData ;   

            /* Look at the incoming message ID */
            switch( message2.address )
            {        
            /* Is it one of ours ? */
            case ENG_TEMPERATURE :    
            case LOAD : 
            case ENG_SPEED : 
            case BATTERY_VOLTS : 
                /* Yes, remember we got ours */
                isAddressMatch1 = true ;
                break ; 

            default :
                /* Else, it's not our message,  so remember that */
                isAddressMatch1 = false ;   
                break ; 
            } /* End of 'switch( inputData )' */

            /* We received the message ID */
            msgState2 = GOT_ID ;    
            break ; 

        case GOT_ID : 
            /* Save the extended ID( if any )*/
            message2.ext_address = inputData ; 
            /* We received the extended ID */
            msgState2 = GOT_EXTID ;    
            break ; 

        case GOT_EXTID : 
            /* Save the state of IDE */
            message2.ide = inputData ;    
            /* ...and remember we did */
            msgState2 = GOT_IDE ;    
            break ; 

        case GOT_IDE : 
            /* Save the state of RTR */
            message2.rtr = inputData ;    
            /* ...and remember we did */
            msgState2 = GOT_RTR ;   
            break ; 

        case GOT_RTR : 
            /* Save the message length */
            message2.length = inputData ;    

            /* Did we get a valid Remote Request ? */    
            if( message2.rtr && isAddressMatch1 && ( message2.length == 0 ) )
            {        
                /* Point to the xmit message block and clear it */
                msgPtr2 = (char*)&xmit2 ;    
                for( i2 = 0 ; i2 < (varsize) sizeof( message2 ) ; i2++ ) 
                {
                    *msgPtr2++ = 0 ;
                }

                /* Respond with same message ID */
                xmit2.address = message2.address ;    
                /* Depending on which message we got, */
                switch( message2.address )
                {        
                case ENG_TEMPERATURE :    
                    /* If it's Engine Temperature, then */
                    /* ...set the message length */
                    xmit2.length = ENG_TEMP_LENGTH ;    
                    /* ...and set the selected value */
                    xmit2.data1 = (char)( engineTemp &0x00FF ) ;
                    xmit2.data2 = (char)( engineTemp >> 8 ) ; 
                    break ; 

                case LOAD :   
                    /* If it's Engine Load, then */
                    /* ...set the message length */
                    xmit2.length = LOAD_LENGTH ;    
                    /* ...and set the selected value */
                    xmit2.data1 = (char)( engineLoad &0x00FF ) ;    
                    xmit2.data2 = (char)( engineLoad >> 8 ) ; 
                    break ; 

                case ENG_SPEED :    
                    /* If it's Engine Speed, then */
                    /* ...set the message length */
                    xmit2.length = ENG_SPEED_LENGTH ;    
                    /* ...and set the selected value */
                    xmit2.data1 = (char)( engineSpeed &0x00FF ) ;    
                    xmit2.data2 = (char)( engineSpeed >> 8 ) ; 
                    break ; 

                default :    
                    /* Else, it must be Battery Voltage, */
                    /* ...set the message length */
                    xmit2.length = BATTERY_LENGTH ;
                    /* ...and set the selected value */
                    xmit2.data1 = (n_char)batteryVolts ;    
                    break ; 

                } /* End of 'switch( inputData )' */

                /* We only allow standard CAN addressing */
                xmit2.ext_address = 0 ;    

                /* Now, will trasmit the response to Remote Request : */    
                /* Xmit the address( ID )*/
                xmitBuffer = xmit2.address ; 

#if BMDEBUG        
                /* Display the output to 'xmitBuffer' */
                th_sprintf( szDebug, "  << %5ld", (long)xmitBuffer ) ;
                DebugOut( szDebug ) ; 
#else           /* Or write value directly to RAM file */
                WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */

                /* ...and the extended address */
                xmitBuffer = xmit2.ext_address ;    

#if BMDEBUG        
                /* Display the output to 'xmitBuffer' */
                th_sprintf( szDebug, ", %5ld", (long)xmitBuffer ) ;    
                DebugOut( szDebug ) ; 
#else           /* Or write value directly to RAM file */
                WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */

                /*  and the Data Field length */
                xmitBuffer = xmit2.length ;    

#if BMDEBUG        
                /* Display the output to 'xmitBuffer' */
                th_sprintf( szDebug, ", %5ld", (long)xmitBuffer ) ;    
                DebugOut( szDebug ) ; 
#else           /* Or write value directly to RAM file */
                WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */

                /* Point to the Data Field */
                msgPtr2 = &xmit2.data1 ;    

                /* Xmit all the data */
                for( i2 = 0 ; i2 < xmit2.length ; i2++ )
                {        
                    xmitBuffer = *msgPtr2++ ; 
#if BMDEBUG        
                    /* Xmit all the data */
                    th_sprintf( szDebug, ", %5ld", (long)xmitBuffer ) ;    
                    DebugOut( szDebug ) ; 
#else               /* Or write value directly to RAM file */
                    WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */
                }
#if BMDEBUG        
                DebugOut( " >>\n           " ) ; 
#endif /* BMDEBUG */

            } /* End of processing Remote Request */

            /* Did received message have any data ? */
            if( message2.length == 0 )   
            {
                /* No, back to LOOKING... */
                msgState2 = LOOKING ;    
            }
            else 
            {        
                /* Yes, remember how much data */
                length2 = message2.length ;    
                /* Point to message data storage */
                msgPtr2 = &message2.data1 ;    
                /* ...and will copy the data */
                msgState2 = GET_DATA ;    
            }
            break ; 

        case GET_DATA :
            /* Receiving message data... */
            /* Store the data */
            *msgPtr2++ = (n_char)inputData ;    
            /* When we're done storing data, */
            if( --length2 == 0 )  
            {
                /* ...then start LOOKING again */
                msgState2 = LOOKING ;    
            }
            break ; 
        }    /* End of 'switch( msgState )' */


#if BMDEBUG        
        th_sprintf( szDebug, "%6ld, %6ld, %2d, %2d, %2d", 
            (long)message2.address, 
            (long)message2.ext_address,
            message2.ide,
            message2.rtr, 
            message2.length ) ; 
        DebugOut( szDebug ) ; 

        th_sprintf( szDebug, ", %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d\n",
            message2.data1,
            message2.data2,
            message2.data3,
            message2.data4,
            message2.data5,
            message2.data6,
            message2.data7,
            message2.data8 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /***********************************************************************
            Third Pass                                                          
        ***********************************************************************/

        /* Gets 'inputData' value from test data*/
        isTableLooped += GetInputValues() ;

#if BMDEBUG        
        th_sprintf( szDebug, "[%8ld]%s", (long)inputData, 
            msgState3 ? " " : "*" ) ;
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

        /* Process the CAN message strings : */    
        switch( msgState3 )
        {        
        case LOOKING :
        
            /* Point to the message block */
            msgPtr3 = (char*)&message3 ;    

            /* Clear it out */
            for( i3 = 0 ; i3 < (varsize) sizeof( message3 ) ; i3++ ) {
                *msgPtr3++ = 0 ; 
            }
        
            /* Save the message ID */
            message3.address = (unsigned short)inputData ;   

            /* Look at the incoming message ID */
            switch( message3.address )
            {        
            /* Is it one of ours ? */
            case ENG_TEMPERATURE :    
            case LOAD : 
            case ENG_SPEED : 
            case BATTERY_VOLTS : 
                /* Yes, remember we got ours */
                isAddressMatch1 = true ;
                break ; 

            default :
                /* Else, it's not our message,  so remember that */
                isAddressMatch1 = false ;   
                break ; 
            } /* End of 'switch( inputData )' */

            /* We received the message ID */
            msgState3 = GOT_ID ;    
            break ; 

        case GOT_ID : 
            /* Save the extended ID( if any )*/
            message3.ext_address = inputData ; 
            /* We received the extended ID */
            msgState3 = GOT_EXTID ;    
            break ; 

        case GOT_EXTID : 
            /* Save the state of IDE */
            message3.ide = inputData ;    
            /* ...and remember we did */
            msgState3 = GOT_IDE ;    
            break ; 

        case GOT_IDE : 
            /* Save the state of RTR */
            message3.rtr = inputData ;    
            /* ...and remember we did */
            msgState3 = GOT_RTR ;   
            break ; 

        case GOT_RTR : 
            /* Save the message length */
            message3.length = inputData ;    

            /* Did we get a valid Remote Request ? */    
            if( message3.rtr && isAddressMatch1 && ( message3.length == 0 ) )
            {        
                /* Point to the xmit message block and clear it */
                msgPtr3 = (char*)&xmit3 ;    
                for( i3 = 0 ; i3 < (varsize) sizeof( message3 ) ; i3++ ) 
                {
                    *msgPtr3++ = 0 ;
                }

                /* Respond with same message ID */
                xmit3.address = message3.address ;    
                /* Depending on which message we got, */
                switch( message3.address )
                {        
                case ENG_TEMPERATURE :    
                    /* If it's Engine Temperature, then */
                    /* ...set the message length */
                    xmit3.length = ENG_TEMP_LENGTH ;    
                    /* ...and set the selected value */
                    xmit3.data1 = (char)( engineTemp &0x00FF ) ;
                    xmit3.data2 = (char)( engineTemp >> 8 ) ; 
                    break ; 

                case LOAD :   
                    /* If it's Engine Load, then */
                    /* ...set the message length */
                    xmit3.length = LOAD_LENGTH ;    
                    /* ...and set the selected value */
                    xmit3.data1 = (char)( engineLoad &0x00FF ) ;    
                    xmit3.data2 = (char)( engineLoad >> 8 ) ; 
                    break ; 

                case ENG_SPEED :    
                    /* If it's Engine Speed, then */
                    /* ...set the message length */
                    xmit3.length = ENG_SPEED_LENGTH ;    
                    /* ...and set the selected value */
                    xmit3.data1 = (char)( engineSpeed &0x00FF ) ;    
                    xmit3.data2 = (char)( engineSpeed >> 8 ) ; 
                    break ; 

                default :    
                    /* Else, it must be Battery Voltage, */
                    /* ...set the message length */
                    xmit3.length = BATTERY_LENGTH ;
                    /* ...and set the selected value */
                    xmit3.data1 = (n_char)batteryVolts ;    
                    break ; 

                } /* End of 'switch( inputData )' */

                /* We only allow standard CAN addressing */
                xmit3.ext_address = 0 ;    

                /* Now, will trasmit the response to Remote Request : */    
                /* Xmit the address( ID )*/
                xmitBuffer = xmit3.address ; 

#if BMDEBUG        
                /* Display the output to 'xmitBuffer' */
                th_sprintf( szDebug, "  << %5ld", (long)xmitBuffer ) ;
                DebugOut( szDebug ) ; 
#else           /* Or write value directly to RAM file */
                WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */

                /* ...and the extended address */
                xmitBuffer = xmit3.ext_address ;    

#if BMDEBUG        
                /* Display the output to 'xmitBuffer' */
                th_sprintf( szDebug, ", %5ld", (long)xmitBuffer ) ;    
                DebugOut( szDebug ) ; 
#else           /* Or write value directly to RAM file */
                WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */

                /*  and the Data Field length */
                xmitBuffer = xmit3.length ;    

#if BMDEBUG        
                /* Display the output to 'xmitBuffer' */
                th_sprintf( szDebug, ", %5ld", (long)xmitBuffer ) ;    
                DebugOut( szDebug ) ; 
#else           /* Or write value directly to RAM file */
                WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */

                /* Point to the Data Field */
                msgPtr3 = &xmit3.data1 ;    

                /* Xmit all the data */
                for( i3 = 0 ; i3 < xmit3.length ; i3++ )
                {        
                    xmitBuffer = *msgPtr3++ ; 
#if BMDEBUG        
                    /* Xmit all the data */
                    th_sprintf( szDebug, ", %5ld", (long)xmitBuffer ) ;    
                    DebugOut( szDebug ) ; 
#else               /* Or write value directly to RAM file */
                    WriteOut( xmitBuffer ) ; 
#endif /* BMDEBUG */
                }
#if BMDEBUG        
                DebugOut( " >>\n           " ) ; 
#endif /* BMDEBUG */

            } /* End of processing Remote Request */

            /* Did received message have any data ? */
            if( message3.length == 0 )   
            {
                /* No, back to LOOKING... */
                msgState3 = LOOKING ;    
            }
            else 
            {        
                /* Yes, remember how much data */
                length3 = message3.length ;    
                /* Point to message data storage */
                msgPtr3 = &message3.data1 ;    
                /* ...and will copy the data */
                msgState3 = GET_DATA ;    
            }
            break ; 

        case GET_DATA :
            /* Receiving message data... */
            /* Store the data */
            *msgPtr3++ = (n_char)inputData ;    
            /* When we're done storing data, */
            if( --length3 == 0 )  
            {
                /* ...then start LOOKING again */
                msgState3 = LOOKING ;    
            }
            break ; 
        }    /* End of 'switch( msgState )' */

#if BMDEBUG        
        th_sprintf( szDebug, "%6ld, %6ld, %2d, %2d, %2d", 
            (long)message3.address, 
            (long)message3.ext_address,
            message3.ide,
            message3.rtr, 
            message3.length ) ; 
        DebugOut( szDebug ) ; 

        th_sprintf( szDebug, ", %2d, %2d, %2d, %2d, %2d, %2d, %2d, %2d\n",
            message3.data1,
            message3.data2,
            message3.data3,
            message3.data4,
            message3.data5,
            message3.data6,
            message3.data7,
            message3.data8 ) ; 
        DebugOut( szDebug ) ; 
#endif /* BMDEBUG */

#if BMDEBUG        
        th_sprintf( szDebug, ", %08lX\n", (long)loop_cnt ) ; 
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
/* NON_INTRUSIVE_CRC_CHECK not ok, only on stimuli */
#if		NON_INTRUSIVE_CRC_CHECK
/* Final results are iteration dependant */
    tcdef->CRC=0;
	for (loop_cnt=0;loop_cnt<NUM_TESTS;loop_cnt++) {
	tcdef->CRC = Calc_crc32((e_u32)inpVariable[loop_cnt],tcdef->CRC);
	}
#elif	CRC_CHECK
	tcdef->CRC=0;
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
    xil_printf(">>     Start of canrdr...\n\r");
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
            th_free(inpVariable);
            xil_printf(">>   Test is working just fine, iteration: %8ld\n\r",benchIter++);
        }
    }
    xil_printf(">>      canrdr test is finished\n\r");
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

