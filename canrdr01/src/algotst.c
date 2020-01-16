/*==============================================================================
 *$RCSfile: algotst.c,v $
 *
 *   DESC : CAN Reader algorithm
 *
 * AUTHOR : 
 *
 *  EEMBC : Automotive/Industrial Subcommittee 
 *
 *    CVS : $Revision: 1.3 $
 *          $Date: 2002/07/19 23:10:23 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/canrdr01/algotst.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: algotst.c,v $
 * Revision 1.3  2002/07/19 23:10:23  rick
 * Fix iteration dependant NI CRC's
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

#include "algo.h"

/*  DECLARATIONS  */    

/*
* The following test data represents the CAN message input data stream.
*
* Presumably, the Basic CAN environment provides us with enough hardware
* to decode the bit stream, resolving any CRC, framing or other errors, 
* and producing the message Identifier Field and the Data Length. And we
* need to know the status of the RTR( Remote Transmit Request )bit.
*
* There are 2048 maximum possible message ID's for Standard CAN.
* Standard CAN has an 11 bit message ID.
* Standard CAN has an Identifier Extension bit false.
* Extended CAN has an Extended Address field of 18 bits.
* Extended CAN has an Identifier Extension bit true.
* A Remote Request has the RTR bit true, and contains no data.
* There are up to 8 bytes of data.
*
* 2/18/99 Revised the test data to provide three streams of CAN
* messages, for the expanded kernel.
*
*/    

/*
* Simulated CAN message format : 
*
* Std. ID, Ext. ID, IDE bit, RTR bit, Data Length, {optional data}
*
*/    

const varsize inpVariableROM[] = 
{        
    9933,   /* Phony 'engineTemp' */

    4456,   /* Phony 'engineLoad' */

    5511,   /* Phony 'engineSpeed' */

    66,     /* Phony 'batteryVolts' */


    2000, 

    1200, 

    1100,   /* ID=2000, EXTID=0, IDE=0, RTR=1, no data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1100, 

    2000, 

    1200,    /* ID=1100, EXTID=0, IDE=0, RTR=1, no data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1200, 

    900, 

    800, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1300, 

    1400, 

    1500, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1400, 

    1500, 

    1300, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1500, 

    1600, 

    1700, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1600, 

    1700, 

    300, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1700, 

    300, 

    1600, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1800, 

    2000, 

    1900, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1900, 

    1800, 

    2000, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    2000, 

    1900, 

    100, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1000, 

    1100, 

    1001, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    900, 

    100, 

    200, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    2000, 

    1800, 

    1700,    /* ID=2000, EXTID=100, IDE=1, RTR=0, no data */

    100, 

    100, 

    100, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    2000, 

    1700, 

    2000,    /* ID=2000, EXTID=100, IDE=1, RTR=1, no data */

    100, 

    100, 

    100, 

    1, 

    1, 

    1, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    2000, 

    1900, 

    1800,    /* ID=2000, EXTID=100, IDE=1, RTR=0, 1 data */

    100, 

    200, 

    300, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    12, 

    34, 

    56, 


    2000, 

    1900, 

    1800,    /* ID=2000, EXTID=100, IDE=1, RTR=0, 2 data */

    100, 

    200, 

    300, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 

    2, 

    2, 

    2, 

    12, 

    34, 

    56, 

    34, 

    56, 

    78, 


    2000, 

    100, 

    200,    /* ID=2000, EXTID=100, IDE=1, RTR=0, 3 data */

    100, 

    200, 

    3400, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 

    3, 

    3, 

    3, 

    12, 

    99, 

    11, 

    34, 

    67, 

    43, 

    56, 

    98, 

    23, 


    2000, 

    1500, 

    1300,    /* ID=2000, EXTID=100, IDE=1, RTR=0, 4 data */

    100, 

    454, 

    678, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 

    4, 

    4, 

    4, 

    12, 

    98, 

    23, 

    34, 

    32, 

    76, 

    56, 

    123, 

    88, 

    78, 

    200, 

    100, 


    2000, 

    345, 

    687,    /* ID=2000, EXTID=100, IDE=1, RTR=0, 5 data */

    100, 

    243, 

    998, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 

    5, 

    5, 

    5, 

    12, 

    34, 

    64, 

    34, 

    120, 

    38, 

    56, 

    9, 

    23, 

    78, 

    0, 

    4, 

    89, 

    2, 

    8, 


    2000, 

    10, 

    1,    /* ID=2000, EXTID=100, IDE=1, RTR=0, 6 data */

    100, 

    24, 

    213, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 

    6, 

    6, 

    6, 

    12, 

    99, 

    88, 

    34, 

    77, 

    6, 

    56, 

    55, 

    44, 

    78, 

    33, 

    22, 

    89, 

    11, 

    0, 

    01, 

    2, 

    3, 


    2000, 

    980, 

    765,    /* ID=2000, EXTID=100, IDE=1, RTR=0, 7 data */

    100, 

    234, 

    567, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 

    7, 

    7, 

    7, 

    12, 

    28, 

    29, 

    34, 

    93, 

    88, 

    56, 

    26, 

    85, 

    78, 

    92, 

    39, 

    89, 

    95, 

    74, 

    01, 

    20, 

    45, 

    23, 

    1, 

    6, 


    2000, 

    100, 

    1800,    /* ID=2000, EXTID=100, IDE=1, RTR=0, 8 data */

    100, 

    345, 

    678, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 

    8, 

    8, 

    8, 

    12, 

    45, 

    23, 

    34, 

    56, 

    78, 

    56, 

    01, 

    23, 

    78, 

    34, 

    56, 

    89, 

    45, 

    67, 

    01, 

    89, 

    98, 

    23, 

    12, 

    34, 

    45, 

    11, 

    22, 


    1000, 

    1000, 

    1000,    /* ID=1000, EXTID=0, IDE=0, RTR=0, no data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    1200, 

    1234, 

    1456,    /* ID=1200, EXTID=0, IDE=0, RTR=1, no data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1234, 

    1345, 

    987,    /* ID=1234, EXTID=0, IDE=0, RTR=0, 1 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    12, 

    23, 

    34, 


    234, 

    345, 

    567,    /* ID=234,  EXTID=0, IDE=0, RTR=0, 2 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    2, 

    2, 

    2, 

    12, 

    34, 

    56, 

    34, 

    56, 

    78, 


    1230, 

    1245, 

    1256,    /* ID=1230, EXTID=0, IDE=0, RTR=0, 3 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    3, 

    3, 

    3, 

    12, 

    55, 

    66, 

    34, 

    77, 

    8, 

    56, 

    99, 

    32, 


    123, 

    234, 

    345,    /* ID=123,  EXTID=0, IDE=0, RTR=0, 4 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    4, 

    4, 

    4, 

    12, 

    34, 

    56, 

    34, 

    78, 

    90, 

    56, 

    01, 

    23, 

    78, 

    99, 

    88, 


    1000, 

    1100, 

    1001,    /* ID=1000, EXTID=0, IDE=0, RTR=0, 5 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    5, 

    5, 

    5, 

    12, 

    22, 

    33, 

    34, 

    44, 

    55, 

    56, 

    66, 

    77, 

    78, 

    88, 

    99, 

    89, 

    0, 

    1, 


    1100, 

    1000, 

    1001,    /* ID=1100, EXTID=0, IDE=0, RTR=0, 6 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    6, 

    6, 

    6, 

    12, 

    34, 

    56, 

    34, 

    78, 

    90, 

    56, 

    01, 

    23, 

    78, 

    45, 

    67, 

    89, 

    89, 

    01, 

    01, 

    11, 

    22, 


    1400, 

    1300, 

    1200,    /* ID=1400, EXTID=0, IDE=0, RTR=0, 7 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    7, 

    7, 

    7, 

    12, 

    34, 

    56, 

    34, 

    78, 

    90, 

    56, 

    12, 

    34, 

    78, 

    56, 

    78, 

    89, 

    99, 

    88, 

    01, 

    7, 

    6, 

    23, 

    5, 

    4, 


    1500, 

    1400, 

    1300,    /* ID=1500, EXTID=0, IDE=0, RTR=0, 8 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    8, 

    8, 

    8, 

    12, 

    33, 

    44, 

    34, 

    55, 

    66, 

    56, 

    77, 

    88, 

    78, 

    99, 

    0, 

    89, 

    11, 

    22, 

    01, 

    120, 

    123, 

    23, 

    101, 

    100, 

    45, 

    99, 

    98, 


    1000, 

    1001, 

    1100,    /* ID=1000, EXTID=0, IDE=0, RTR=0, no data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    1200, 

    1300, 

    1400,    /* ID=1200, EXTID=0, IDE=0, RTR=1, no data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    1234, 

    1345, 

    1456,    /* ID=1234, EXTID=0, IDE=0, RTR=0, 1 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    12, 

    34, 

    56, 


    234, 

    345, 

    456,    /* ID=234,  EXTID=0, IDE=0, RTR=0, 2 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    2, 

    2, 

    2, 

    34, 

    33, 

    44, 

    12, 

    55, 

    66, 


    1230, 

    1234, 

    1543,    /* ID=1230, EXTID=0, IDE=0, RTR=0, 3 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    3, 

    3, 

    3, 

    56, 

    11, 

    22, 

    98, 

    44, 

    55, 

    76, 

    99, 

    100, 


    123, 

    234, 

    345,    /* ID=123,  EXTID=0, IDE=0, RTR=0, 4 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    4, 

    4, 

    4, 

    12, 

    23, 

    34, 

    77, 

    88, 

    99, 

    88, 

    0, 

    1, 

    99, 

    2, 

    3, 


    1000, 

    1001, 

    1100,    /* ID=1000, EXTID=0, IDE=0, RTR=0, 5 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    5, 

    5, 

    5, 

    1, 

    2, 

    3, 

    34, 

    45, 

    56, 

    56, 

    67, 

    89, 

    37, 

    99, 

    88, 

    89, 

    0, 

    1, 


    1100, 

    1000, 

    1001,    /* ID=1100, EXTID=0, IDE=0, RTR=0, 6 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    6, 

    6, 

    6, 

    2, 

    1, 

    3, 

    58, 

    4, 

    5, 

    56, 

    6, 

    7, 

    78, 

    8, 

    9, 

    94, 

    10, 

    11, 

    01, 

    12, 

    13, 


    1400, 

    1300, 

    1500,    /* ID=1400, EXTID=0, IDE=0, RTR=0, 7 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    7, 

    7, 

    7, 

    12, 

    34, 

    56, 

    84, 

    76, 

    54, 

    56, 

    34, 

    56, 

    78, 

    89, 

    99, 

    89, 

    12, 

    34, 

    01, 

    11, 

    22, 

    20, 

    2, 

    56, 


    1500, 

    1345, 

    1567,    /* ID=1500, EXTID=0, IDE=0, RTR=0, 8 data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    8, 

    8, 

    8, 

    12, 

    01, 

    92, 

    33, 

    83, 

    74, 

    56, 

    65, 

    76, 

    55, 

    48, 

    29, 

    89, 

    10, 

    02, 

    11, 

    99, 

    88, 

    99, 

    12, 

    34, 

    15, 

    55, 

    66, 


    1000, 

    1100, 

    1001,    /* ID=1000, EXTID=0, IDE=0, RTR=0, no data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    1200, 

    1800, 

    1900,    /* ID=1200, EXTID=0, IDE=0, RTR=1, no data */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    ENG_TEMPERATURE, 

    LOAD, 

    BATTERY_VOLTS,    /* CAN Remote Request for Engine Temperature */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    ENG_TEMPERATURE, 

    LOAD, 

    BATTERY_VOLTS,    /* CAN( error )Remote Request for Engine Temperature */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    LOAD, 

    BATTERY_VOLTS, 

    ENG_TEMPERATURE,    /* CAN Remote Request for Engine Load */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    LOAD, 

    BATTERY_VOLTS, 

    ENG_TEMPERATURE,    /* CAN( error )Remote Request for Engine Load */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    ENG_SPEED, 

    ENG_TEMPERATURE, 

    LOAD,    /* CAN Remote Request for Engine Speed( RPM )*/

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    ENG_SPEED, 

    ENG_TEMPERATURE, 

    LOAD,    /* CAN( error )Remote Request for Engine Speed( RPM )*/

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    BATTERY_VOLTS, 

    ENG_SPEED, 

    ACCELERATOR,    /* CAN Remote Request for Battery Voltage */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    BATTERY_VOLTS, 

    ENG_SPEED, 

    ACCELERATOR,    /* CAN( error )Remote Request for Battery Voltage */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    ACCELERATOR, 

    ACCELERATOR, 

    ACCELERATOR,    /* CAN message for Accelerator Position */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    2, 

    2, 

    2, 

    33, 

    44, 

    55, 

    33, 

    11, 

    22, 


    ACCELERATOR, 

    ROAD_SPEED, 

    LOAD,    /* CAN Remote Request for Accelerator Position */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    ROAD_SPEED, 

    ACCELERATOR, 

    ROAD_SPEED,    /* CAN message for Road Speed */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    2, 

    2, 

    2, 

    99, 

    88, 

    77, 

    99, 

    88, 

    77, 


    ROAD_SPEED, 

    ROAD_SPEED, 

    ROAD_SPEED,    /* CAN Remote Request for Road Speed */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    ENG_TEMPERATURE, 

    BATTERY_VOLTS, 

    ENG_SPEED,    /* CAN Remote Request for Engine Temperature */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    ENG_TEMPERATURE, 

    BATTERY_VOLTS, 

    ENG_SPEED,    /* CAN( error )Remote Request for Engine Temperature */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    ENG_TEMPERATURE, 

    BATTERY_VOLTS, 

    ENG_SPEED,    /* CAN( error )Remote Request for Engine Temperature */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    12, 

    34, 

    56, 


    LOAD, 

    BATTERY_VOLTS, 

    ENG_SPEED,    /* CAN Remote Request for Engine Load */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    LOAD, 

    ENG_TEMPERATURE, 

    BATTERY_VOLTS,    /* CAN( error )Remote Request for Engine Load */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    LOAD, 

    ENG_TEMPERATURE, 

    BATTERY_VOLTS,    /* CAN( error )Remote Request for Engine Load */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    3, 

    3, 

    3, 

    12, 

    99, 

    88, 

    34, 

    1, 

    2, 

    56, 

    45, 

    67, 


    ENG_SPEED, 

    ENG_TEMPERATURE, 

    BATTERY_VOLTS,    /* CAN Remote Request for Engine Speed( RPM )*/

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    ENG_SPEED, 

    ENG_TEMPERATURE, 

    BATTERY_VOLTS,    /* CAN( error )Remote Request for Engine Speed( RPM )*/

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    ENG_SPEED, 

    ENG_TEMPERATURE, 

    BATTERY_VOLTS,    /* CAN( error )Remote Request for Engine Speed( RPM )*/

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    5, 

    5, 

    5, 

    12, 

    98, 

    76, 

    34, 

    54, 

    32, 

    56, 

    10, 

    01, 

    78, 

    23, 

    45, 

    90, 

    66, 

    77, 


    BATTERY_VOLTS, 

    ENG_SPEED, 

    ENG_TEMPERATURE,    /* CAN Remote Request for Battery Voltage */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    BATTERY_VOLTS, 

    ENG_SPEED, 

    ENG_TEMPERATURE,    /* CAN( error )Remote Request for Battery Voltage */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 


    BATTERY_VOLTS, 

    ENG_SPEED, 

    ENG_TEMPERATURE,    /* CAN( error )Remote Request for Battery Voltage */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    8, 

    8, 

    8, 

    1, 

    9, 

    8, 

    2, 

    7, 

    6, 

    3, 

    5, 

    4, 

    4, 

    3, 

    2, 

    5, 

    1, 

    0, 

    6, 

    99, 

    88, 

    7, 

    77, 

    66, 

    8, 

    55, 

    44, 


    ACCELERATOR, 

    ENG_SPEED, 

    ENG_TEMPERATURE,    /* CAN message for Accelerator Position */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    2, 

    2, 

    2, 

    33, 

    44, 

    55, 

    33, 

    66, 

    77, 


    ACCELERATOR, 

    BATTERY_VOLTS, 

    LOAD,    /* CAN Remote Request for Accelerator Position */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0, 


    ROAD_SPEED, 

    ENG_SPEED, 

    ENG_TEMPERATURE,    /* CAN message for Road Speed */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    2, 

    2, 

    2, 

    99, 

    88, 

    66, 

    99, 

    88, 

    66, 


    ROAD_SPEED, 

    ENG_SPEED, 

    ENG_TEMPERATURE,    /* CAN Remote Request for Road Speed */

    0, 

    0, 

    0, 

    0, 

    0, 

    0, 

    1, 

    1, 

    1, 

    0, 

    0, 

    0

} ; /* End of test values :  inpVariableROM[] */

/*******************************************************************************
    Functions                                                                   
*******************************************************************************/

/*
*  Function :  GetTestData
*
*  Builds the table of input data which represents the CAN message data
*  input stream.  This is the "real world" data stream which drives the
*  algorithm.  This will eventually be replaced by downloading the test
*  data from the host in order to prove adherence to the algorithm.
*
*/    

n_int
GetTestData( n_void )
{        
    n_int i ; 

    inpVariable = (varsize*)th_malloc( (NUM_TESTS + 1) * sizeof( varsize ) ) ;

    if( inpVariable == NULL )
    {
        return false ; 
    }

    /*  Copy the test values from ROM to RAM  */    
    engineTemp = inpVariableROM[0] ;    /* Phony engine data to xmit */
    engineLoad = inpVariableROM[1] ;    /* Phony engine data to xmit */
    engineSpeed = inpVariableROM[2] ;   /* Phony engine data to xmit */
    batteryVolts = inpVariableROM[3] ;  /* Phony engine data to xmit */

    for( i = 0 ; i < NUM_TESTS ; i++ )
    {
        inpVariable[i] = (varsize)inpVariableROM[i + 4] ; 
    }

    return true ; 
} /* End of function 'GetTestData' */

/*
*  Function :  GetInputValues
*
*  On each pass of the table lookup, a value must be input for 'inputData'.
*  Each time this function is called, the next input value is
*  pulled from the table in RAM.  The table wraps around, so that input data is
*  continuous. A flag is returned TRUE whenever the table wraps around.
*
*/    

n_int
GetInputValues( n_void )
{        
    inputData = inpVariable[tableCount] ; 

    /* If you run out of test data, then start table over */    
    if( ++tableCount < NUM_TESTS )
    {
        return false ; 
    }

    tableCount = 0 ; 
    return true ; 
} /* End of function 'GetInputValues' */


