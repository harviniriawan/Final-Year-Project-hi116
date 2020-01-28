/*==============================================================================
 *$RCSfile: algotst.c,v $
 *
 *   DESC : Angle-to-Time Conversion - a2time00
 *
 * AUTHOR : dt
 *
 *  EEMBC : Automotive Subcommittee 
 *
 *    CVS : $Revision: 1.3 $
 *          $Date: 2002/04/19 18:31:37 $
 *          $Author: rick $
 *          $Source: d:/cvs/eembc2/automotive/a2time01/algotst.c,v $
 *          
 * NOTE   :
 *
 *------------------------------------------------------------------------------
 *
 * HISTORY :
 *
 * $Log: algotst.c,v $
 * Revision 1.3  2002/04/19 18:31:37  rick
 * Bug #146: global tablecount uninitialized
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
 * The following test data represents values read from a realtime clock
 * which is captured( in hardware )on each rising edge of the ring-gear
 * tooth.  The counter is expected to roll over at maximum count, and
 * it is presumed to be an "up" counter.
 *
 */    

const varsize inpAngleROM[] = 
{        
    123, 

    456, 

    796, 

    1143, 

    1497, 

    1858, 

    2226, 

    2601, 

    3358, 

    3743, 

    4136, 

    4535, 

    4942, 

    5355, 

    5776, 

    6203, 

    6638, 

    7079, 

    7528, 

    7983, 

    8446, 

    8915, 

    9392, 

    9875, 

    10366, 

    10863, 

    11368, 

    11879, 

    12398, 

    12923, 

    13456, 

    13995, 

    14542, 

    15095, 

    15656, 

    16223, 

    16798, 

    17379, 

    17968, 

    18563, 

    19166, 

    19775, 

    20392, 

    21015, 

    21646, 

    22283, 

    22928, 

    23579, 

    24238, 

    24903, 

    25576, 

    26255, 

    26942, 

    27635, 

    28336, 

    29043, 

    29758, 

    30479, 

    31208, 

    31943, 

    32686, 

    668, 

    1425, 

    2189, 

    2960, 

    3738, 

    4523, 

    5315, 

    6906, 

    7708, 

    8518, 

    9334, 

    10158, 

    10988, 

    11826, 

    12670, 

    13522, 

    14380, 

    15246, 

    16118, 

    16998, 

    17884, 

    18778, 

    19678, 

    20586, 

    21500, 

    22422, 

    23350, 

    24286, 

    25228, 

    26178, 

    27134, 

    28098, 

    29068, 

    30046, 

    31030, 

    32022, 

    250, 

    1253, 

    2263, 

    3280, 

    4304, 

    5335, 

    6373, 

    7418, 

    8470, 

    9529, 

    10595, 

    11668, 

    12748, 

    13835, 

    14929, 

    16030, 

    17138, 

    18253, 

    19375, 

    20504, 

    21640, 

    22783, 

    23933, 

    25090, 

    26254, 

    27425, 

    28603, 

    29788, 

    30980, 

    32179, 

    620, 

    3045, 

    4264, 

    5491, 

    6724, 

    7965, 

    9212, 

    10467, 

    11728, 

    12997, 

    14272, 

    15555, 

    16844, 

    18141, 

    19444, 

    20755, 

    22072, 

    23397, 

    24728, 

    26067, 

    27412, 

    28765, 

    30124, 

    31491, 

    100, 

    1484, 

    2875, 

    4273, 

    5678, 

    7090, 

    8509, 

    9935, 

    11368, 

    12808, 

    14255, 

    15709, 

    17170, 

    18638, 

    20113, 

    21595, 

    23084, 

    24580, 

    26083, 

    27593, 

    29110, 

    30634, 

    32165, 

    930, 

    2470, 

    4017, 

    5571, 

    7132, 

    8700, 

    10275, 

    11857, 

    13446, 

    15042, 

    16645, 

    18255, 

    19872, 

    21496, 

    24751, 

    26385, 

    28027, 

    29675, 

    31331, 

    230, 

    1904, 

    3585, 

    5273, 

    6968, 

    8670, 

    10379, 

    12095, 

    13818, 

    15548, 

    17285, 

    19029, 

    20780, 

    22538, 

    24303, 

    26075, 

    27854, 

    29640, 

    31433, 

    460, 

    2262, 

    4071, 

    5887, 

    7710, 

    9540, 

    11377, 

    13221, 

    15072, 

    16930, 

    18795, 

    20667, 

    22546, 

    24432, 

    26325, 

    28225, 

    30132, 

    32046, 

    1200, 

    3129, 

    5065, 

    7008, 

    8958, 

    10915, 

    12879, 

    14850, 

    16828, 

    18813, 

    20805, 

    22804, 

    24810, 

    26823, 

    28843, 

    30870, 

    130, 

    2165, 

    6242, 

    8287, 

    10340, 

    12399, 

    14466, 

    16539, 

    18620, 

    20707, 

    22802, 

    24903, 

    27012, 

    29127, 

    31250, 

    610, 

    2745, 

    4887, 

    7036, 

    9192, 

    11355, 

    13525, 

    15702, 

    17886, 

    20077, 

    22275, 

    24480, 

    26692, 

    28911, 

    31137, 

    600, 

    2838, 

    5083, 

    7335, 

    9594, 

    11860, 

    14133, 

    16413, 

    18700, 

    20994, 

    23295, 

    25603, 

    27918, 

    30240, 

    32569, 

    2140, 

    4486, 

    6812, 

    9118, 

    11404, 

    13670, 

    15916, 

    18142, 

    20348, 

    22534, 

    24700, 

    26846, 

    28972, 

    31078, 

    430, 

    2530, 

    4610, 

    8777, 

    10867, 

    12938, 

    14988, 

    17019, 

    19029, 

    21020, 

    22990, 

    24941, 

    26871, 

    28782, 

    30672, 

    32543, 

    1650, 

    3505, 

    5340, 

    7155, 

    8950, 

    10725, 

    12480, 

    14215, 

    15930, 

    17625, 

    19300, 

    20955, 

    22590, 

    24205, 

    25800, 

    27375, 

    28930, 

    30465, 

    31980, 

    720, 

    2208, 

    3676, 

    5124, 

    6552, 

    7960, 

    9371, 

    10785, 

    12202, 

    13622, 

    15045, 

    16471, 

    17900, 

    19332, 

    20767, 

    22205, 

    23646, 

    25090, 

    26537, 

    27987, 

    29440, 

    30896, 

    32355, 

    1050, 

    2516, 

    3985, 

    5457, 

    6932, 

    9889, 

    11374, 

    12863, 

    14354, 

    15849, 

    17346, 

    18847, 

    20350, 

    21857, 

    23366, 

    24879, 

    26394, 

    27913, 

    29434, 

    30959, 

    32486, 

    1250, 

    2784, 

    4322, 

    5862, 

    7406, 

    8952, 

    10502, 

    12054, 

    13610, 

    15168, 

    16730, 

    18294, 

    19862, 

    21432, 

    23006, 

    24582, 

    26162, 

    27744, 

    29330, 

    30918, 

    32510, 

    1340, 

    2941, 

    4545, 

    6152, 

    7762, 

    9375, 

    10991, 

    12610, 

    14232, 

    15857, 

    17485, 

    19116, 

    20750, 

    22387, 

    24027, 

    25670, 

    27316, 

    28965, 

    30617, 

    32272, 

    1160, 

    2819, 

    4481, 

    7812, 

    9484, 

    11160, 

    12838, 

    14520, 

    16204, 

    17892, 

    19582, 

    21276, 

    22972, 

    24672, 

    26374, 

    28080, 

    29788, 

    31500, 

    450, 

    2171, 

    3895, 

    5622, 

    7352, 

    9085, 

    10821, 

    12560, 

    14302, 

    16047, 

    17795, 

    19546, 

    21300, 

    23057, 

    24817, 

    26580, 

    28346, 

    30115, 

    31887, 

    900, 

    2684, 

    4471, 

    6261, 

    8054, 

    9850, 

    11649, 

    13451, 

    15256, 

    17064, 

    18875, 

    20689, 

    22503, 

    24317, 

    26131, 

    27945, 

    29759, 

    31573, 

    620, 

    2435, 

    4250, 

    6065, 

    7880, 

    9695, 

    11510, 

    13325, 

    16962, 

    18787, 

    20613, 

    22438, 

    24264, 

    26089, 

    27915, 

    29740, 

    31566, 

    620, 

    2442, 

    4264

} ; /* End of test values :  inpAngleROM[] */

/*******************************************************************************
    Functions                                                                   
*******************************************************************************/

/*
*  Function :  GetTestData
*
*    Builds the table of input data which represents 'angle' input 
*    from the mag input on the ring gear.  This is the "real world" data stream which
*    drives the algorithm.  This data is installed in RAM prior to execution of the
*    algorithm.
*
*/    
n_int 
GetTestData( n_void )
{        
    n_int i ; 

    tonewheelTeeth = NUM_TEETH ; 
    inpAngleCount = (varsize *)th_malloc( ( NUM_TESTS + 1 ) * sizeof(varsize) ) ; 

    if( inpAngleCount == NULL )
    {
        return false ; 
    }

    /* Copy the test values from ROM to RAM  */    
    for( i = 0 ; i < NUM_TESTS ; i++ )
    {
        inpAngleCount[i] = inpAngleROM[i] ; 
    }

    return true ; 
} /* End of function 'GetTestData' */

/*
*  Function :  GetInputValues
*
*    On each pass of the table lookup, a value must be input for 'angleCount.
*    Each time this function is called, the next input value is
*    pulled from the table in RAM.  The table wraps around, so that input data is
*    continuous.     A flag is returned TRUE whenever the table wraps around.
*
*/    

n_int 
GetInputValues( n_void )
{        
    angleCounter = inpAngleCount[tableCount] ; 

    if( ++tableCount < NUM_TESTS )
    {
        return false ; 
    }

    tableCount = 0 ; 
    return true ; 
} /* End of function 'GetInputValues' */