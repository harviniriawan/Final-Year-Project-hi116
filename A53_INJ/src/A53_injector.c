#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_exception.h"
#include "xil_cache.h"
#include "xscugic.h"
#include "xipipsu.h"
#include "xipipsu_hw.h"
#include "sleep.h"

/* IPI device ID to use for this test */
#define TEST_CHANNEL_ID	XPAR_XIPIPSU_0_DEVICE_ID
/* Test message length in words. Max is 8 words (32 bytes) */
#define TEST_MSG_LEN	4
/* Interrupt Controller device ID */
#define INTC_DEVICE_ID	XPAR_SCUGIC_0_DEVICE_ID
/* Time out parameter while polling for response */
#define TIMEOUT_COUNT 500000
#define BTCM_START 0x20000
#define BTCM_LENGTH 0x10000
/* Sampling period of error injection */
/* Every SAMPLE_PERIOD, decide whether to do injection or not */
#define SAMPLE_PERIOD 1000000
#define UPPER_LIM 100000

#define THREE_BITS_ERROR_TH 0.00001f
#define TWO_BITS_ERROR_TH 0.001f
#define ONE_BITS_ERROR_TH 0.1f

/* Change for every benchmark */
#define BENCH_START 0x2b0
#define BENCH_END 0xfac

XScuGic GicInst;
XIpiPsu IpiInst;

/* Buffers to store Test Data */
u32 MsgBuffer[TEST_MSG_LEN];

void IpiIntrHandler(void *XIpiPsuPtr);
static XStatus SetupInterruptSystem(XScuGic *IntcInstancePtr,XIpiPsu *IpiInstancePtr, u32 IpiIntrId) ;
static XStatus DoIpiTest(XIpiPsu *InstancePtr,u32 corrupted_addr, u32 bits_mask, u32 shift);
float get_probability(u32 th);
u32 get_number_of_ones(u32 num);
u32 bits_to_shift(u32 mask);
int uniform_distribution(int rangeLow, int rangeHigh);

int main()
{
    init_platform();

    Xil_DCacheDisable();

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

	/* Call the test routine */
	xil_printf("Seed is : %s\r\n",__TIME__);
	srand(__TIME__);
	u32 mask;
	u32 tempMask;
	u32 byteCorr;
	u32 shift;
	u32 corrAddr;
	u32 pos;
	float Samp_prob;

	while (1){
		usleep_A53(SAMPLE_PERIOD);
		Samp_prob = get_probability(UPPER_LIM);
		mask = 0;
		/* Pick an address to be corrupted */
		/* Benchmark Specific */
		corrAddr = uniform_distribution(BENCH_START,BENCH_END);
		/* Add the BTCM start offset */
		corrAddr += BTCM_START;
		/* Force Addr to be doubleword aligned */
		corrAddr &= 0xFFFFFFF8;

		if (Samp_prob < THREE_BITS_ERROR_TH){
			/* Pick bit positions to corrupt, from 0 to 3 */
			do {
				tempMask = 1;
				pos = uniform_distribution(0,31);
				tempMask <<= pos;
				mask |= tempMask;
			} while (get_number_of_ones(mask) < 3);

		} else if (Samp_prob < TWO_BITS_ERROR_TH){
			do {
				tempMask = 1;
				pos = uniform_distribution(0,31);
				tempMask <<= pos;
				mask |= tempMask;

			} while (get_number_of_ones(mask) < 2);

		} else if (Samp_prob < ONE_BITS_ERROR_TH){
			tempMask = 1;
			pos = uniform_distribution(0,31);
			tempMask <<= pos;
			mask |= tempMask;
		}

		if (Samp_prob < ONE_BITS_ERROR_TH){
			shift = bits_to_shift(mask);
			Status = DoIpiTest(&IpiInst, corrAddr, mask, shift);
			if (Status != XST_SUCCESS){
				xil_printf("WRONG!\r\n");
				break;
			}
		}
	}

    cleanup_platform();
    return 0;

}

/* From the mask, find out which bytes are not corrupted, and shift to that byte */
u32 bits_to_shift(u32 mask)
{
	u32 bits;
	for(u32 i=0; i<4; i++)
    {

        if((mask & 0xFF) == 0x0){
            bits = i;
            break;
        }

        mask>>=8;
    }

    return bits*8;

}

u32 get_number_of_ones(u32 num)
{
	u32 ones = 0;
	u32 upper;
	upper = 32;

   for(u32 i=0; i<upper; i++)
    {
        /* If LSB is set then increment ones otherwise zeros */
        if(num & 1)
            ones++;

        /* Right shift bits of num to one position */
        num >>= 1;
    }
    return ones;
}

int uniform_distribution(int rangeLow, int rangeHigh)
{
    int range = rangeHigh - rangeLow + 1; //+1 makes it [rangeLow, rangeHigh], inclusive.
    int copies=RAND_MAX/range; // we can fit n-copies of [0...range-1] into RAND_MAX
    // Use rejection sampling to avoid distribution errors
    int limit=range*copies;    
    int myRand=-1;
    while( myRand<0 || myRand>=limit){
        myRand=rand();   
    }
    return myRand/copies+rangeLow;    // note that this involves the high-bits
}

float get_probability(u32 th)
{
	u32 num;
	float prob;
	num = uniform_distribution(0,th);
	prob = ((float)num/th);
	return prob;

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

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Enable the interrupt for the device */
	XScuGic_Enable(IntcInstancePtr, IpiIntrId);

	/* Enable interrupts */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

static XStatus DoIpiTest(XIpiPsu *InstancePtr,u32 corrupted_addr, u32 bits_mask, u32 shift)
{

	u32 Index;
	u32 Status;

	XIpiPsu_Config *DestCfgPtr;
	DestCfgPtr = XIpiPsu_LookupConfig(TEST_CHANNEL_ID);

	MsgBuffer[0] = corrupted_addr;
	MsgBuffer[1] = bits_mask;
	MsgBuffer[2] = shift;
	MsgBuffer[3] = get_number_of_ones(bits_mask);

	/**
	 * Send a Message to TEST_TARGET and WAIT for ACK
	 */

	XIpiPsu_WriteMessage(InstancePtr, XPAR_XIPIPS_TARGET_PSU_CORTEXR5_0_CH0_MASK, MsgBuffer,TEST_MSG_LEN,XIPIPSU_BUF_TYPE_MSG);

	XIpiPsu_TriggerIpi(InstancePtr, XPAR_XIPIPS_TARGET_PSU_CORTEXR5_0_CH0_MASK);

	Status = XIpiPsu_PollForAck(InstancePtr, XPAR_XIPIPS_TARGET_PSU_CORTEXR5_0_CH0_MASK, TIMEOUT_COUNT);

	if (XST_SUCCESS == Status) {

		Status = XST_SUCCESS;

	} else {
		xil_printf("Error: Timed Out polling for response\r\n");
	}

	return Status;
}
