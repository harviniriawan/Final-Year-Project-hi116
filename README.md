# Final-Year-Project-hi116

## Hardware setup

1. Connect the power and reset pin from the ultra96 board to the Arduino digital pins
2. In the project, digital pin 12 is connected to the power pin (pin number 4 of Ultra96)
3. Digital pin 13 is connected to the reset pin (pin number 6 of Ultra96)
4. Connect both Ultra96 board and Arduino Uno to a host computer.

## Script setup
There are several arguments that needed to be adjusted in the `controlscript.py` file. Full details are in the source code.

## Script flow
1. The Python script invokes XSCT which set up an SDK workspace, imports all the folders containing source codes,
and builds the entire benchmarks in the project.
2. The  `main()` function of the script is executed, and experiment using a hardware configuration begins.

## Changing hardware configurations
1. Go to each benchmark folder, and decide on whether to use the `#define NO_ECC` directive. Using the directive means that for that kernel, the experiment is run without TCM ECC protection.
2. Go to `A53_INJ`, this is where the TCM injection parameters can be adjusted :
```
/* Time out parameter while polling for response */
#define TIMEOUT_COUNT 1000000
#define BTCM_START 0x20000
#define BTCM_LENGTH 0x10000
/* Sampling period of error injection */
/* Every SAMPLE_PERIOD, decide whether to do injection or not */
/*Default: Sample per 1 second*/
#define SAMPLE_PERIOD 100000
#define UPPER_LIM 1000000

/*Default number from FIT Dony's report*/
#define THREE_BITS_ERROR_TH 0.00011f
#define TWO_BITS_ERROR_TH 0.00125f
#define ONE_BITS_ERROR_TH 0.0125f
```

## Running Experiment
1. Go to the folder `results`, invoke the python script `python controlscript.py`, test runs automatically.
