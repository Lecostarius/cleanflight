Overall structure of Cleanflight / Betaflight / iNav
----------------------------------------------------

Top level directory:

|- docs
|           Almost completely empty, contains docs for two boards
|- downloads
|           Downloaded files (e.g. the arm crosscompiler zipfile) go here; initially empty
|- lib
|           external libraries: STM32.. from STMicro, MAVLink, dyad [a communication library], DSP_Lib
|- make
|           sub-Makefiles (included by toplevel Makefile)
|- obj
|           all object files, elf files, and hex files are created here
|- src 
|           this is the source code
|- support
|           ST Microelectronics STM Flashloader stuff
|- tools
|           the arm_none_eabi crosscompiler


Makefile
build_docs.sh
fake_travis_build.sh 
travis.sh 
.travis.yml
Notes.md 
README.md
Vagrantfile


The Make process and the make/ subdirectory
--------------------------------------------
Basic usage: 
make clean ; make TARGET=NAZE

Some explicit main Makefile targets that are useful:
 version 
 targets
 help
 clean 

 Some more Makefile targets that are a bit less useful but still:
 test, test unittest, unbrick, cppcheck, binary, hex, flash, openocd-gdb, all, official

Content of the make/ directory:
make 
  |-local.mk 
  |     developer preferences, this is in .gitignore, the developers playground
  |-linux|windows|macosx].mk
  |     OS dependent setting of environment variable PYTHON
  |-tools.mk
  |     the makefile that defines targets 'arm_sdk_install' and 'openocd_install', 'stm32flash_install',
  |     'dfuutil_install', 'uncrustify_install', 'breakpad_install', plus a few others
  |-targets.mk 
  |     sets only a lot of makefile variables dependent on the board (TARGET) you specify
  |     this contains information about F1/F3/F4/F7 etc.
 
The lib/ directory
------------------
lib/		this contains test/ and main/, where main/ contains the STM32 files for the STM32F30x/STM32F10x.
			There is CMSIS/ with core_cm3.c/h, and all the .c/.h files for the peripherals like e.g.
			stm32f30x_i2c, stm32f30x_tim usw usf. For CMSIS, I did not find out the version; this is originally
			from ARM; the only indication is a Release Note for STM32F10x CMSIS which is V3.5.0 of 11 March 2011.
			In main\STM32F30x_StdPeriph_Driver we find STMs standard peripheral library, in version 1.1.1 from
			April 04, 2014. (As of April 2016, there is the 1.2.2 out.)

The src/ directory

There are two subdirs, main/ and test/. Here is the content of main/:


main/ contains 12 directories and 7 c files (and 6 header files).
The directories are

blackbox			just 2 .c files and 3 header files relating to blackbox
common				atomic, axis, color, colorconversion, encoding, filter, maths, printf, typeconversion, utils
					mainly short macro definitions. maths.c contains applyDeadband().
config				seems to handle configuration in EEPROM?
drivers				dozens and dozens of files, for serial, sonar, sound_beeper, timer, pwm, acc, baro...
					beeper is easy to understand and small
flight				altitudehold, failsafe, gps_conversion, gtune, imu, lowpass, mixer, navigation, pid
io					a lot - beeper, display, serial_cli, serial_1wire, statusindicator, flashfs & many more
rx					ibus, msp, pwm, rx, sbus, spektrum, sumd, sumh, xbus
sensors				acceleration, barometer, battery, boardalignment, compass, gyro, initialisation, sonar. sensors.h
startup				3 assembler files: e.g. 'startup_stm32f30x_md_gcc.S'
target				one subdir per target containing just 3 files each plus 7 global files, e.g. 'stm32_flash_f303_128k.ld'
telemetry			telemetry.c/.h and 4 files: frsky/hott/ltm/smartport.c/h
vcp					10 or so seemingly hardware-close .c/.h like usb_pwr.c but also hw_config and stm32_it, all from
					ST microelectronics. Obviously not really part of cleanflight.

and the 7 source files are 
build_config.c/h		almost empty; checks compile-time whether #of motors and sensors exceeds MAX_SUPPORTED
debug.c					almost empty; mainly declares an int16_t array debug[]
main.c					this defines mainly init() and main(). init() calls heaps of peripheral init functions
						depending on preprocessor #defines like USE_SERVOS or SONAR
mw.c					this contains annexCode() and the flight-relevant task handler functions, e.g. taskUpdateRxMain(),
						taskCalculateAttitude(), but also taskLedStrip() etc.
scheduler_tasks.c		this just defines the array cfTasks[TASK_COUNT] with all tasks that need to be done,
						their priority, name, period etc.
scheduler.c				the scheduler itself, contains scheduler()
version.c/h				defines: targetName, shortGitRevision, buildDate, buildTime (all const char *)

platform.h				just #includes the proper manufacturers .h files for targets stm32f1 and stm32f3
						(depends on preprocessor #define STM32F10X or #define STM32F303xC)
						
Adding a Sonar
--------------
I have two Sonars, one of which is I2C, the other has a sort of PWM output plus an analog output. The PWM
has a pulse width of 58 microseconds per cm distance measured (at 10 meters, this is 58 milliseconds...)
The problem with the I2C one is that it is really slow. Without clock stretching, it allows max 50 kHz,
with clock stretching, it can live with 400 kHz (but not with Cleanflights 1 MHz, called i2COverclock).
In main.c there is a call that sets either 400 kHz or overclock, 1 MHz:
   i2cSetOverclock(masterConfig.i2c_highspeed);
The function is declared in main/drivers/bus_i2c_stm32f30x.c; this is where 100 kHz should be set.
It seems that setting clock stretching is configurable in STM32, so it is a bit unclear whether this has
been activated or not.

Sonar is handled similar to other sensors in the code, however since only one type is supported so far, it
is more hardcoded than other sensor types. There is no configurable sonar object which stores the handler
functions that can be exchanged depending on the hardware that is there. See barometer as a counterexample.

In drivers/, there is the actual functionality, in sonar_hcsr04.c/h, including the interrupt handler for the
PWM time management, read-out, and init.

In sensors/ there is the middleware code that provides the interface to the low level driver code, it handles
the hardware dependencies (here: only the boards, the sensor is always the hcsr04). This hardware setup code
is directly called from main.c. 
Additionally sensors/sonar.c provides the interface functions:
void sonarUpdate(void); // regularly called, to start the triggering
int32_t sonarRead(void); // provides the actual reading (including median filter), if >380 cm, returns -1
int32_t sonarCalculateAltitude(int32_t sonarDistance, float cosTiltAngle);
int32_t sonarGetLatestAltitude(void); // just returns the value the last time sonarCalculateAttitude() returned

TASK_SONAR from the scheduler is configured to be called every 50 ms and calls taskUpdateSonar() which is in mw.c
as all tasks, and just calls sonarUpdate().

The sonar is read not from the task managers SONAR task, but from TASK_ALTITUDE with 40 times a second (25 ms).
from altitudehold.c::calculateEstimatedAltitude(). Here, the usage is implemented: if the 
altitude is below sonarCfAltCm, only sonar is used and baro is ignored; if it is between sonarCfAltCm and 
sonarMaxAltWithTiltCm, it will be interpolated with baro, above that, baro is used.

main()
------
main() really just configures the scheduler - depending on constants like DISPLAY, which, if #defined, will do
a setTaskEnables(TASK_DISPLAY, feature(FEATURE_DISPLAY));. Then, the main loop that is running all the time is:
    while (1) {
        scheduler();
        processLoopback();
    }
processLoopback() is a special handler for a #define SOFTSERIAL_LOOPBACK which is usually not #defined. If we have
no SOFTSERIAL_LOOPBACK defined, processLoopback does nothing: #define processLoopback()
So, really, we call scheduler() as fast as we can all the time.

Scheduler
---------
There are 17 tasks predefined in the code, in scheduler_tasks.c. 
All 17 task functions are of the type 'void taskUpdateDisplay(void)'.
The definitions of all of them - except taskSystem() - are in 'mw.c', and frequently, they just call another function like
in this case 'updateDisplay()' which is in 'display.c'.
Each task has a priority (enum cfTaskPriority_e, from 0 to 255 (highest)), and a fixed ID (TASK_SYSTEM is 0, 
TASK_GYROPID is 1, and so on), and the scheduler maintains a struct cfTask_t with fields char *taskName,
boolean isEnabled, a desiredPeriod, the priority, the function pointer taskFunc, a checkFunc() function pointer,
and statistics over its execution time (average, total, max). There is also things like lastExecutedAt, lastSignaledAt,
taskAgeCycles, dynamicPriority.
cfTasks[taskId].taskAgeCycles - usually 0. Gives how often the task is already overdue. If 2, it should have been executed twice
                                since it was last really executed (according to its desiredPeriod). 
								
cfTasks[taskId].dynamicPriority	gives the actual current priority of a task. Usually, this is 0, but if the system
								is under load and the task does not get executed often enough, this is incremented by
								staticPriority*taskAgeCycles: the longer it has waited, the more, and in steps of its staticPriority.
								
taskLatestDeltaTime				stores the actual time interval between the last two invocations in microseconds.
lastExecutedAt					stores when the task was last executed.
averageExecutionTime			sliding window over last 32 invocations of this task, measures how long it takes to complete
totalExecutionTime				adds and adds and adds taskExecutionTime
maxExecutionTime				longest time it ever took to run this task



Generally, a task has a wait time, given in microseconds (100 is the minimum, no task can be executed with 10kHz or more).
The function of checkFunc is to indicate whether the task is event driven: an event driven task has a checkFunc. It can
additionally have a desiredPeriod. If it has a checkFunc, the scheduler will determine by calling the checkFunc with the
time passed since last invocation whether the corresponding tasks needs to be run - the checkFunc should check the time
when it was last called, check its internal state, and then decide whether it needs to be run again or not by returning TRUE/FALSE.

From the 17 default tasks, only TASK_RX has one (taskUpdateRxCheck), all the others are time driven.

void taskSystem(void) - only updates some internal static variables of the scheduler itself (realtimeGuardInterval...)

Helpers and Debug Tools:
void getTaskInfo(taskId, cfTaskInfo_t *) - just a getter for the private cfTasks[taskId] array to read the task information
void rescheduleTask(taskId, newPeriodMicros) - a setter for cfTasks[taskId], sets the desiredPeriod anew (and prevents <100 us)
void setTaskEnabled(taskId, bool newState) - a setter for cfTasks[taskId].isEnabled
uint32_t getTaskDeltaTime(taskId) - this is a getter for cfTasks[taskId].taskLatestDeltaTime

ID	Task Name		#ifdef			period [ms]	priority		actual child	in
0	TASK_SYSTEM		-				100	TASK_PRIORITY_HIGH		taskSystem	scheduler.c
1	TASK_GYROPID	-				1	TASK_PRIORITY_REALTIME	taskMainPidLoopChecker()	mw.c
2	TASK_ACCEL		-				10	TASK_PRIORITY_MEDIUM	imuUpdateAccelerometer	flight/imu.c
3	TASK_SERIAL		-				10	TASK_PRIORITY_LOW		handleSerial	io/serial.c
4	TASK_BEEPER		BEEPER			10	TASK_PRIORITY_MEDIUM	beeperUpdate	io/beeper.c
5	TASK_BATTERY	-				20	TASK_PRIORITY_MEDIUM	updateBattery+updateCurrentMeter	sensors/battery.c
6	TASK_RX			-				20	TASK_PRIORITY_HIGH		processRx	mw.c
7	TASK_GPS		GPS				10	TASK_PRIORITY_MEDIUM	gpsThread+updateGpsIndicator	io/gps.c
8	TASK_COMPASS	MAG				10	TASK_PRIORITY_MEDIUM	updateCompass	sensors/compass.c
9	TASK_BARO		BARO			50	TASK_PRIORITY_MEDIUM	baroUpdate	sensors/barometer.c
10	TASK_SONAR		SONAR			50	TASK_PRIORITY_MEDIUM	sonarUpdate	sensors/sonar.c
11	TASK_ALTITUDE	BARO||SONAR		25	TASK_PRIORITY_MEDIUM	calculateEstimatedAltitude	flight/altitudehold.c
12	TASK_DISPLAY	DISPLAY			10	TASK_PRIORITY_LOW		updateDisplay	io/display.c
13	TASK_TELEMETRY	TELEMETRY		4	TASK_PRIORITY_IDLE		telemetryProcess+telemetryCheckState	telemetry/telemetry.c
14	TASK_LEDSTRIP	LED_STRIP		10	TASK_PRIORITY_IDLE		updateLedStrip	io/ledstrip.c
15	TASK_TRANSPONDER	TRANSPONDER	4	TASK_PRIORITY_LOW		updateTransponder	io/transponder_ir.c



Drivers
-------
system.c	enables interrupt handlers (can be registered here), defines micros(), millis(), delayMicroseconds(), delay(millisecs),
			and has systemInit()
timer.c		USABLE_TIMER_CHANNEL_COUNT = 14
			
PWM-in/out
----------
es gibt in drivers/ ein pwm_mapping.c, das hat auch eine Hardwarespezifische Sektion. 
pwmInit() in pwm_mapping.c initialisiert, je nach Modus in der drv_pwm_config_t: PWM, PPM, jeweils für Copter oder Airplane.
Die Konfiguration ist in einer uint16_t[] und macht ein mapping von PWM1..PWM16 auf entweder MAP_TO_PWM_INPUT, oder
MAP_TO_MOTOR_OUTPUT (es gibt auch noch MAP_TO_SERVO_OUTPUT und MAP_TO_PPM_INPUT). Das Mapping bildet vom "timerIndex"
ab auf den Typ: "timerIndex" sind die niedrigen 8 Bits 0x00FF und der Typ die oberen 8 Bit. "timerIndex" geht von PWM1
bis PWM16 (ist eine enum, 1..16), und der Typ geht von 1 bis 4 (1=MAP_TO_PPM_INPUT, 2=MAP_TO_PWM_INPUT, 3=MAP_TO_MOTOR_OUTPUT,
4=MAP_TO_SERVO_OUTPUT). In pwmInit() wird nun der timerIndex als Index in das Array "timerHardware_t timerHardware[timerIndex]" 
benutzt. Dieses Array enthält alle benutzbaren Channels aller Timer des STM303, es sollte also eher timerChannel[] heissen;
und das enum PWM1 ist also der erste Eintrag dieses Arrays, PWM2 der zweite - damit entspricht PWM1 dem Timer 2 Channel1 und
PWM2 dem Timer 2 Channel 2.
Danach folgt eine lange Sequenz von Abfragen, ob nun dieses Mapping wirklich gemacht werden soll (oder ob der Timer,
oder der Port, oder was auch immer schon anderweitig belegt ist, z.B. für VBAT sensing). Beim SPRACINGF3 wird PWM15+16 auf Servo
umgemappt (aber nur wenn timerHardware[]->tim == TIM15 ist, was es ist). Die Bezeichnung "PWM1" bis "PWM16" ist total verwirrend und blödsinnig,
denn es handelt sich hierbei nur um einen Index in timerHardware[], was alle verwendbaren Timer im System subsummiert.
Schlussendlich wird dann abhängig vom Typ (MAP_TO_MOTOR_OUTPUT usw) eine von 4 kleinen Programmteilen aufgerufen, hier der für
PWM_INPUT und der für MOTOR_OUTPUT:
       } else if (type == MAP_TO_PWM_INPUT) {
	        // die eigentliche Konfiguration: pwmInConfig() ist in pwm_rx.c. Das channelIndex wird als Index in ein
			// statisches Array dort in pwm_rx.c verwendet, 
            pwmInConfig(timerHardwarePtr, channelIndex);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flaaaaags = PWM_PF_PWM;
            pwmIOConfiguration.pwmInputCount++;
            channelIndex++; <-- ausschliesslich für PWM input genutzt! Init auf null, nur hier inkrementiert!
		if type==MAP_TO_MOTOR_OUTPUT
		    // die eigentliche Konfiguration: in pwm_output.c, dort wird letztendlich pwmOutConfig() aufgerufen
            pwmBrushlessMotorConfig(timerHardwarePtr, pwmIOConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_PWM ;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.motorCount;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware = timerHardwarePtr;
            pwmIOConfiguration.motorCount++;
Man sieht dass immer eine Strukt pwmIOConfiguration gefüllt wird: die ist der Rückgabewert von pwmInit() und enthält nachher
die komplette Konfiguration von PWM/PPM/SERVO/MOTOR in-out. Diese Strukt ist aber rein informativ - die eigentliche Hardware-
Initialisierung ist bereits fertig gemacht (z.B. in pwm_rx.c oder pwm_output.c)

in pwm_rx.c ist die pwmInConfig(timerHardwarePtr, channelIdx); die macht:
 - pwmGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, timerHardwarePtr->gpioInputMode);
 - pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);
Beide sind in pwm_rx.c deklariert; die pwmICConfig ist nur noch ein kleiner Wrapper für STMs Funktion TIM_ICInit().
pwmGPIOConfig() ist ein wrapper indirekt für STMs GPIO_Init(). Indirekt weil erst gpio_stm32f303.c::gpioInit() aufgerufen;
der Sinn ist vor allem der dass nicht alle Pins konfiguriert werden müssen sondern es auch möglich ist nur eines oder wenige
zu konfigurieren. Konfiguriert wird hier pull up/pull down, Speed des Ports, Ausgang/Eingang, Special Function
In pwmICConfig() in pwm_rx.c in Drivers/ wird die STM-vorgegebene Struktur TIM_ICInitTypeDef TIM_ICInitStructure gefüllt - 
da kommen Sachen rein wie der Input Filter, Prescaler, ... - und dann wird die STM peripheral library Funktion TIM_ICConfig() 
aufgerufen.
Danach wird noch aufgerufen timerConfigure(), timerChCCHandlerInit(), timerChOvrHandlerInit(), timerChConfigCallbacks().
timerConfigure() in timer.c: ruft TIM_TimeBaseInit() auf und setzt den Prescaler, countmode=UP, ClockDivision. Ruft auch timerNVICConfigure() auf
							welches die Interrupts konfiguriert. Macht ENABLE auf den Timer. 
timerChCCHandlerInit() in timer.c: ein Witz, setzt lediglich self->edgeCB->fn = pwmEdgeCallback
timerChOvrHandlerInit() in timer.c: ein Witz, setzt lediglich self->edgeCB->fn=pwmOverflowCallback; self->edgeCb->next=NULL;
timerChConfigCallbacks() in timer.c: im Prinzip setzt es nur TIM_ITConfig() und aktiviert IRQs für diesen Channel. Es überprüft aber
									noch ob alles OK ist (z.B. functionpointer=0), es setzt die timerConfig[idx].edgeCallback[channelIndex]
									und konfiguriert auch UpdateOverflow; das ist zusätzlich enorm kompliziert und ich habe nicht tiefer reingeschaut.
									Was aber die UpdateOverflow macht, ist, ins globale (statische) captures[] array die Konstante PPM_RCVR_TIMEOUT
									reinzuschreiben (egal ob PPM benutzt wird oder PWM). 

pwmEdgeCallback() rekonfiguriert den Timer immer auf polarity falling/rising und bei falling speichert er die Zeit im globalen
static Array uint16_t captures[pwmInputPort->channel], definiert in pwm_rx.c.
Dieses Array wird über die Funktion pwmRead(channel) ausgelesen.

timerNVICConfigure() konfiguriert den Interrupt für den Timer.

Die Timer-Hardware ist zusammengefasst in drivers/timer.c, dort wird abhängig vom Board :

#if defined(SPRACINGF3)
//	Timer#	 GPIOport	Pin	TimerChannel	Interrupt			in/out	IOpin config	GPIOpin		???
//                      Pin                    und                      IOpin               sind redundant
// TIM_Type	 GPIO_Type	pin	channel			irq					outputEnable	gpioInputMode	gpioPinSource	alternateFunction
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2,  GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_1}, // RC_CH1 - PA0  - *TIM2_CH1
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_1}, // RC_CH2 - PA1  - *TIM2_CH2, TIM15_CH1N
    // Production boards swapped RC_CH3/4 swapped to make it easier to use SerialRX using supplied cables - compared to first prototype.
    { TIM2,  GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_1}, // RC_CH3 - PB11 - *TIM2_CH4, USART3_RX (AF7)
    { TIM2,  GPIOB, Pin_10, TIM_Channel_3, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource10, GPIO_AF_1}, // RC_CH4 - PB10 - *TIM2_CH3, USART3_TX (AF7)
    { TIM3,  GPIOB, Pin_4,  TIM_Channel_1, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource4,  GPIO_AF_2}, // RC_CH5 - PB4  - *TIM3_CH1
    { TIM3,  GPIOB, Pin_5,  TIM_Channel_2, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource5,  GPIO_AF_2}, // RC_CH6 - PB5  - *TIM3_CH2
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_2}, // RC_CH7 - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_2}, // RC_CH8 - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N

    { TIM16, GPIOA, Pin_6,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, Mode_AF_PP, GPIO_PinSource6,  GPIO_AF_1},  // PWM1 - PA6  - TIM3_CH1, TIM8_BKIN, TIM1_BKIN, *TIM16_CH1
    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource7,  GPIO_AF_1},  // PWM2 - PA7  - TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    { TIM4,  GPIOA, Pin_11, TIM_Channel_1, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_10}, // PWM3 - PA11
    { TIM4,  GPIOA, Pin_12, TIM_Channel_2, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource12, GPIO_AF_10}, // PWM4 - PA12
    { TIM4,  GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_2},  // PWM5 - PB8
    { TIM4,  GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource9,  GPIO_AF_2},  // PWM6 - PB9
    { TIM15, GPIOA, Pin_2,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource2,  GPIO_AF_9},  // PWM7 - PA2
    { TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource3,  GPIO_AF_9},  // PWM8 - PA3

    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_6},  // GPIO_TIMER / LED_STRIP
};

#define USED_TIMERS  (TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(15) | TIM_N(16) |TIM_N(17))

#define TIMER_APB1_PERIPHERALS (RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4)
#define TIMER_APB2_PERIPHERALS (RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM15 | RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM17)
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB)

#endif
Weiterhin werden in timer.c die Interrupts für die Timer (die eigentlichen Handler) konfiguriert. Es gibt die zentralen beschreibenden
structs timerConfig_t, timerChannelInfo_t, timerInfo_t. 

TASK_RX
--------
Die taskFunc ist taskUpdateRxMain() in mw.c, die ruft nur processRx() ebenfalls in mw.c auf. Diese ruft 
rx.c::calculateRxChannelsAndUpdateFailsafe() auf. Die ruft rx.c::readRxChannelsApplyRanges() auf welche ein Remapping der Channels macht
und dann rcReadRawFunc() aufruft - das wurde aber bei der Initialisierung in pwm.c::rxPwmInit() auf pwm.c::pwmReadRawRC() gelegt und
holt nur den Inhalt des Arrays captures[channel] was die rohen Captures sind. FALLS der channel einer der ersten 4 ist (also kein AUX),
wird noch ein applyRxChannelRangeConfiguration() aufgerufen, ansonsten wird der Wert in rcRaw[channel] abgelegt.

Eigentlich sollte also das Signal jedes AUX channels 1:1 übernommen werden; es gibt keinen Grund, warum es immer zwischen 1000 und 2000
liegen sollte.

Allerdings wird in rx.c::calculateRxChannelsAndUpdateFailsafe() nach dem Aufruf von rx.c::readRxChannelsApplyRanges() nachher noch 
rx.c::detectAndApplySignalLossBehaviour() aufgerufen. Das geht über alle Channels und testet mit isPulseValid(sample) ob das Sample
okay ist. Wenn nicht, wird es für MAX_INVALID_PULS_TIME (300 ms) durch den alten Wert ersetzt; danach aber wird es durch getRxFailValue()
ersetzt. rx.c::isPulseValid() testet ob die pulseDuration zwischen rxConfig->rx_min_usec und rxConfig->rx_max_usec liegt.

Schlussendlich kommt das Ergebnis in rcData[channel].

rxConfig->rx_min_usec wird durch rx.c::isP

Blackbox
--------
Geloggt wird rcCommand[0..3] wobei das in mw.c ab Zeile 200 oder so aus rcData[] berechnet wird und es für YAW,PITCH,ROLL von -500..500 
und für THROTTLE von 0..1000 geht.
Es gibt laut Blackboxtools folgende Arten von Frames:
I frames - 49.4 byte avg, 2000
P frames - 23 bytes avg, 30000
H frames - 9.3 bytes avg, 19
G frames - 12.3 bytes avg, 316
E frames, 9 bytes avg, 2
S frames, 4 bytes avg, 16
Also im Wesentlichen P frames und 1/16 I frames, 15/16 P frames. die G frames sind fast sicher GPS.

config
------
Es gibt ein directory config/ (parallel zu drivers/ usw). Da finden sich die in einigen Headerfiles die folgenden
config_types:
master_t masterConfig - das enthält alle configs als Elemente. Auch profile_t und controlRateConfig_t (von dem es 3 gibt), da gibt 
						es die Felder currentProfile und currentControlRateProfile im masterConfig.
profile_t - in diesem sind einige configs zusammengefasst, wohl solche, zwischen denen man hin- und herschalten können möchte.
            Es gibt ein array von drei solchen Profilen in profile[3]. 
controlRateConfig_t - auch hier gibt es ein array von 3 solchen Profilen; da sind rcExpo, rcRate und so weiter drin.

Die masterConfig hat eine Funktion "resetConf()" die alles auf default zurücksetzt.

Es gibt eine enum features_e mit 22 Bits die gesetzt sein können; z.B. ob VBAT monitoring an ist, ob es ein SONAR gib,t
ob PARALLEL_PWM verwendet wird, ob es ein DISPLAY gibt usw. Das enum ist in config.h. Die eigentlichen Daten sind eine 
uint32_t masterConfig.enabledFeatures - die werden durch die Funktion latchActiveFeatures() in config.c in eine static uint32_t
activeFeaturesLatch reinkopiert und von dort werden sie von allen möglichen Funktionen gelesen - dazu gibt es die Funktion 
feature(mask) in config.c wo man z.B. sagt if feature(VBAT) { ... }.
			
Dann gibt es noch eine config für so ziemlich jede Funktion - z.B. gpsProfile_t, pidProfile, barometerConfig_t. Und in config.c
findet sich eine resetXxxProfile() Funktion (z.B. "resetGpsProfile()" Funktion). 

config.c managt auch das Schreiben und Lesen der masterConfig ins EEPROM.

In rx.h ist die rxConfig_t. In der findet sich das Mapping der Channels auf die immergleiche interne Order, sbus_inversion,
rssi, rcSmoothing, midrc, mincheck, maxcheck, rx_min_usec, rx_max_usec, und channelRanges[0..3] für die non-AUX-channels.