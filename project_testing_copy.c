/* ***************************************************************************
 * Paulo Pedreiras Sept 2024
 * pbrp@ua.pt
 * 
 * Example code for acquiring and processing sound using SDL
 * - There are several possibilities, set by #define directives
 * 		- Record sound and play it back
 * 		- Generate a sine wave (configurable frequency and duration)
 * 		- Generate a sine wave (configurable frequency and duration)
 * 		- Add an echo (works fine for syntetich signals)
 * 		- Computing the FFT of a signal
 * 		- Debug functions (print samples, max/min, ...) * 
 * 
 * 	- All functiosn are very crude, with almost no validations. Use with care.
 * 		Bugs are almost guranteed to exist ... please report them to pbrp@ua.pt
 * 
 * Record/playback sound code adapted from:
 * 	https://gist.github.com/cpicanco/12147c60f0b62611edb1377922f443cd
 * 
 * Main SDL 2 doc.
* 	https://wiki.libsdl.org/SDL2/FrontPage
 * *************************************************************************/
#include <SDL.h>
#include <bits/pthreadtypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <complex.h>
#include <sched.h> //sched_setscheduler
#include <pthread.h>
#include <unistd.h>	// usleep
#include "fft/fft.h"


#define MONO 1 					/* Sample and play in mono (1 channel) */
#define SAMP_FREQ 44100			/* Sampling frequency used by audio device */
#define FORMAT AUDIO_U16		/* Format of each sample (signed, unsigned, 8,16 bits, int/float, ...) */
#define ABUFSIZE_SAMPLES 4096	/* Audio buffer size in sample FRAMES (total samples divided by channel count) */

#define NS_IN_SEC 1000000000L
#define PERIOD_NS (100*1000*1000) 	// Period (ns component)
#define PERIOD_S (5)				// Period (seconds component)
#define THREAD_INIT_OFFSET 1000000	// Initial offset (i.e. delay) of rt thread
#define MOTOR_FREQS_SIZE 100
#define VARIANCE 5

//Periodicies:
#define SOUND_GEN_PERIOD_NS (200 * 1000 * 1000)  // 200ms in nanoseconds
#define SOUND_GEN_PERIOD_S (3)                   // 0 seconds
#define MEASURING_SPEED_PERIOD_NS (800 * 1000 * 1000)  // 1s in nanoseconds
#define MEASURING_SPEED_PERIOD_S (3)                   // 0 seconds
#define BEARING_ISSUES_PERIOD_NS (800 * 1000 * 1000)  // 1s in nanoseconds
#define BEARING_ISSUES_PERIOD_S (3)                   // 0 seconds
#define PLAYBACK_PERIOD_NS (900 * 1000 * 1000)  // 1s in nanoseconds
#define PLAYBACK_PERIOD_S (4)                   // 0 seconds
#define DB_PRINT_PERIOD_NS (800 * 1000 * 1000)  // 1s in nanoseconds
#define DB_PRINT_PERIOD_S (4)                   // 0 seconds

// Priorities
#define SOUND_GEN_PRIORITY 1
#define MEASURING_SPEED_PRIORITY 2
#define BEARING_ISSUES_PRIORITY 3
#define PLAYBACK_PRIORITY 4
#define DB_PRINT_PRIORITY 5

/* ***********************************************
* Prototype
* ***********************************************/
struct  timespec TsAdd(struct  timespec  ts1, struct  timespec  ts2);

/* ***********************************************
* Global variables
* ***********************************************/
const int MAX_RECORDING_DEVICES = 10;		/* Maximum allowed number of souns devices that will be detected */

//Maximum recording time
const int MAX_RECORDING_SECONDS = 5;

//Maximum recording time plus padding
const int RECORDING_BUFFER_SECONDS = MAX_RECORDING_SECONDS + 1;

//Receieved audio spec
SDL_AudioSpec gReceivedRecordingSpec;
SDL_AudioSpec gReceivedPlaybackSpec;

//Recording data buffer
Uint8 * gRecordingBuffer = NULL;

//Size of data buffer
Uint32 gBufferByteSize = 0;

//Position in data buffer
Uint32 gBufferBytePosition = 0;

//Maximum position in data buffer for recording
Uint32 bufferMaxPosition = 0; // Uint32 gBufferByteMaxPosition = 0;


int gRecordingDeviceCount = 0;

char y;

SDL_AudioDeviceID playbackDeviceId = 0; 	/* Structure with ID of playback device */

int possibleMotorFreqs[MOTOR_FREQS_SIZE];	// Array used to compute different motor frequencies

/* ***************************************
 * CAB
 * ***************************************/
#define CAB_SIZE 529200   // Define the size of the CAB buffer

typedef struct{
	int users; // Number of users
	uint8_t *data; // Data buffer
	Uint32 position; // Position in the data buffer
	Uint32 max_position; // Maximum position in the data buffer
	pthread_mutex_t lock; // Synchronization lock
} buffer;

typedef struct {
    Uint32 buffer_size;  // Size of each message
    int num_buffers;  // Total number of buffers available in the CAB
    buffer **buffers;   // Array of pointers to buffer slots
    int current_index; // Index of the most recent message
    // pthread_mutex_t lock; // Synchronization lock
} CAB;

// Initialize CAB
CAB* open_cab(Uint32 buffer_size, int num_buffers) {
    CAB* cab = malloc(sizeof(CAB));
    cab->buffer_size = buffer_size;
    cab->num_buffers = num_buffers;
    // cab->buffers = malloc(num_buffers * sizeof(void*));
    cab->buffers = malloc(num_buffers * sizeof(buffer));

    // Initialize buffers
    for (int i = 0; i < num_buffers; i++) {
        cab->buffers[i] = malloc(sizeof(buffer));
		cab->buffers[i]->data = malloc(buffer_size);
		memset(cab->buffers[i]->data, 0, buffer_size);
		cab->buffers[i]->users = 0;
		cab->buffers[i]->max_position = buffer_size;
		pthread_mutex_init(&cab->buffers[i]->lock, NULL);
    }
    cab->current_index = -1; // No message yet
    // pthread_mutex_init(&cab->lock, NULL);
    return cab;
}

// Write to a buffer in the CAB
void CAB_write(CAB* cab, void* data, int size) {
	// pthread_mutex_lock(&cab->lock);
	//printf("WRITING\n");
	int current_index = -1;
	for(int i = 0; i < cab->num_buffers; i++){
		if(cab->buffers[i]->users == 0){
			current_index = i;
			break;
		}
	}
	// cab->current_index = (cab->current_index + 1) % cab->num_buffers;
	if(current_index == -1)
		printf("Failed to find an available buffer\n");
	// cab->buffers[current_index]->data = data;
	memcpy(cab->buffers[current_index]->data, data, size);
	cab -> current_index = current_index;
	cab -> buffers[current_index]->position = 0;
	// pthread_mutex_unlock(&cab->lock);
}

// Read from the buffer in the CAB that has the most recent data
void CAB_read(CAB* cab, buffer *tempBuffer, int size) {
	int i = cab->current_index;
	pthread_mutex_lock(&cab->buffers[i]->lock);
	cab->buffers[i]->users++;
	pthread_mutex_unlock(&cab->buffers[i]->lock);
	memcpy(tempBuffer, cab->buffers[i], sizeof(buffer));
	pthread_mutex_lock(&cab->buffers[i]->lock);
	cab->buffers[i]->users--; // Decrement the user count after reading
	pthread_mutex_unlock(&cab->buffers[i]->lock);
	
}

// Cleanup CAB
void delete_cab(CAB* cab) {
    for (int i = 0; i < cab->num_buffers; i++) {
        free(cab->buffers[i]);
    }
    free(cab->buffers);
    free(cab);
}

CAB cab;

/* ***************************************
 * CAB
 * ***************************************/


/* ***************************************
 * Data-Base
 * ***************************************/
typedef struct {
	int motorFreqIdx;
	int motorFreq[50];
	int rpm[50];
	int bearingFreqIdx;
	int bearingFreq[50];
	int bearingAmplitude[50];
	char *description;
	int status;
} real_time_data_base;

real_time_data_base db;


/* ************************************************************** 
 * Callback issued by the capture device driver. 
 * Args are:
 * 		userdata: optional user data passed on AudioSpec struct
 * 		stream: buffer of samples acquired 
 * 		len: length of buffer ***in bytes*** (not # of samples) 
 * 
 * Just copy the data to the application buffer and update index 
 * **************************************************************/
void audioRecordingCallback(void* userdata, Uint8* stream, int len )
{
	/* Copy bytes acquired from audio stream */
	// memcpy(&gRecordingBuffer[ gBufferBytePosition ], stream, len);

	// WE NEED TO CHANGE THIS
	memcpy(&cab.buffers[0]->data[cab.buffers[0]->position], stream, len);
	/* Update buffer pointer */
	cab.buffers[0]->position += len;
}

/* *********************************************************************************** 
 * Callback issued by the playback device driver. 
 * Args are:
 * 		userdata: optional user data passed on AudioSpec struct
 * 		stream: buffer o samples acquired 
 * 		len: length of buffer ***in bytes*** (not # of samples) 
 * 
 * Reverse of above. Copy buffer data to the devide-river buffer and update index 
 * **********************************************************************************/
void audioPlaybackCallback(void* userdata, Uint8* stream, int len )
{
	// THERE IS SOMETHING WRONG WITH CAB_READ
	///* Copy buffer with audio samples to stream for playback */

	int i = cab.current_index;
	// while (cab->buffers[i]->users > 0) {
	// 	i = (i + 1) % cab->num_buffers;
	// }

	cab.buffers[i]->users++;
	//printf("Buffer %d has %d users\n", i, cab->buffers[i]->users);
	memcpy(stream, &cab.buffers[i]->data[cab.buffers[i]->position], len);
	// memcpy(stream, &cab.buffers[0]->data[cab.buffers[0]->position], len);

	///* Update buffer index */
	// WE NEED TO CHANGE THIS
	cab.buffers[i]->position += len;
	cab.buffers[i]->users--; // Decrement the user count after reading
	
	
}


/* ***********************************************
 * Debug function: 
 *       Prints the buffer contents - 8 bit samples * 
 * **********************************************/
void printSamplesU8(uint8_t * buffer, int size) {
	int i=0;
	
	printf("\n\r Samples: \n\r");
	for(i = 0; i < size; i++) {
		printf("%3u ",buffer[i]);
		if((i%20) == 0)
			printf("\n\r");
	}		
}

/* ***********************************************
 * Debug function: 
 *       Prints the buffer contents - uint16 samples * 
 * **********************************************/
void printSamplesU16(uint8_t * buffer, int nsamples) {
	int i=0;
	uint16_t * bufu16 = (uint16_t *)buffer;
	 
	printf("\n\r Samples: \n\r");
	for(i = 0; i < nsamples; i++) {
		printf("%5u ",bufu16[i]);
		if((i%20) == 0)
			printf("\n\r");
	}		
}

/* **************************************************
 * Audio processing example:
 *     Adds two echoes - uint16 sample format
 * 		Be carefull with saturation 
 * ************************************************* */ 
void addEchoU16(uint32_t delay1MS, uint32_t delay2MS, float gain1, float gain2, uint8_t * buffer, uint32_t nSamples)
{	
		
	int delaySamples1, delaySamples2;
	int i=0;
		
	uint16_t * procBuffer; 	/* Temporary buffer */
	uint16_t * origBuffer; 	/* Pointer to original buffer, with right sample type (UINT16 in the case) */
		
	/* Get pointer to buffer of the right type */
	origBuffer = (uint16_t *)buffer;
	
	/* Convert time to number of samples */	
	delaySamples1 = SAMP_FREQ * (float)delay1MS/1000;
	delaySamples2 = SAMP_FREQ * (float)delay2MS/1000;
	
	/* allocate temporary buffer and copy data to it*/
	procBuffer = (uint16_t *)malloc(nSamples*sizeof(uint16_t));	
	memcpy((uint8_t *)procBuffer, (uint8_t *)origBuffer, nSamples*sizeof(uint16_t));
	
	/* Add the echoes */		
	for(i = delaySamples1; i < nSamples; i++) {				
		procBuffer[i] += (uint16_t)(origBuffer[i-delaySamples1] * gain1);		
	}
	
	for(i = delaySamples2; i < nSamples; i++) {				
		procBuffer[i] +=  (uint16_t)(origBuffer[i-delaySamples2] * gain2);		
	}

	/* Move data to the original (playback) buffer */
	memcpy((uint8_t *)origBuffer, (uint8_t *)procBuffer, nSamples*sizeof(uint16_t));	
	
	/* Release resources */
	free(procBuffer);	
}

/* **************************************************************
 * Audio processing example:
 *  	Applies a low-pass filter
 * 		Args are cutoff frequency, buffer and nsamples in buffer
 * 
 * 		Simple realization derived from the discretization on an analog RC low-pass filter. See e.g. 
 * 			https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter 
 * ************************************************************ */ 
void filterLP(uint32_t cof, uint32_t sampleFreq, uint8_t * buffer, uint32_t nSamples)
{					
	
	int i;
	
	uint16_t * procBuffer; 	/* Temporary buffer */
	uint16_t * origBuffer; 	/* Pointer to original buffer, with right sample type (UINT16 in the case) */
	
	float alfa, beta; 
		
	/* Compute alfa and beta multipliers */
	alfa = (2 * M_PI / sampleFreq * cof ) / ( (2 * M_PI / sampleFreq * cof ) + 1 );
	beta = 1-alfa;
	
	
	/* Get pointer to buffer of the right type */
	origBuffer = (uint16_t *)buffer;
	
	/* allocate temporary buffer and init it */
	procBuffer = (uint16_t *)malloc(nSamples*sizeof(uint16_t));		
	memset(procBuffer,0, nSamples*sizeof(uint16_t));
	        
	/* Apply the filter */		
	for(i = 1; i < nSamples; i++) {				
		procBuffer[i] = alfa * origBuffer[i] + beta * procBuffer[i-1];		
	}
	
	/* Move data to the original (playback) buffer */
	memcpy(buffer, (uint8_t *)procBuffer, nSamples*sizeof(uint16_t));	
	
	/* Release resources */
	free(procBuffer);	
	
	return;
}



/* *************************************************************************************
 * Audio processing example:
 *      Generates a sine wave. 
 *      Frequency in Hz, durationMS in miliseconds, amplitude 0...0xFFFF, stream buffer 
 * 
 * *************************************************************************************/ 
// void genSineU16(uint16_t freq, uint32_t durationMS, uint16_t amp, uint8_t *buffer)
// {	
// 	int i=0, nSamples=0;
		
// 	float sinArgK = 2*M_PI*freq;				/* Compute once constant part of sin argument, for efficiency */
	
// 	uint16_t * bufU16 = (uint16_t *)buffer; 	/* UINT16 pointer to buffer for access sample by sample */
	
// 	nSamples = ((float)durationMS / 1000) * SAMP_FREQ; 	/* Compute how many samples to generate */
			
// 	/* Generate sine wave */
// 	for(i = 0; i < nSamples; i++) {
// 		bufU16[i] = amp/2*(1+sin((sinArgK*i)/SAMP_FREQ));		
// 	}		
	
// 	return;
// }

void genSine(uint32_t freq, uint32_t durationMS, uint16_t amp, uint8_t *buffer)
{	
	int i=0, nSamples=0;
		
	float sinArgK = 2*M_PI*freq;				/* Compute once constant part of sin argument, for efficiency */
	
	uint16_t * bufU16 = (uint16_t *)buffer; 	/* UINT16 pointer to buffer for access sample by sample */
	
	nSamples = ((float)durationMS / 1000) * SAMP_FREQ; 	/* Compute how many samples to generate */
	//printf("nSamples: %d\n", cab.buffer_size);
	// Ensure nSamples does not exceed the buffer size (ABUFSIZE_SAMPLES)
    //if (nSamples > ABUFSIZE_SAMPLES) {
    //    nSamples = ABUFSIZE_SAMPLES;
    //}
	//printf("nSamples: %d\n", nSamples);
	/* Generate sine wave */
	for(i = 0; i < nSamples; i++) {
		//printf("i: %d\n", i);
		bufU16[i] = amp/2*(1+sin((sinArgK*i)/SAMP_FREQ));		
	}		
	
	return;
}
void genSineWithIssues(uint32_t freq, uint32_t durationMS, uint16_t amp, uint8_t *buffer) {	
	int i=0, nSamples=0;
	float sinArgK = 2 * M_PI * freq;					// Main motor frequency component
	float bearingFreq = 100; 							// Frequency of bearing noise component
	float sinArgBearing = 2 * M_PI * bearingFreq;		// Bearing noise frequency component

	uint16_t * bufU16 = (uint16_t *)buffer; 	// UINT16 pointer to buffer for access sample by sample
	
	nSamples = ((float)durationMS / 1000) * SAMP_FREQ; 	// Compute number of samples to generate

	// Generate sine wave with added bearing noise
	for(i = 0; i < nSamples; i++) {

		// Bearing noise amplitude varies between 0 and 20% of the main amplitude to simulate an intermittent fault
		float bearingAmp = 0.05 * amp * ((rand() % 20) / 100.0);  
		// Main motor sine wave + bearing noise sine wave
		bufU16[i] = (uint16_t)(amp / 2 * (1 + sin((sinArgK * i) / SAMP_FREQ)) + 
		                       bearingAmp * sin((sinArgBearing * i) / SAMP_FREQ));
	}		
}

void generateSimulatedAudio(Uint8 *buffer, int bufferSize, int micNum, float delayInSeconds, int sampleRate) {
    // Simulate time delay by adding silence at the beginning of the buffer
    int delaySamples = delayInSeconds * sampleRate;
    for (int i = 0; i < bufferSize; i++) {
        if (i < delaySamples) {
            buffer[i] = 0;  // Silence for the delay
        } else {
            buffer[i] = sin(2 * M_PI * 440 * (i - delaySamples) / sampleRate) * 127;  // Example sine wave
        }
    }
}


/* *************************************************************************************
 * Debug function 
 *      Returns the max and min amplitude of signal in a buffer - uint16 format
 * 
 * *************************************************************************************/ 
void getMaxMinU16(uint8_t * buffer, uint32_t nSamples, uint32_t * max, uint32_t * min)
{	
	int i=0;
		
	uint16_t * origBuffer; 	/* Pointer to original buffer, with right sample type (UINT16 in the case) */
			
	/* Get pointer to buffer of the right type */
	origBuffer = (uint16_t *)buffer;
	
	/* Get max value */
	*max=origBuffer[0];
	*min=*max;
	for(i = 1; i < nSamples; i++) {		
		if(origBuffer[i] > *max)
			*max=origBuffer[i];
		if(origBuffer[i] < *min)
			*min=origBuffer[i];		
	}
	
	return;	
}

/* ***************************************
 * Threads 
 * ***************************************/

/*
* Periodicies:
* 	- Sound_gen: 100ms
* 	- MeasuringSpeed: 1s
* 	- BearingIssues: 1s
* 	- Playback: 1s
* 	- dbPrint: 1s
*
* Priorities:
* 	- Sound_gen: 1
* 	- MeasuringSpeed: 2
* 	- BearingIssues: 3
* 	- Playback: 4
* 	- dbPrint: 5
*/


// thread to print the contents of the real-time database
// Global variable to store the Gnuplot pipe
FILE *gnuplotPipe = NULL;

void *dbPrint(){
    usleep(THREAD_INIT_OFFSET);
    struct timespec ts, // thread next activation time (absolute)
            ta,         // activation time of current thread activation (absolute)
            tit,        // thread time from last execution,
            ta_ant,     // activation time of last instance (absolute),
            tp;         // Thread period

    /* Set absolute activation time of first instance */
    tp.tv_nsec = DB_PRINT_PERIOD_NS;
    tp.tv_sec = DB_PRINT_PERIOD_S;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts = TsAdd(ts,tp);
    int count = 0;

    // Open a pipe to Gnuplot only once
    if(gnuplotPipe == NULL) {
        gnuplotPipe = popen("gnuplot -persistent", "w");
        if (gnuplotPipe) {
            // Set the Gnuplot window and axis labels
            fprintf(gnuplotPipe, "set title 'Real-time RPM and Bearing Issues'\n");
            fprintf(gnuplotPipe, "set xlabel 'Time (s)'\n");
            fprintf(gnuplotPipe, "set ylabel 'RPM / Bearing Issues'\n");
            fprintf(gnuplotPipe, "set yrange [0:]\n");  // Adjust Y range as needed
            fflush(gnuplotPipe);  // Ensure Gnuplot processes the commands
        } else {
            printf("Error: Could not open Gnuplot.\n");
            return NULL;
        }
    }

    while (1) {
        /* Wait until next cycle */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,&ts,NULL);
        clock_gettime(CLOCK_MONOTONIC, &ta);
        ts = TsAdd(ts,tp);
        if (count == 2){
            int initialIdx;
            FILE *rotations = fopen("rotations.temp", "w");
            FILE *issues = fopen("issues.temp", "w");
            if(db.motorFreqIdx<5)
                initialIdx = 0;
            else
                initialIdx = db.motorFreqIdx-5;
            for (int i = initialIdx; i < db.motorFreqIdx; i++) {
                fprintf(rotations, "%d %d\n", i, db.rpm[i]/1000);  // Each line: x y
            }

            if(db.bearingFreqIdx<5)
                initialIdx = 0;
            else
                initialIdx = db.bearingFreqIdx-5;
            for(int i = initialIdx; i < db.bearingFreqIdx;i++){
                fprintf(issues, "%d %d\n", i, db.bearingFreq[i]);  // Each line: x y
                printf("%d %d\n", i, db.bearingFreq[i]);
            }

            fclose(rotations);
            fclose(issues);

            // Update the plot by sending new data and refreshing the plot in Gnuplot
            if (gnuplotPipe) {
                fprintf(gnuplotPipe, "plot 'rotations.temp' with lines title 'rpm/1000', \\\n");
                fprintf(gnuplotPipe, "     'issues.temp' with lines title 'bearing issues'\n");
                fprintf(gnuplotPipe, "pause 1\n"); // Pause for 1 second between updates
                fflush(gnuplotPipe); // Ensure the new data is flushed to Gnuplot
            }
        }

        // Print real-time database contents
        printf("\nReal time DataBase:\n"
                    "motor frequency: %dHz\n"
                    "rpm: %d \n"
                    ,db.motorFreq[db.motorFreqIdx-1],db.rpm[db.motorFreqIdx-1]);
        if(db.bearingAmplitude[db.bearingFreqIdx-1]>0){
            printf( "bearing issue frequency: %dHZ\n"
                    "bearing issue amplitude: %d\n"
                    ,db.bearingFreq[db.bearingFreqIdx-1],db.bearingAmplitude[db.bearingFreqIdx-1]);
            printf("Detected a problem with the motor!\n\n");
        }
        count=(count+1)%3;
    }
}

// recording thread
//void *recording(void *arg){
//	/* Delays theread execution start to prevent output of main() and thread to get garbled */
//	usleep(THREAD_INIT_OFFSET);
//	/* Timespec variables to manage time */
//	struct timespec ts, // thread next activation time (absolute)
//			ta, 		// activation time of current thread activation (absolute)
//			tit, 		// thread time from last execution,
//			ta_ant, 	// activation time of last instance (absolute),
//			tp; 		// Thread period
//
//	/* Set absolute activation time of first instance */
//	tp.tv_nsec = SOUND_GEN_PERIOD_NS;
//	tp.tv_sec = SOUND_GEN_PERIOD_S;	
//	clock_gettime(CLOCK_MONOTONIC, &ts);
//	ts = TsAdd(ts,tp);	
//
//	/* Other variables */
//	int policy;    // To obtain scheduling policy
//	struct sched_param parm; // To get thread priority
//
//	SDL_AudioDeviceID recordingDeviceId = *((SDL_AudioDeviceID*) arg);
//
//    printf("Recording thread started\n");
//	Uint8 *tempBuffer = (Uint8 *)malloc(cab.buffer_size);  // Temporary buffer for each recording chunk
//
//    /* After being open devices have callback processing blocked (paused_on active), to allow configuration without glitches */
//    /* Devices must be unpaused to allow callback processing */
//    SDL_PauseAudioDevice(recordingDeviceId, SDL_FALSE);  /* Args are SDL device id and pause_on */
//
//    /* Wait until recording buffer is full */
//    while(1)
//    {
//        /* Lock callback. Prevents the following code from competing with callback function */
//        SDL_LockAudioDevice(recordingDeviceId);
//
//        /* Receiving buffer full? */
//        if(gBufferBytePosition > cab.buffer_size){
//            /* Stop recording audio */
//            SDL_PauseAudioDevice(recordingDeviceId, SDL_TRUE);
//            SDL_UnlockAudioDevice(recordingDeviceId);
//            break;
//        }
//
//		memcpy(tempBuffer, gRecordingBuffer, cab.buffer_size);
//
//        // Write data to the CAB
//        CAB_write(&cab, tempBuffer, cab.buffer_size);
//
//
//        /* Buffer not yet full? Keep trying ... */
//        SDL_UnlockAudioDevice(recordingDeviceId);
//    }
//
//    printf("Recording thread finished.\n");
//    //pthread_exit(NULL);
//
//}


// Sound generator thread. This is the Thread that will write in the buffer
void *Sound_gen(void *arg){
	/* Timespec variables to manage time */
	struct timespec ts, // thread next activation time (absolute)
			ta, 		// activation time of current thread activation (absolute)
			tit, 		// thread time from last execution,
			ta_ant, 	// activation time of last instance (absolute),
			tp; 		// Thread period

	/* Set absolute activation time of first instance */
	tp.tv_nsec = SOUND_GEN_PERIOD_NS;
	tp.tv_sec = SOUND_GEN_PERIOD_S;	
	clock_gettime(CLOCK_MONOTONIC, &ts);
	ts = TsAdd(ts,tp);		

	/* Other variables */
	int policy;    // To obtain scheduling policy
	struct sched_param parm; // To get thread priority	

	uint32_t max, min;
	int newIdx, randomNum, lower, upper;

	int idx = rand()%100; // create an index between 0 and 100
	int freq = possibleMotorFreqs[idx];
	int count = 0;
	
	while(1){
		/* Wait until next cycle */
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,&ts,NULL);
		clock_gettime(CLOCK_MONOTONIC, &ta);		
		ts = TsAdd(ts,tp);	
		//printf("Task 'Sound_gen': initiating...\n");

		// values used to generate a number between -7 and 5 IF they dont exceed the upper and lower bounds of the possible values for the motor frequencies
		if(idx >= 7)
			lower = -7;
		else
			lower = -idx;
		if(idx < MOTOR_FREQS_SIZE-5)
			upper = 5;
		else
			upper = MOTOR_FREQS_SIZE-idx-1;
		
		// create a value to change the frequency between -7 values below or 5 values up
		int randomNum = (rand() % (upper - lower + 1)) + lower;
		idx = idx + randomNum;
		// printf("lower: %d, upper: %d, idx: %d\n",lower,upper,idx);
		freq = possibleMotorFreqs[idx]; //next frequency to generate
		// printf("newFreq: %d\n",freq);

		Uint8 * tempBuffer = (uint8_t *)malloc(cab.buffer_size);  // 
		if (count < 5)
			genSine(freq, 1000, 30000, tempBuffer); 	/* freq, durationMS, amp, buffer */
		else
			genSineWithIssues(freq, 1000, 30000, tempBuffer); 	/* freq, durationMS, amp, buffer */

		// get max and min amplitude of the signal form the current buffer in the cab
		// getMaxMinU16(tempBuffer, cab.buffer_size/sizeof(uint16_t), &max, &min);  // getMaxMinU16(uint8_t * buffer, uint32_t nSamplesm, uint32_t max, uint32_t min)		
		// printf("Max amplitude: = %u Min amplitude is:%u\n",max, min);

		filterLP(1000, SAMP_FREQ, tempBuffer, cab.buffer_size/sizeof(uint16_t));
		CAB_write(&cab, tempBuffer, cab.buffer_size);  // Write the sound data to the CAB
		free(tempBuffer);

		count++;
	}
	

	return NULL;
}

// void *LPFilter(void *arg){
// 	/* Delays theread execution start to prevent output of main() and thread to get garbled */
// 	usleep(THREAD_INIT_OFFSET);
// 	/* Timespec variables to manage time */
// 	struct timespec ts, // thread next activation time (absolute)
// 			ta, 		// activation time of current thread activation (absolute)
// 			tit, 		// thread time from last execution,
// 			ta_ant, 	// activation time of last instance (absolute),
// 			tp; 		// Thread period

// 	/* Set absolute activation time of first instance */
// 	tp.tv_nsec = PERIOD_NS;
// 	tp.tv_sec = PERIOD_S-1;	
// 	clock_gettime(CLOCK_MONOTONIC, &ts);
// 	ts = TsAdd(ts,tp);		

// 	int policy;    // To obtain scheduling policy
// 	struct sched_param parm; // To get thread priority
// 	buffer tempBuffer;
	
// 	while(1){
// 		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,&ts,NULL);
// 		clock_gettime(CLOCK_MONOTONIC, &ta);		
// 		ts = TsAdd(ts,tp);	
// 		printf("\n Applying LP filter \n");
				
// 		// TODO memcopy não esta a funcionar

// 		// int i = cab.current_index;
// 		// cab.buffers[i]->users++;
// 		// filterLP(1000, SAMP_FREQ, cab.buffers[i]->data, cab.buffer_size/sizeof(uint16_t));  // Apply the LP filter 
// 		// cab.buffers[i]->users--; // Decrement the user count after reading

		
// 		CAB_read(&cab,&tempBuffer,0);
// 		filterLP(1000, SAMP_FREQ, cab.buffers[0]->data, cab.buffer_size/sizeof(uint16_t));  // Apply the LP filter
// 	} 
// }

void *MeasuringSpeed(void *arg){
	/* Delays theread execution start to prevent output of main() and thread to get garbled */
	usleep(THREAD_INIT_OFFSET);

	struct timespec ts, // thread next activation time (absolute)
			ta, 		// activation time of current thread activation (absolute)
			tit, 		// thread time from last execution,
			ta_ant, 	// activation time of last instance (absolute),
			tp; 		// Thread period

	/* Set absolute activation time of first instance */
	tp.tv_nsec = MEASURING_SPEED_PERIOD_NS;
	tp.tv_sec = MEASURING_SPEED_PERIOD_S;	
	clock_gettime(CLOCK_MONOTONIC, &ts);
	ts = TsAdd(ts,tp);	

	buffer tempBuffer;
	while(1){
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,&ts,NULL);
		clock_gettime(CLOCK_MONOTONIC, &ta);		
		ts = TsAdd(ts,tp);	
		CAB_read(&cab,&tempBuffer,0);

		int N=0;	// Number of samples to take
		int sampleDurationMS = 100; /* Duration of the sample to analyze, in ms */
		int k=0; 	// General counter
		uint16_t * sampleVector = (uint16_t *)cab.buffers[0]->data; /* Pointer to the buffer with samples */
		float * fk; /* Pointer to array with frequencies */
		float * Ak; /* Pointer to array with amplitude for frequency fk[i] */
		complex double * x; /* Pointer to array of complex values for samples and FFT */

		//printf("\nComputing FFT of signal\n");
		
		/* Get the vector size, making sure it is a power of two */
		for(N=1; pow(2,N) < (SAMP_FREQ*sampleDurationMS)/1000; N++);
		N--;
		N=(int)pow(2,N);
		
		//printf("# of samples is: %d\n",N);
		
		/* Allocate memory for  samples, frequency and amplitude vectors */
		x = (complex double *)malloc(N * sizeof(complex double)); /* Array for samples and FFT output */
		fk = (float *)malloc(N * sizeof(float)); 	/* Array with frequencies */
		Ak = (float *)malloc(N * sizeof(float)); 	/* Array with amplitude for frequency fk[i] */
				
		/* Copy samples to complex input vector */
		for(k=0; k<N;k++) {
			x[k] = sampleVector[k];
		}
				
		/* Compute FFT */
		fftCompute(x, N);

		//printf("\nFFT result:\n");/		
		//printComplexArray(x, N);

		/* Compute the amplitude at each frequency and print it */
		fftGetAmplitude(x,N,SAMP_FREQ, fk,Ak);

		double maxAmplitude = 0.0;
		double maxAmplitudeFreq = 0.0;
		double minMotorFreq = 2000.0;
		double maxMotorFreq = 5000.0;

		// 	Divided minMotorFreq by 11 because the samples appear with a difference of 11 most of the times. 
		// The others, appear with a difference of 10.
		// This way, when we iterate through the this smaller part of the lsit, we will ignore less data than when we would 
		// iterate through the whole list (1948 Hz - 5383 Hz instead of 0 Hz - 22050Hz)
		for(k=minMotorFreq/11; k<=maxMotorFreq/10; k++) {	
			if(fk[k] >= minMotorFreq && fk[k] <= maxMotorFreq){
				if (Ak[k] > maxAmplitude){
					maxAmplitude = Ak[k];
					maxAmplitudeFreq = fk[k];
				}
				//printf("Amplitude at frequency %f Hz is %f \n", fk[k], Ak[k]);
			}
			
		}
		//printf("Max amplitude %f at frequency %f\n",maxAmplitude,maxAmplitudeFreq);
		// cab.buffers[i]->users--;
		if(db.motorFreqIdx==50){
			for(int i=0; i<25;i++){
				db.motorFreq[i]=db.motorFreq[i+25];
			}
			db.motorFreqIdx=25;
		}
		db.motorFreq[db.motorFreqIdx] = maxAmplitudeFreq;
		db.rpm[db.motorFreqIdx] = maxAmplitudeFreq * 60;
		db.motorFreqIdx++;
	}
}

void *BearingIssues(void *arg){
	/* Delays theread execution start to prevent output of main() and thread to get garbled */
	usleep(THREAD_INIT_OFFSET);
	struct timespec ts, // thread next activation time (absolute)
			ta, 		// activation time of current thread activation (absolute)
			tit, 		// thread time from last execution,
			ta_ant, 	// activation time of last instance (absolute),
			tp; 		// Thread period

	/* Set absolute activation time of first instance */
	tp.tv_nsec = BEARING_ISSUES_PERIOD_NS;
	tp.tv_sec = BEARING_ISSUES_PERIOD_S;	
	clock_gettime(CLOCK_MONOTONIC, &ts);
	ts = TsAdd(ts,tp);	
	
	buffer tempBuffer;
	while(1){
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,&ts,NULL);
		clock_gettime(CLOCK_MONOTONIC, &ta);		
		ts = TsAdd(ts,tp);	
		
		//printf("Bearing issues\n");
		CAB_read(&cab,&tempBuffer,0);

		int N=0;	// Number of samples to take
		int sampleDurationMS = 100; /* Duration of the sample to analyze, in ms */
		int k=0; 	// General counter
		uint16_t * sampleVector = (uint16_t *)tempBuffer.data; /* Pointer to the buffer with samples */
		float * fk; /* Pointer to array with frequencies */
		float * Ak; /* Pointer to array with amplitude for frequency fk[i] */
		complex double * x; /* Pointer to array of complex values for samples and FFT */

		
		/* Get the vector size, making sure it is a power of two */
		for(N=1; pow(2,N) < (SAMP_FREQ*sampleDurationMS)/1000; N++);
		N--;
		N=(int)pow(2,N);
		
		/* Allocate memory for  samples, frequency and amplitude vectors */
		x = (complex double *)malloc(N * sizeof(complex double)); /* Array for samples and FFT output */
		fk = (float *)malloc(N * sizeof(float)); 	/* Array with frequencies */
		Ak = (float *)malloc(N * sizeof(float)); 	/* Array with amplitude for frequency fk[i] */
				
		/* Copy samples to complex input vector */
		for(k=0; k<N;k++) {
			x[k] = sampleVector[k];
		}
				
		/* Compute FFT */
		fftCompute(x, N);


		/* Compute the amplitude at each frequency and print it */
		fftGetAmplitude(x,N,SAMP_FREQ, fk,Ak);

		// Store the highest amplitude
		double maxAmplitude = 0.0;
		for(k=1;k<N/2;k++){
			if(Ak[k]>maxAmplitude)
				maxAmplitude=Ak[k];
		}

		// check if there is frequencies below 200Hz that have an amplitude bigger than 20% of the highest amplitude
		for(k=1; k<=N/2; k++) {	
			if(fk[k] > 200.0)
				break;
			if(Ak[k] > 0.2*maxAmplitude){
				//printf("Bearing issue detected at %f with amplitude: %f with maxAmplitude: %f\n",fk[k],Ak[k],maxAmplitude);
				db.bearingFreq[db.bearingFreqIdx] = fk[k];
				db.bearingAmplitude[db.bearingFreqIdx] = Ak[k];
			}
		}
		if(db.bearingFreqIdx==50){
			for(int i=0; i<25;i++){
				db.bearingFreq[i]=db.bearingFreq[i+25];
			}
			db.bearingFreqIdx=25;
		}
		db.bearingFreqIdx++;
		free(x);
		free(fk);
		free(Ak);
		

	}
}

void *Playback(void *arg){
	/* Delays theread execution start to prevent output of main() and thread to get garbled */
	usleep(THREAD_INIT_OFFSET);
	struct timespec ts, // thread next activation time (absolute)
			ta, 		// activation time of current thread activation (absolute)
			tit, 		// thread time from last execution,
			ta_ant, 	// activation time of last instance (absolute),
			tp; 		// Thread period

	/* Set absolute activation time of first instance */
	tp.tv_nsec = PLAYBACK_PERIOD_NS;
	tp.tv_sec = PLAYBACK_PERIOD_S;	
	clock_gettime(CLOCK_MONOTONIC, &ts);
	ts = TsAdd(ts,tp);	
	while(1){
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,&ts,NULL);
		clock_gettime(CLOCK_MONOTONIC, &ta);		
		ts = TsAdd(ts,tp);	


		printf("Playback\n");
			
		/* Reset buffer index to the beginning */
		// gBufferBytePosition = 0;
		int i = cab.current_index;
		cab.buffers[i]->position = 0;

		/* Enable processing of callbacks by playback device (required after opening) */
		SDL_PauseAudioDevice(playbackDeviceId, SDL_FALSE);

		/* Play buffer */
		while(1)
		{
			/* Lock callback */
			SDL_LockAudioDevice(playbackDeviceId);

			/* Playback is finished? */
			if(cab.buffers[i]->position > bufferMaxPosition)
			{
				/* Stop playing audio */
				SDL_PauseAudioDevice(playbackDeviceId, SDL_TRUE);
				SDL_UnlockAudioDevice(playbackDeviceId);
				break;
			}
			/* Unlock callback and try again ...*/
			SDL_UnlockAudioDevice(playbackDeviceId);

		}
	}
}



/* ***************************************
 * Main 
 * ***************************************/
int main(int argc, char ** argv)
{
	
	/* ****************
	 *  Variables 
	 **************** */
	SDL_AudioDeviceID recordingDeviceId = 0; 	/* Structure with ID of recording device */
	// SDL_AudioDeviceID playbackDeviceId = 0; 	/* Structure with ID of playback device */
	SDL_AudioSpec desiredPlaybackSpec;			/* Structure for desired playback attributes (the ones returned may differ) */
	const char * deviceName;					/* Capture device name */
	int index;									/* Device index used to browse audio devices */
	int bytesPerSample;							/* Number of bytes each sample requires. Function of size of sample and # of channels */ 
	int bytesPerSecond;							/* Intuitive. bytes per sample sample * sampling frequency */
	int err;
	int nthreads = 5;
	
	/* Initialize the array where we store the frequencies that the motor can have*/
	for(int i= 0;i<MOTOR_FREQS_SIZE;i++){
		possibleMotorFreqs[i]=2000+i*(3000/MOTOR_FREQS_SIZE);
	}

	/* Threads */
	pthread_t threadid[nthreads];
	struct sched_param parm;
	pthread_attr_t attr[nthreads];
	int prio[]={SOUND_GEN_PRIORITY,MEASURING_SPEED_PRIORITY,BEARING_ISSUES_PRIORITY,PLAYBACK_PRIORITY,DB_PRINT_PRIORITY};


	/* Create periodic thread/task with RT scheduling attributes*/
	for(int i = 0; i < nthreads; i++){
		pthread_attr_init(&attr[i]);
		pthread_attr_setinheritsched(&attr[i], PTHREAD_EXPLICIT_SCHED);
		pthread_attr_setschedpolicy(&attr[i], SCHED_OTHER);
		parm.sched_priority = prio[i];
		pthread_attr_setschedparam(&attr[i], &parm);
	}
	
	/* SDL Init */
	if(SDL_Init(SDL_INIT_AUDIO) < 0)
	{
		printf("SDL could not initialize! SDL Error: %s\n", SDL_GetError());
		return 1;
	}

	/* *************************************
	 * Get and open recording device 
	 ************************************* */
	SDL_AudioSpec desiredRecordingSpec;
	/* Defined in SDL_audio.h */
	SDL_zero(desiredRecordingSpec);				/* Init struct with default values */
	desiredRecordingSpec.freq = SAMP_FREQ;		/* Samples per second */
	desiredRecordingSpec.format = FORMAT;		/* Sampling format */
	desiredRecordingSpec.channels = MONO;		/* 1 - mono; 2 stereo */
	desiredRecordingSpec.samples = ABUFSIZE_SAMPLES;		/* Audio buffer size in sample FRAMES (total samples divided by channel count) */
	desiredRecordingSpec.callback = audioRecordingCallback;

	/* Get number of recording devices */
	gRecordingDeviceCount = SDL_GetNumAudioDevices(SDL_TRUE);		/* Argument is "iscapture": 0 to request playback device, !0 for recording device */

	if(gRecordingDeviceCount < 1)
	{
		printf( "Unable to get audio capture device! SDL Error: %s\n", SDL_GetError() );
		return 0;
	}
	
	/* and lists them */
	for(int i = 0; i < gRecordingDeviceCount; ++i)
	{
		//Get capture device name
		deviceName = SDL_GetAudioDeviceName(i, SDL_TRUE);/* Arguments are "index" and "iscapture"*/
		printf("%d - %s\n", i, deviceName);
	}

	/* If device index supplied as arg, use it, otherwise, ask the user */
	if(argc == 2) {
		index = atoi(argv[1]);
	} else {
		/* allow the user to select the recording device */
		printf("Choose audio\n");
		scanf("%d", &index);
	}
	
	if(index < 0 || index >= gRecordingDeviceCount) {
		printf( "Invalid device ID. Must be between 0 and %d\n", gRecordingDeviceCount-1 );
		return 0;
	} else {
		printf( "Using audio capture device %d - %s\n", index, deviceName );
	}

	/* and open it */
	recordingDeviceId = SDL_OpenAudioDevice(SDL_GetAudioDeviceName(index, SDL_TRUE), SDL_TRUE, &desiredRecordingSpec, &gReceivedRecordingSpec, SDL_AUDIO_ALLOW_FORMAT_CHANGE);
	
	/* if device failed to open terminate */
	if(recordingDeviceId == 0)
	{
		//Report error
		printf("Failed to open recording device! SDL Error: %s", SDL_GetError() );
		return 1;
	}


	/* **********************************
	 *  Get and open playback 
	 * **********************************/
	
	SDL_zero(desiredPlaybackSpec);
	desiredPlaybackSpec.freq = SAMP_FREQ;
	desiredPlaybackSpec.format = FORMAT; 
	desiredPlaybackSpec.channels = MONO;
	desiredPlaybackSpec.samples = ABUFSIZE_SAMPLES;
	desiredPlaybackSpec.callback = audioPlaybackCallback;
	

	/* Open playback device */
	playbackDeviceId = SDL_OpenAudioDevice( NULL, SDL_FALSE, &desiredPlaybackSpec, &gReceivedPlaybackSpec, SDL_AUDIO_ALLOW_FORMAT_CHANGE );

	/* if error abort */
	if(playbackDeviceId == 0)
	{
		//Report error
		printf("Failed to open playback device! SDL Error: %s", SDL_GetError());
		return 1;
	}


	/* **************************************************
	 * Recording and playback devices opened and OK.
	 * Time to init some data structures 
	 * **************************************************/ 
	/* Calculate number of bytes per SAMPLE */
	bytesPerSample = gReceivedRecordingSpec.channels * (SDL_AUDIO_BITSIZE(gReceivedRecordingSpec.format) / 8);

	/* Calculate number of bytes per SECOND */
	bytesPerSecond = gReceivedRecordingSpec.freq * bytesPerSample;

	/* Calculate buffer size, for the desired duration  */
	Uint32 size = RECORDING_BUFFER_SECONDS * bytesPerSecond;

	/* Calculate max buffer use - some additional space to allow for extra samples*/
	/* Detection of buffer use is made form device-driver callback, so can be a biffer overrun if some */
	/* leeway is not added */
	bufferMaxPosition = MAX_RECORDING_SECONDS * bytesPerSecond;

	// 
	cab = *open_cab(size,32);

	
	printf("\n\r *********** \n\r");
	printf("bytesPerSample=%d, bytesPerSecond=%d, buffer byte size=%d (allocated) buffer byte size=%d (for nominal recording)", \
			bytesPerSample, bytesPerSecond,size, bufferMaxPosition);
	printf("\n\r *********** \n\r");


//#define RECORD
#ifdef RECORD

	err = pthread_create(&threadid[0], &attr[0], recording, &recordingDeviceId);
	if(err != 0)
		printf("\n\r Error creating Thread record [%s]", strerror(err));

#endif

#define GENSINE
#ifdef GENSINE
	// printf("\n Generating a sine wave \n");
	// genSineU16(1000, 1000, 30000, gRecordingBuffer); 	/* freq, durationMS, amp, buffer */	
	err = pthread_create(&threadid[0], &attr[0], Sound_gen, NULL);
	if(err != 0)
		printf("\n\r Error creating Thread genshin [%s]", strerror(err));
	///////
	// err=pthread_join(threadid[0], NULL);
	// if(err != 0)
	// 	printf("\n\r Error joining Thread [%s]", strerror(err));
	///////
#endif

//#define ADDECHO
#ifdef ADDECHO
	printf("Adding an echo \n");
	addEchoU16(1000, 2000, 0.2, 0.1,gRecordingBuffer,gBufferByteMaxPosition/sizeof(uint16_t)); /* (float delay1MS, float delay2MS, float gain1, float gain2, uint8 * buffer, uint32_t nSamples) */
#endif

	/* For debug - if you want to check the data. */
	/* Can be usefull to add a function that dumps the data to a file for processing e.g. in Matlab/Octave */
//	printSamplesU16(gRecordingBuffer,4000); /* Args are pointer to buffer and nsamples (not bytes)	*/		
	
	/* For debug															*/ 
	/* Getting max-min can be usefull to detect possible saturation 		*/
	{
		/*
		uint32_t max, min;
		//get max and min amplitude of the signal form the current buffer in the cab
		getMaxMinU16(cab.buffers[cab.current_index]->data, cab.buffer_size/sizeof(uint16_t), &max, &min);  // getMaxMinU16(uint8_t * buffer, uint32_t nSamplesm, uint32_t max, uint32_t min)		
		printf("Max amplitude: = %u Min amplitude is:%u\n",max, min);
		*/
		// contents of the buffer
		//printSamplesU16(cab.buffers[cab.current_index], 1000);
	}


//#define MAXMIN
#ifdef MAXMIN	
	{
		uint32_t max, min;
		getMaxMinU16(gRecordingBuffer, gBufferByteMaxPosition/sizeof(uint16_t), &max, &min);  // getMaxMinU16(uint8_t * buffer, uint32_t nSamplesm, uint32_t max, uint32_t min)
		printf("\n Max amplitude = %u Min amplitude = %u\n",max, min);
	}
#endif	

#define LPFILTER
#ifdef LPFILTER
	/* Apply LP filter */
	/* Args are cutoff freq, sampling freq, buffer and # of samples in the buffer */
	// err = pthread_create(&threadid[1], &attr[1], LPFilter, NULL);
	// if(err != 0)
	// 	printf("\n\r Error creating Thread [%s]", strerror(err));
	// err=pthread_join(threadid[1], NULL);
	// if(err != 0)
	// 	printf("\n\r Error joining Thread [%s]", strerror(err));
#endif

#define FFT
#ifdef FFT
	{		

		err = pthread_create(&threadid[1], &attr[1], MeasuringSpeed, NULL);
		if(err != 0)
			printf("\n\r Error creating Thread fft [%s]", strerror(err));
		// err = pthread_join(threadid[2], NULL);
		// if(err != 0)
		// 	printf("\n\r Error creating Thread [%s]", strerror(err));

		
	}
	
#endif

#define BEARINGISSUES
#ifdef BEARINGISSUES
	err = pthread_create(&threadid[2], &attr[2], BearingIssues, NULL);
	if(err != 0)
		printf("\n\r Error creating Thread bear-issue [%s]", strerror(err));
	// err = pthread_join(threadid[3], NULL);
	// if(err != 0)
	// 	printf("\n\r Error creating Thread [%s]", strerror(err));
#endif
	
	/* *****************************************************************
	 * Recorded/generated data obtained. Now play it back
	 * *****************************************************************/
	// printf("Playback\n");
		
	// /* Reset buffer index to the beginning */
	// // gBufferBytePosition = 0;
	// int i = cab.current_index;
	// cab.buffers[i]->position = 0;

	// /* Enable processing of callbacks by playback device (required after opening) */
	// SDL_PauseAudioDevice(playbackDeviceId, SDL_FALSE);

	// /* Play buffer */
	// while(1)
	// {
	// 	/* Lock callback */
	// 	SDL_LockAudioDevice(playbackDeviceId);

	// 	/* Playback is finished? */
	// 	if(cab.buffers[i]->position > bufferMaxPosition)
	// 	{
	// 		/* Stop playing audio */
	// 		SDL_PauseAudioDevice(playbackDeviceId, SDL_TRUE);
	// 		SDL_UnlockAudioDevice(playbackDeviceId);
	// 		break;
	// 	}
	// 	/* Unlock callback and try again ...*/
	// 	SDL_UnlockAudioDevice(playbackDeviceId);

	// }

	err = pthread_create(&threadid[3], &attr[3], Playback, NULL);
	if(err != 0)
		printf("\n\r Error creating Thread playback [%s]", strerror(err));


#define DB
#ifdef DB
	err = pthread_create(&threadid[4], &attr[4], dbPrint, NULL);
	if(err != 0){
		printf("\n\r Error creating Thread databse [%s]", strerror(err));
	}
#endif

	// err=pthread_join(threadid[4], NULL);
	// if(err != 0)
	// 	printf("\n\r Error joining Thread [%s]", strerror(err));

	/* *******************************************
	 * All done! Release resources and terminate
	 * *******************************************/
	// if( gRecordingBuffer != NULL )
	// {
	// 	free(gRecordingBuffer);
	// 	gRecordingBuffer = NULL;
	// }


	// SDL_Quit();
	while (1);
	// for(int i=0;i<nthreads;i++){
	// 	err = pthread_join(threadid[i], NULL);
	// 	if(err != 0)
	// 		printf("\n\r Error creating Thread [%s]", strerror(err));
	// }
	
	return 0;
}





// Adds two timespect variables
struct  timespec  TsAdd(struct  timespec  ts1, struct  timespec  ts2){
	
	struct  timespec  tr;
	
	// Add the two timespec variables
		tr.tv_sec = ts1.tv_sec + ts2.tv_sec ;
		tr.tv_nsec = ts1.tv_nsec + ts2.tv_nsec ;
	// Check for nsec overflow	
	if (tr.tv_nsec >= NS_IN_SEC) {
			tr.tv_sec++ ;
		tr.tv_nsec = tr.tv_nsec - NS_IN_SEC ;
		}

	return (tr) ;
}
