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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <complex.h>
#include <sched.h> //sched_setscheduler
#include <pthread.h>

#include "fft/fft.h"


#define MONO 1 					/* Sample and play in mono (1 channel) */
#define SAMP_FREQ 44100			/* Sampling frequency used by audio device */
#define FORMAT AUDIO_U16		/* Format of each sample (signed, unsigned, 8,16 bits, int/float, ...) */
#define ABUFSIZE_SAMPLES 4096	/* Audio buffer size in sample FRAMES (total samples divided by channel count) */

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
Uint32 gBufferByteMaxPosition = 0;

int gRecordingDeviceCount = 0;

char y;


/* ***************************************
 * CAB
 * ***************************************/
#define CAB_SIZE 529200   // Define the size of the CAB buffer

typedef struct{
	int users; // Number of users
	Uint8 *data; // Data buffer
	Uint32 position; // Position in the data buffer
	Uint32 max_position; // Maximum position in the data buffer
} buffer;

typedef struct {
    Uint32 buffer_size;  // Size of each message
    int num_buffers;  // Total number of buffers available in the CAB
    buffer **buffers;   // Array of pointers to buffer slots
    int current_index; // Index of the most recent message
	int users; // Number of users
    pthread_mutex_t lock; // Synchronization lock
} CAB;

// Initialize CAB
CAB* open_cab(Uint32 buffer_size, int num_buffers) {
    CAB* cab = malloc(sizeof(CAB));
    cab->buffer_size = buffer_size;
    cab->num_buffers = num_buffers;
    cab->buffers = malloc(num_buffers * sizeof(void*));

    // Initialize buffers
    for (int i = 0; i < num_buffers; i++) {
        cab->buffers[i] = malloc(buffer_size);
		cab->buffers[i]->data = malloc(buffer_size);
    }
    cab->current_index = -1; // No message yet
    pthread_mutex_init(&cab->lock, NULL);
    return cab;
}

// Write to a buffer in the CAB
void CAB_write(CAB* cab, void* data, int size) {
	pthread_mutex_lock(&cab->lock);
	cab->current_index = (cab->current_index + 1) % cab->num_buffers;
	memcpy(cab->buffers[cab->current_index], data, size);
	pthread_mutex_unlock(&cab->lock);
}

// Read from a buffer in the CAB that does not have the number of user > 0
void CAB_read(CAB* cab, uint8_t * data, int size) {
	int i = cab->current_index;
	while (cab->buffers[i]->users > 0) {
		i = (i + 1) % cab->num_buffers;
	}
	cab->buffers[i]->users++;
	//printf("Buffer %d has %d users\n", i, cab->buffers[i]->users);
	memcpy(data, cab->buffers[i]->data, size);
	
	cab->buffers[i]->users--; // Decrement the user count after reading
}



CAB cab;

/* ***************************************
 * CAB
 * ***************************************/



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
	memcpy(&gRecordingBuffer[ gBufferBytePosition ], stream, len);

	/* Update buffer pointer */
	gBufferBytePosition += len;
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
	///* Copy buffer with audio samples to stream for playback */
	memcpy(stream, &gRecordingBuffer[ gBufferBytePosition ], len);

	///* Update buffer index */
	gBufferBytePosition += len;
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

// Sound generator thread. This is the Thread that will write in the buffer
void *Sound_gen(void *arg){
	
	printf("Task 'Sound_gen': initiating...\n");

	/* Other variables */
	int policy;    // To obtain scheduling policy
	struct sched_param parm; // To get thread priority
	
	//printf("\n Generating a sine wave \n");
	// genSineU16(1000, 1000, 30000, gRecordingBuffer); 	/* freq, durationMS, amp, buffer */
	//printf("temp buffer\n");
	uint8_t tempBuffer[cab.buffer_size];  // 
	//printf("ABUFSIZE_SAMPLES: %lu\n", 44100 * sizeof(uint16_t));
	//printf("genSine\n");
	genSine(4000, 1000, 30000, tempBuffer); 	/* freq, durationMS, amp, buffer */
	//printf("CAB_write\n");
	
	CAB_write(&cab, tempBuffer, cab.buffer_size);  // Write the sound data to the CAB

	printf("Task 'Sound_gen': done.\n");

	return NULL;
}

void *LPFilter(void *arg){
	

	int policy;    // To obtain scheduling policy
	struct sched_param parm; // To get thread priority
	printf("\n Applying LP filter \n");
	
	uint8_t tempBuffer[cab.buffer_size];  // Temporary buffer to store data 
	printf("CAB_read\n");
	CAB_read(&cab, tempBuffer, cab.buffer_size);  // Read the sound data from the CAB

	// content of the buffer
	//printf("LPFilter\n");
	//printSamplesU16(tempBuffer, 1000);


	filterLP(1000, SAMP_FREQ, tempBuffer, cab.buffer_size/sizeof(uint16_t));  // Apply the LP filter 


}

void *MeasuringSpeed(void *arg){

	uint8_t tempBuffer[cab.buffer_size];  // Temporary buffer to store data
	CAB_read(&cab, tempBuffer, cab.buffer_size);  // Read the sound data from the CAB

	// content of the buffer
	//printSamplesU16(tempBuffer, 1000);

	int N=0;	// Number of samples to take
	int sampleDurationMS = 100; /* Duration of the sample to analyze, in ms */
	int k=0; 	// General counter
	uint16_t * sampleVector = (uint16_t *)tempBuffer; /* Pointer to the buffer with samples */
	float * fk; /* Pointer to array with frequencies */
	float * Ak; /* Pointer to array with amplitude for frequency fk[i] */
	complex double * x; /* Pointer to array of complex values for samples and FFT */

	printf("\nComputing FFT of signal\n");
	
	/* Get the vector size, making sure it is a power of two */
	for(N=1; pow(2,N) < (SAMP_FREQ*sampleDurationMS)/1000; N++);
	N--;
	N=(int)pow(2,N);
	
	printf("# of samples is: %d\n",N);
	
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
			printf("Amplitude at frequency %f Hz is %f \n", fk[k], Ak[k]);
		}
		
	}
	printf("Max amplitude %f at frequency %f\n",maxAmplitude,maxAmplitudeFreq);
}

void *BearingIssues(void *arg){
	
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
	SDL_AudioDeviceID playbackDeviceId = 0; 	/* Structure with ID of playback device */
	SDL_AudioSpec desiredPlaybackSpec;			/* Structure for desired playback attributes (the ones returned may differ) */
	const char * deviceName;					/* Capture device name */
	int index;									/* Device index used to browse audio devices */
	int bytesPerSample;							/* Number of bytes each sample requires. Function of size of sample and # of channels */ 
	int bytesPerSecond;							/* Intuitive. bytes per sample sample * sampling frequency */
	int err;
	int nthreads = 4;
	
	

	/* Threads */
	pthread_t threadid[nthreads];
	struct sched_param parm;
	pthread_attr_t attr[nthreads];
	int prio[]={50,50,50,50};

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
	desiredPlaybackSpec.userdata = &cab;

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
	Uint32 bufferMaxPosition = MAX_RECORDING_SECONDS * bytesPerSecond;

	// 
	cab = *open_cab(size,32);

	
	printf("\n\r *********** \n\r");
	printf("bytesPerSample=%d, bytesPerSecond=%d, buffer byte size=%d (allocated) buffer byte size=%d (for nominal recording)", \
			bytesPerSample, bytesPerSecond,size, bufferMaxPosition);
	printf("\n\r *********** \n\r");

//#define RECORD
#ifdef RECORD

	/* ******************************************************
	 * All set. Time to record, process and play sounds  
	 * ******************************************************/
	
	printf("Recording\n");
	
	/* Set index to the beginning of buffer */
	gBufferBytePosition = 0;

	/* After being open devices have callback processing blocked (paused_on active), to allow configuration without glitches */
	/* Devices must be unpaused to allow callback processing */
	SDL_PauseAudioDevice(recordingDeviceId, SDL_FALSE ); /* Args are SDL device id and pause_on */
	
	/* Wait until recording buffer full */
	while(1)
	{
		/* Lock callback. Prevents the following code to not concur with callback function */
		SDL_LockAudioDevice(recordingDeviceId);

		/* Receiving buffer full? */
		if(gBufferBytePosition > gBufferByteMaxPosition)
		{
			/* Stop recording audio */
			SDL_PauseAudioDevice(recordingDeviceId, SDL_TRUE );
			SDL_UnlockAudioDevice(recordingDeviceId );
			break;
		}

		/* Buffer not yet full? Keep trying ... */
		SDL_UnlockAudioDevice( recordingDeviceId );
	}

	/* *****************************************************************
	 * Recorded data obtained. Now process it and play it back
	 * *****************************************************************/
 
#endif

#define GENSINE
#ifdef GENSINE
	// printf("\n Generating a sine wave \n");
	// genSineU16(1000, 1000, 30000, gRecordingBuffer); 	/* freq, durationMS, amp, buffer */	
	err = pthread_create(&threadid[0], &attr[0], Sound_gen, NULL);
	if(err != 0)
		printf("\n\r Error creating Thread [%s]", strerror(err));
	///////
	err=pthread_join(threadid[0], NULL);
	if(err != 0)
		printf("\n\r Error joining Thread [%s]", strerror(err));
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
		
		uint32_t max, min;
		//get max and min amplitude of the signal form the current buffer in the cab
		getMaxMinU16(cab.buffers[cab.current_index], cab.buffer_size/sizeof(uint16_t), &max, &min);  // getMaxMinU16(uint8_t * buffer, uint32_t nSamplesm, uint32_t max, uint32_t min)		
		printf("Max amplitude: = %u Min amplitude is:%u\n",max, min);
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
	// printf("\n Applying LP filter \n");
	// filterLP(1000, SAMP_FREQ, gRecordingBuffer, gBufferByteMaxPosition/sizeof(uint16_t)); 
	err = pthread_create(&threadid[1], &attr[1], LPFilter, NULL);
	if(err != 0)
		printf("\n\r Error creating Thread [%s]", strerror(err));
	err=pthread_join(threadid[1], NULL);
	if(err != 0)
		printf("\n\r Error joining Thread [%s]", strerror(err));
#endif

#define FFT
#ifdef FFT
	{		
		

		err = pthread_create(&threadid[2], &attr[2], MeasuringSpeed, NULL);
		if(err != 0)
			printf("\n\r Error creating Thread [%s]", strerror(err));
		err = pthread_join(threadid[2], NULL);
		if(err != 0)
			printf("\n\r Error creating Thread [%s]", strerror(err));

		
	}
	
#endif
	
	/* *****************************************************************
	 * Recorded/generated data obtained. Now play it back
	 * *****************************************************************/
	printf("Playback\n");
		
	/* Reset buffer index to the beginning */
	gBufferBytePosition = 0;
	
	/* Enable processing of callbacks by playback device (required after opening) */
	SDL_PauseAudioDevice(playbackDeviceId, SDL_FALSE);

	/* Play buffer */
	while(1)
	{
		/* Lock callback */
		SDL_LockAudioDevice(playbackDeviceId);

		/* Playback is finished? */
		if(gBufferBytePosition > gBufferByteMaxPosition)
		{
			/* Stop playing audio */
			SDL_PauseAudioDevice(playbackDeviceId, SDL_TRUE);
			SDL_UnlockAudioDevice(playbackDeviceId);	
			break;
		}

		/* Unlock callback and try again ...*/
		SDL_UnlockAudioDevice(playbackDeviceId);
	}

	/* *******************************************
	 * All done! Release resources and terminate
	 * *******************************************/
	if( gRecordingBuffer != NULL )
	{
		free(gRecordingBuffer);
		gRecordingBuffer = NULL;
	}

	SDL_Quit();
	
	return 0;
}
