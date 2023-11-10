#include "mbed.h"
#include <cstdio>



#include "arm_math.h"
#include "math_helper.h"
#include "stm32l475e_iot01_gyro.h"

#if defined(SEMIHOSTING)
#include <stdio.h>
#endif



#define TEST_LENGTH_SAMPLES 320

#define SNR_THRESHOLD_F32 75.0f
#define BLOCK_SIZE 32

#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)

#define NUM_TAPS_ARRAY_SIZE 32
#else
#define NUM_TAPS_ARRAY_SIZE 29
#endif

#define NUM_TAPS 29


extern float32_t testInput_f32_1kHz_15kHz[TEST_LENGTH_SAMPLES];
extern float32_t refOutput[TEST_LENGTH_SAMPLES];


static float32_t testOutput[TEST_LENGTH_SAMPLES];

#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
static float32_t firStateF32[2 * BLOCK_SIZE + NUM_TAPS - 1];
#else
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
#endif

#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
    -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f,
    +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
    -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f,
    +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
    +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f,
    -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
    +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f,
    -0.0018225230f, 0.0f,           0.0f,           0.0f};
#else
const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
    -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f,
    +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
    -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f,
    +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
    +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f,
    -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
    +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f,
    -0.0018225230f};
#endif


uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES / BLOCK_SIZE;

float32_t snr;


int32_t main(void) {
  BSP_GYRO_Init();
  uint32_t i;
  arm_fir_instance_f32 S;
  arm_status status;
  float32_t *inputF32, *outputF32;
  float32_t pGyroDataXYZ[3] = {0};
  float32_t pGyroZSeq[320] = {0};
  int currentIdx = 0;
  outputF32 = &testOutput[0];
  /*
  arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);
  for(i=0; i < numBlocks; i++)
  {
    arm_fir_f32(&S, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
  }
  */

  arm_fir_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0],
                   blockSize);

  for (int i = 0; i < TEST_LENGTH_SAMPLES; i++) {
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    pGyroZSeq[i] = pGyroDataXYZ[2];
    ThisThread::sleep_for(50);
  }
  for (i = 0; i < numBlocks; i++) {
    arm_fir_f32(&S, pGyroZSeq + (i * blockSize), outputF32 + (i * blockSize),
                blockSize);
  }
  for (int i = 0; i < TEST_LENGTH_SAMPLES; i++) {
    printf("%f, ", pGyroZSeq[i]);
  }
  printf("\n\n\n");
  for (int i = 0; i < TEST_LENGTH_SAMPLES; i++) {
    printf("%f, ", testOutput[i]);
  }
  printf("\n\n\n");
  while (1);
}
