/**
 ******************************************************************************
 * @file    main.cpp
 * @author  AST / EST
 * @version V0.0.1
 * @date    9-August-2016
 * @brief   Simple Example application for using the X_NUCLEO_IKS01A1 
 *          MEMS Inertial & Environmental Sensor Nucleo expansion board.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
*/ 

/* Includes */
#include "mbed.h"
#include "x_nucleo_iks01a2.h"

/* Instantiate the expansion board */
static X_NUCLEO_IKS01A2 *mems_expansion_board = X_NUCLEO_IKS01A2::Instance(D14, D15);

/* Retrieve the composing elements of the expansion board */
static LSM303AGR_MAG_Sensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGR_ACC_Sensor *accelerometer = mems_expansion_board->accelerometer;

/* Helper function for printing floats & doubles */
static char *printDouble(char* str, double v, int decimalDigits=2)
{
  int i = 1;
  int intPart, fractPart;
  int len;
  char *ptr;

  /* prepare decimal digits multiplicator */
  for (;decimalDigits!=0; i*=10, decimalDigits--);

  /* calculate integer & fractinal parts */
  intPart = (int)v;
  fractPart = (int)((v-(double)(int)v)*i);

  /* fill in integer part */
  sprintf(str, "%i.", intPart);

  /* prepare fill in of fractional part */
  len = strlen(str);
  ptr = &str[len];

  /* fill in leading fractional zeros */
  for (i/=10;i>1; i/=10, ptr++) {
    if(fractPart >= i) break;
    *ptr = '0';
  }

  /* fill in (rest of) fractional part */
  sprintf(ptr, "%i", fractPart);

  return str;
}


/* Simple main function */
int main() {
  uint8_t id;
  float value1, value2;
  char buffer1[32], buffer2[32];
  int32_t axes[3];
  
  /* Enable all sensors */
  hum_temp->Enable();
  press_temp->Enable();
  magnetometer->Enable();
  accelerometer->Enable();
  acc_gyro->Enable_X();
  acc_gyro->Enable_G();
  
  printf("\r\n--- Starting new run ---\r\n");

  hum_temp->ReadID(&id);
  printf("HTS221  humidity & temperature    = 0x%X\r\n", id);
  press_temp->ReadID(&id);
  printf("LPS22HB  pressure & temperature   = 0x%X\r\n", id);
  magnetometer->ReadID(&id);
  printf("LSM303AGR magnetometer            = 0x%X\r\n", id);
  accelerometer->ReadID(&id);
  printf("LSM303AGR accelerometer           = 0x%X\r\n", id);
  acc_gyro->ReadID(&id);
  printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);
  
  wait(3);
 
  while(1) {
    printf("\r\n");

    hum_temp->GetTemperature(&value1);
    hum_temp->GetHumidity(&value2);
    printf("HTS221: [temp] %7s C,   [hum] %s%%\r\n", printDouble(buffer1, value1), printDouble(buffer2, value2));
    
    press_temp->GetTemperature(&value1);
    press_temp->GetPressure(&value2);
    printf("LPS22HB: [temp] %7s C, [press] %s mbar\r\n", printDouble(buffer1, value1), printDouble(buffer2, value2));

    printf("---\r\n");

    magnetometer->GetAxes(axes);
    printf("LSM303AGR [mag/mgauss]:  %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
    
    accelerometer->GetAxes(axes);
    printf("LSM303AGR [acc/mg]:  %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    acc_gyro->Get_X_Axes(axes);
    printf("LSM6DSL [acc/mg]:      %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    acc_gyro->Get_G_Axes(axes);
    printf("LSM6DSL [gyro/mdps]:   %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    wait(1.5);
  }
}
