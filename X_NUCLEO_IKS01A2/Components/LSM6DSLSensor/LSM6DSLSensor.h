/**
 ******************************************************************************
 * @file    LSM6DSLSensor.h
 * @author  AST
 * @version V1.0.0
 * @date    5 August 2016
 * @brief   Abstract Class of an LSM6DSL Inertial Measurement Unit (IMU) 6 axes
 *          sensor.
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
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LSM6DSLSensor_H__
#define __LSM6DSLSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "DevI2C.h"
#include "LSM6DSL_ACC_GYRO_driver.h"
#include "MotionSensor.h"
#include "GyroSensor.h"

/* Defines -------------------------------------------------------------------*/

#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_FOR_FS_16G  0.488  /**< Sensitivity value for 16 g full scale [mg/LSB] */

#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_125DPS   04.375  /**< Sensitivity value for 125 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_245DPS   08.750  /**< Sensitivity value for 245 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_500DPS   17.500  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_1000DPS  35.000  /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */
#define LSM6DSL_GYRO_SENSITIVITY_FOR_FS_2000DPS  70.000  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */

#define LSM6DSL_PEDOMETER_THRESHOLD_LOW       0x00  /**< Lowest  value of pedometer threshold */
#define LSM6DSL_PEDOMETER_THRESHOLD_MID_LOW   0x07
#define LSM6DSL_PEDOMETER_THRESHOLD_MID       0x0F
#define LSM6DSL_PEDOMETER_THRESHOLD_MID_HIGH  0x17
#define LSM6DSL_PEDOMETER_THRESHOLD_HIGH      0x1F  /**< Highest value of pedometer threshold */

#define LSM6DSL_WAKE_UP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
#define LSM6DSL_WAKE_UP_THRESHOLD_MID_LOW   0x0F
#define LSM6DSL_WAKE_UP_THRESHOLD_MID       0x1F
#define LSM6DSL_WAKE_UP_THRESHOLD_MID_HIGH  0x2F
#define LSM6DSL_WAKE_UP_THRESHOLD_HIGH      0x3F  /**< Highest value of wake up threshold */

#define LSM6DSL_TAP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_THRESHOLD_MID_LOW   0x08
#define LSM6DSL_TAP_THRESHOLD_MID       0x10
#define LSM6DSL_TAP_THRESHOLD_MID_HIGH  0x18
#define LSM6DSL_TAP_THRESHOLD_HIGH      0x1F  /**< Highest value of wake up threshold */

#define LSM6DSL_TAP_SHOCK_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_SHOCK_TIME_MID_LOW   0x01
#define LSM6DSL_TAP_SHOCK_TIME_MID_HIGH  0x02
#define LSM6DSL_TAP_SHOCK_TIME_HIGH      0x03  /**< Highest value of wake up threshold */

#define LSM6DSL_TAP_QUIET_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_QUIET_TIME_MID_LOW   0x01
#define LSM6DSL_TAP_QUIET_TIME_MID_HIGH  0x02
#define LSM6DSL_TAP_QUIET_TIME_HIGH      0x03  /**< Highest value of wake up threshold */

#define LSM6DSL_TAP_DURATION_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DSL_TAP_DURATION_TIME_MID_LOW   0x04
#define LSM6DSL_TAP_DURATION_TIME_MID       0x08
#define LSM6DSL_TAP_DURATION_TIME_MID_HIGH  0x0C
#define LSM6DSL_TAP_DURATION_TIME_HIGH      0x0F  /**< Highest value of wake up threshold */

/* Typedefs ------------------------------------------------------------------*/

typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
} LSM6DSL_Event_Status_t;

/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an LSM6DSL Inertial Measurement Unit (IMU) 6 axes
 * sensor.
 */
class LSM6DSLSensor : public MotionSensor, public GyroSensor
{
  public:
    LSM6DSLSensor(DevI2C &i2c, PinName INT1_pin, PinName INT2_pin);
    LSM6DSLSensor(DevI2C &i2c, PinName INT1_pin, PinName INT2_pin, uint8_t address);
    virtual int Init(void *init);
    virtual int ReadID(uint8_t *id);
    virtual int Get_X_Axes(int32_t *pData);
    virtual int Get_G_Axes(int32_t *pData);
    virtual int Get_X_Sensitivity(float *pfData);
    virtual int Get_G_Sensitivity(float *pfData);
    virtual int Get_X_AxesRaw(int16_t *pData);
    virtual int Get_G_AxesRaw(int16_t *pData);
    virtual int Get_X_ODR(float *odr);
    virtual int Get_G_ODR(float *odr);
    virtual int Set_X_ODR(float odr);
    virtual int Set_G_ODR(float odr);
    virtual int Get_X_FS(float *fullScale);
    virtual int Get_G_FS(float *fullScale);
    virtual int Set_X_FS(float fullScale);
    virtual int Set_G_FS(float fullScale);
    int Enable_X(void);
    int Enable_G(void);
    int Disable_X(void);
    int Disable_G(void);
    int Enable_Free_Fall_Detection(void);
    int Disable_Free_Fall_Detection(void);
    int Set_Free_Fall_Threshold(uint8_t thr);
    int Enable_Pedometer(void);
    int Disable_Pedometer(void);
    int Get_Step_Counter(uint16_t *step_count);
    int Reset_Step_Counter(void);
    int Set_Pedometer_Threshold(uint8_t thr);
    int Enable_Tilt_Detection(void);
    int Disable_Tilt_Detection(void);
    int Enable_Wake_Up_Detection(void);
    int Disable_Wake_Up_Detection(void);
    int Set_Wake_Up_Threshold(uint8_t thr);
    int Enable_Single_Tap_Detection(void);
    int Disable_Single_Tap_Detection(void);
    int Enable_Double_Tap_Detection(void);
    int Disable_Double_Tap_Detection(void);
    int Set_Tap_Threshold(uint8_t thr);
    int Set_Tap_Shock_Time(uint8_t time);
    int Set_Tap_Quiet_Time(uint8_t time);
    int Set_Tap_Duration_Time(uint8_t time);
    int Enable_6D_Orientation(void);
    int Disable_6D_Orientation(void);
    int Get_6D_Orientation_XL(uint8_t *xl);
    int Get_6D_Orientation_XH(uint8_t *xh);
    int Get_6D_Orientation_YL(uint8_t *yl);
    int Get_6D_Orientation_YH(uint8_t *yh);
    int Get_6D_Orientation_ZL(uint8_t *zl);
    int Get_6D_Orientation_ZH(uint8_t *zh);
    int Get_Event_Status(LSM6DSL_Event_Status_t *status);
    int ReadReg(uint8_t reg, uint8_t *data);
    int WriteReg(uint8_t reg, uint8_t data);
    
    /**
     * @brief  Attaching an interrupt handler to the INT1 interrupt.
     * @param  fptr An interrupt handler.
     * @retval None.
     */
    void AttachINT1IRQ(void (*fptr)(void))
    {
        INT1_irq.rise(fptr);
    }

    /**
     * @brief  Enabling the INT1 interrupt handling.
     * @param  None.
     * @retval None.
     */
    void EnableINT1IRQ(void)
    {
        INT1_irq.enable_irq();
    }
    
    /**
     * @brief  Disabling the INT1 interrupt handling.
     * @param  None.
     * @retval None.
     */
    void DisableINT1IRQ(void)
    {
        INT1_irq.disable_irq();
    }
    
    /**
     * @brief  Attaching an interrupt handler to the INT2 interrupt.
     * @param  fptr An interrupt handler.
     * @retval None.
     */
    void AttachINT2IRQ(void (*fptr)(void))
    {
        INT2_irq.rise(fptr);
    }

    /**
     * @brief  Enabling the INT2 interrupt handling.
     * @param  None.
     * @retval None.
     */
    void EnableINT2IRQ(void)
    {
        INT2_irq.enable_irq();
    }
    
    /**
     * @brief  Disabling the INT2 interrupt handling.
     * @param  None.
     * @retval None.
     */
    void DisableINT2IRQ(void)
    {
        INT2_irq.disable_irq();
    }
    
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
        return (uint8_t) dev_i2c.i2c_read(pBuffer, address, RegisterAddr, NumByteToRead);
    }
    
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
        return (uint8_t) dev_i2c.i2c_write(pBuffer, address, RegisterAddr, NumByteToWrite);
    }

  private:
    int Set_X_ODR_When_Enabled(float odr);
    int Set_G_ODR_When_Enabled(float odr);
    int Set_X_ODR_When_Disabled(float odr);
    int Set_G_ODR_When_Disabled(float odr);

    /* Helper classes. */
    DevI2C &dev_i2c;

    InterruptIn INT1_irq;
    InterruptIn INT2_irq;

    /* Configuration */
    uint8_t address;
    
    uint8_t X_isEnabled;
    float X_Last_ODR;
    uint8_t G_isEnabled;
    float G_Last_ODR;
};

#ifdef __cplusplus
 extern "C" {
#endif
uint8_t LSM6DSL_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t LSM6DSL_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
  }
#endif

#endif
