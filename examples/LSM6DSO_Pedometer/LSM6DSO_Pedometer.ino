/**
 ******************************************************************************
 * @file    LSM6DSO_Pedometer.ino
 * @author  SRA
 * @version V1.0.0
 * @date    February 2024
 * @brief   Arduino test application for the STMicrolectronics
 *          LSM6DSO MEMS IMU 6-axis sensor.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2024 STMicroelectronics</center></h2>
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
//NOTE: this example isn't compatible with Arduino Uno


#include <LSM6DSOSensor.h>

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

#define INT_1 4

LSM6DSOSensor accGyr(&DEV_I2C);

//Interrupts.
volatile int mems_event = 0;

uint16_t step_count = 0;
char report[256];
uint32_t previous_tick;

void INT1Event_cb();

void setup() {
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();
  
  //Interrupts.
  attachInterrupt(INT_1, INT1Event_cb, RISING);

  accGyr.begin();
  accGyr.Enable_X();
  accGyr.Enable_Pedometer();

  previous_tick = millis();
}

void loop() {
  if (mems_event)
  {
    mems_event=0;
    LSM6DSO_Event_Status_t status;
    accGyr.Get_X_Event_Status(&status);
    if (status.StepStatus)
    {
      // New step detected, so print the step counter
      accGyr.Get_Step_Count(&step_count);
      snprintf(report, sizeof(report), "Step counter: %d", step_count);
      SerialPort.println(report);

      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  // Print the step counter in any case every 3000 ms
  uint32_t current_tick = millis();
  if((current_tick - previous_tick) >= 3000)
  {
    accGyr.Get_Step_Count(&step_count);
    snprintf(report, sizeof(report), "Step counter: %d", step_count);
    SerialPort.println(report);
    previous_tick = millis();
  }
  
}

void INT1Event_cb()
{
  mems_event = 1;
}
