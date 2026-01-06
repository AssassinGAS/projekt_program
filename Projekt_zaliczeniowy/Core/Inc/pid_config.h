#ifndef INC_PID_CONFIG_H_
#define INC_PID_CONFIG_H_

#include "main.h"
#include "arm_math.h" // Biblioteka CMSIS DSP

// Udostępniamy instancję PID na zewnątrz
extern arm_pid_instance_f32 PID1;

// Funkcja inicjalizująca
void PID_Config_Init(void);

#endif /* INC_PID_CONFIG_H_ */
