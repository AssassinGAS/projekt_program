#include "pid_config.h"

#define PID_KP  0.05f
#define PID_KI  0.05f
#define PID_KD  0.01f

arm_pid_instance_f32 PID1;

void PID_Config_Init(void)
{
  PID1.Kp = PID_KP;
  PID1.Ki = PID_KI;
  PID1.Kd = PID_KD;

  // Inicjalizacja: reset stanu wewnętrznego (zerowanie całki)
  arm_pid_init_f32(&PID1, 1);
}
