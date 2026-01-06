#include "pid_config.h"

/* --- NASTAWY DLA METODY PRZYROSTOWEJ --- */
// PID oblicza tylko ZMIANĘ (krok), a nie wartość całkowitą.
#define PID_KP  0.1f   // Mały krok dla stabilności (0.1 oznacza: błąd 10 lux -> zmiana PWM o 1%)
#define PID_KI  0.0f   // Zero (całkowanie robi pętla w main)
#define PID_KD  0.0f   // Zero

arm_pid_instance_f32 PID1;

void PID_Config_Init(void)
{
  PID1.Kp = PID_KP;
  PID1.Ki = PID_KI;
  PID1.Kd = PID_KD;

  // Używamy oryginalnej funkcji bibliotecznej, tak jak chciałeś
  arm_pid_init_f32(&PID1, 1);
}
