/**
 * Exemple d'utilisation de la classe Motor :
 *   Contrôle d'un moteur avec un Driver L298.
 *
 * Pour tester, réaliser les actions suivantes dans 'Device Configuration Tool':
 * - Ajouter des constantes, dans 'User Constants':
 *  - CLOCK_FREQ : 84000000  (fréquence du microcontrôleur utilisé)
 *  - PWM_FREQ : 24000
 *  - PWM_PERIOD : (CLOCK_FREQ / PWM_FREQ - 1)
 * - Configurer un channel d'un Timer en mode 'PWM Generation CH_':
 *  - Définir 'Counter Period' : PWM_PERIOD
 * - Configurer 2 GPIOs en 'GPIO Output' (optionnel: mettre en Pull-Down)
 * - Générer le code
 *
 * Brancher le moteur sur le driver et le driver avec la STM32
 * Enfin, alimenter l'encodeur et lancer le programme (copié-collé de ce fichier dans mainpp.cpp).
 *
 * Exemple de configuration de l'exemple:
 *  - en : Timer 1 - Channel 1 (GPIO PA8)
 *  - in1 : GPIO PB9
 *  - in2 : GPIO PB8
 */

#include "robot/Motor.h"
#include "mainpp.h"
#include "main.h"

// Constantes déjà configurées dans 'Device Configuration Tool':
#define PWM_FREQ 24000
#define PWM_PERIOD (84000000 / PWM_FREQ - 1)

// Motor:
extern TIM_HandleTypeDef htim1;
#define mot_enA { htim1, TIM_CHANNEL_1, PWM_PERIOD, 1.0f, 0.0f }
#define mot_in1 { GPIOB, GPIO_PIN_9 }
#define mot_in2 { GPIOB, GPIO_PIN_8 }
#define mot_def { mot_enA, mot_in1, mot_in2 }
robot::Motor motor(mot_def);

void setup() {
  motor.init();
}

void loop() {
  motor.forward();
  for (int i = 0; i < 10; i++) {
    motor.pwm(i / 10.0f);
    HAL_Delay(1000);
  }

  motor.stop();
  HAL_Delay(2000);

  motor.backward();
  for (int i = 0; i < 10; i++) {
    motor.pwm(i / 10.0f);
    HAL_Delay(1000);
  }

  motor.stop();
  HAL_Delay(2000);
}
