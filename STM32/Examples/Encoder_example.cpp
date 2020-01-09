/**
 * Exemple d'utilisation de la classe Encoder :
 *   Détermine la position et la vitesse d'un encodeur.
 *
 * Pour tester, réaliser les actions suivantes dans 'Device Configuration Tool':
 * - Ajouter des constantes, dans 'User Constants':
 *  - ENCODER_PERIOD : 0xFFFF ou 0xFFFFFFFF (Timer 16 ou 32 bits)
 *  - ENCODER_RELOAD : (ENCODER_PERIOD / 2)
 *  - ENCODER_RESOLUTION : 64  (optionnel)
 *  - ENCODER_LOOP_MS : 200
 * - Configurer un Timer:
 *  - Définir 'Combined Channels' : 'Encoder Mode'
 *  - Définir 'Counter Period' : ENCODER_RELOAD
 *  - Définir 'Encoder Mode' : 'Encoder Mode TI1 and TI2'
 * - Si besoin, configurer les GPIOs du Timer en Pull-Up ou Pull-Down.
 * - Générer le code
 *
 * Brancher les sorties de l'encodeur sur les pins d'entrées du Timer 3.
 * Enfin, alimenter l'encodeur et lancer le programme (copié-collé de ce fichier dans mainpp.cpp).
 *
 * Exemple de configuration de l'exemple:
 * - Timer 3 en Mode Encoder, avec les GPIOs par défaut :
 *   - PA6 : TIM3_CH1
 *   - PA7 : TIM3_CH2
 */

#include "mainpp.h"
#include "main.h"
#include "robot_Encoder.h"
#include <stdio.h>  // printf: "Use float with printf ..." doit être activé
                    //          dans Project / Properties / C/C++ Build / Settings / Tool Settings

// Constantes déjà configurées dans 'Device Configuration Tool':
#define ENCODER_PERIOD 0xFFFF
#define ENCODER_RELOAD (ENCODER_PERIOD / 2)
#define ENCODER_RESOLUTION 64
#define ENCODER_LOOP_MS 200

extern TIM_HandleTypeDef htim3;
#define enc_def { htim3, ENCODER_RESOLUTION, ENCODER_RELOAD }
robot::Encoder encoder(enc_def);

void setup() {
  encoder.init();
}

float dt = ENCODER_LOOP_MS / 1000.0f;
void loop() {
  encoder.update(dt);

  float pos = encoder.getPosition();
  float vel = encoder.getVelocity();
  int dticks = encoder.getDticks();
  printf("pos: %.2f, vel: %.2f, dticks: %d\r\n", pos, vel, dticks);

  HAL_Delay(ENCODER_LOOP_MS);
}
