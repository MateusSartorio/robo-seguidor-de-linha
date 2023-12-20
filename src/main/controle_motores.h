#pragma once

#include <MPU6050_light.h>

#define BASE_VEL 200
#define SLOW_VEL 180
#define DELTA_VEL 40

/**
 * @brief o robo anda reto no cruzamento
 * @return void
 */
void anda_reto(MPU6050& mpu)
{
  return;
}

/**
 * @brief vira o robo para direita
 * @return void
 */
void vira_direita(MPU6050& mpu)
{
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;

  int delta = angulo_atual - angulo_inicial;

  // Vira ate o angulo variar em -90º
  while (delta > -90)
  {
    mpu.update();
    angulo_atual = mpu.getAngleZ();

    delta = angulo_atual - angulo_inicial;

    ledcWrite(PWM1_Ch, 255 - BASE_VEL);
    ledcWrite(PWM2_Ch, 255 - BASE_VEL);
  }
}

/**
 * @brief vira o robo para esquerda
 * @return void
 */
void vira_esquerda(MPU6050& mpu)
{
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;

  int delta = angulo_atual - angulo_inicial;

  // Vira ate o angulo variar em 90º
  while (delta < 90)
  {
    mpu.update();
    angulo_atual = mpu.getAngleZ();

    delta = angulo_atual - angulo_inicial;

    ledcWrite(PWM1_Ch, BASE_VEL);
    ledcWrite(PWM2_Ch, BASE_VEL);
  }
}

/**
 * @brief vira o robo em 180º
 * @return void
 */
void vira_180(MPU6050& mpu)
{
  mpu.update();
  int angulo_inicial = mpu.getAngleZ();
  int angulo_atual = angulo_inicial;

  int delta = angulo_atual - angulo_inicial;

  // Vira ate o angulo variar em 180º
  while (delta < 180)
  {
    mpu.update();
    angulo_atual = mpu.getAngleZ();

    delta = angulo_atual - angulo_inicial;

    ledcWrite(PWM1_Ch, BASE_VEL);
    ledcWrite(PWM2_Ch, BASE_VEL);
  }
}

void segue_linha(int inputL, int inputR, bool& slow) {
    int deltaL = (int)(((double)inputL / 4000.0) * DELTA_VEL);
    int deltaR = (int)(((double)inputR / 4000.0) * DELTA_VEL);

    int pwmL = 0;
    int pwmR = 0;

    if (slow)
    {
      pwmL = SLOW_VEL - deltaL + deltaR;
      pwmR = SLOW_VEL - deltaR + deltaL;
    }
    else
    {
      pwmL = BASE_VEL - deltaL + deltaR;
      pwmR = BASE_VEL - deltaR + deltaL;
    }

    // motor direito
    ledcWrite(PWM1_Ch, pwmR);

    // motor esquerdo (invertido)
    ledcWrite(PWM2_Ch, 255 - pwmL);
}