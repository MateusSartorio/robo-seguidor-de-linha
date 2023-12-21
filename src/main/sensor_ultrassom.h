#pragma once

#define DISTANCIA_OBSTACULO 22
#define QTD_MEDIDAS_ULTRASOM 3
#define SOUND_SPEED 0.034

/**
 * @brief Le a distancia de objetos em relacao ao sensor ultrasom
 * @return double
 * @retval distancia em cm
 */
double le_distancia()
{
  long durations[QTD_MEDIDAS_ULTRASOM] = {0};

  // Le o tempo de percurso da onda 3 vezes
  for (int i = 0; i < QTD_MEDIDAS_ULTRASOM; i++)
  {
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(2);
    // Seta o EN_TRIG em HIGH por 10 microssegundos
    digitalWrite(EN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(EN_TRIG, LOW);

    // Le o MISO_ECHO, returna o tempo de percurso em microssegundos
    durations[i] = pulseIn(MISO_ECHO, HIGH);
  }

  // Calcula a media dos tempos
  long soma = 0;
  for (int i = 0; i < QTD_MEDIDAS_ULTRASOM; i++)
  {
    soma += durations[i];
  }

  double media = (double)soma / (double)QTD_MEDIDAS_ULTRASOM;

  // Calcula a distancia em cm a partir do tempo
  double distanceCm = media * SOUND_SPEED / 2.0;

  return distanceCm;
}