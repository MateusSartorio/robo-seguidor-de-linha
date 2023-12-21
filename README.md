# robo-seguidor-de-linha
Robô seguidor de linha capaz de navegar e solucionar labirintos.

O projeto buscou integrar os aprendizados teóricos e práticos adquiridos ao longo do curso de Sistemas Embarcados I para implementar um robô capaz de seguir uma trajetória delineada por linhas pretas, utilizando sensores e algoritmos para navegação autônoma, de forma que fosse capaz de orientar-se e encontrar a melhor trajetória até a saída.
O solução proposta basea-se no algoritmo de Dijkstra, onde o robô pressupõem que não existe nenhum obstáculo inicialmente, e a medida que ele percorre o labirinto e encontra os obstáculos, a rota é ajustanda a de forma a encontrar o menor caminho possível até a saída com as informações que o robô tem sobre o labirinto até o momento. A documentação completa do projeto está em Documentacao.pdf.

Trabalho da disciplina de Sistemas Embarcados I - Universidade federal do Espírito Santo.

## Vídeo demonstrativo no [YouTube](https://youtu.be/42od_47x8S8).

## O Robô
Abaixo é mostrado o robô utilizado para implementar o algorítimo proposto para solução do labirinto.
![robot](https://github.com/MateusSartorio/robo-seguidor-de-linha/assets/69646100/72749fd0-925d-4d86-a08f-0f7dff8bf461)

## O Labirinto
O labirinto consiste de linhas pretas formando um grid, com obstáculos bloqueando alguns dos caminhos.
![labirinto](https://github.com/MateusSartorio/robo-seguidor-de-linha/assets/69646100/2c11934c-82dd-4713-8143-908e6f4abb90)

## Estrutura do robô
A figura abaixo apresenta a estrutura principal do robô. Ele conta com uma tração diferencial (motor esquerdo e direito, ME e MD, respetivamente), uma roda boba de suporte, e de 3 a 5 sensores de linha (sensores de linha esquerdo, direito e de cruzamento esquerdo, SLE, SLD e SCE, respetivamente, podendo acrescentar sensor de linha central e de cruzamento direito).
![estrutura_do_robo](https://github.com/MateusSartorio/robo-seguidor-de-linha/assets/69646100/1db27bba-5684-4c7f-9526-077784c4664b)

## Esquemático
A organização dos componentes utilizados está disposta na figura abaixo, que representa o esquemático do robô.
![esquematico](https://github.com/MateusSartorio/robo-seguidor-de-linha/assets/69646100/f67462cf-89a0-449b-9f9b-c26d680aee70)

## Licença:

Copyright Universidade Federal do Espirito Santo (Ufes)

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program.  If not, see <https://www.gnu.org/licenses/>.

This program is released under license GNU GPL v3+ license.

## Suporte:

Por favor reporte qualquer problema com o jogo da velha em [github.com/MateusSartorio/robo-seguidor-de-linha](https://github.com/MateusSartorio/robo-seguidor-de-linha).
