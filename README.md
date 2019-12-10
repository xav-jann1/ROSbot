# Sujet 9 - Galileo ROS

Projet de majeure Robotique en 5ème Année à l'école [CPE Lyon](https://www.cpe.fr/).

**Projet :** Implémenter [`ROS`](https://www.ros.org/) sur un robot constituer uniquement de moteurs et d'encodeurs.

**Matériels utilisés :**
- Petit robot à roues différentielles, avec des encodeurs fixés sur des roues touchant le sol.
- [NUCLEO-STM32F411RE](https://www.st.com/en/evaluation-tools/nucleo-f411re.html) : intéragit avec la partie électronique (moteurs et encodeurs).
- [Raspberry Pi 3B](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/) : exécute ROS et permet le contrôle du robot.

Une communication `UART` existe entre la *STM32* et la *Raspberry Pi* grâce à l'implémentation de [`rosserial`](http://wiki.ros.org/rosserial) sur une *STM32* ([rosserial_stm32f4](https://github.com/xav-jann1/rosserial_stm32f4)).

**Outils utilisés :**
- [`ROS`](https://www.ros.org/)
- [`HAL`](https://www.st.com/content/ccc/resource/technical/document/user_manual/2f/71/ba/b8/75/54/47/cf/DM00105879.pdf/files/DM00105879.pdf/jcr:content/translations/en.DM00105879.pdf) : *Hardware Abstraction Layer* pour STM32
- [`STM32CubeIDE`](https://www.st.com/en/development-tools/stm32cubeide.html)


## Contributeur

- Xavier Jannin
