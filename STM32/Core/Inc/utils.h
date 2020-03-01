#ifndef ROBOT_H_
#define ROBOT_H_

#define M_2PI 6.28318530718f
#define Init_Error_Handler() while (1);

// LED:
#define LED_OFF()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define LED_ON()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define LED_TOGGLE() HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5)


#endif /* ROBOT_H_ */
