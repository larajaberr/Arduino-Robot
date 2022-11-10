#ifndef _LIBROBOT_H
#define _LIBROBOT_H

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define OCR0_TOP   156 /* OCR1 value corresponding to miniumum position of arm motors */
#define OCR0_CLOSE  16 /* Gain value to transform maximum motor command to OCR1 value
						  corresponding to maxiumum position of arm motors */
#define OCR0_OPEN    8 /* OCR1 value corresponding to miniumum position of arm motors */

#define OCR1_GAIN 2000 /* Gain value to transform maximum motor command to OCR1 value
						  corresponding to maxiumum position of arm motors */
#define OCR1_MIN  2000 /* OCR1 value corresponding to miniumum position of arm motors */

#include <avr/io.h>

/*  ---------------------- DO NOT MODIFY ABOVE THIS LINE! ---------------------- */

/* Functions to be implemented as part of the assignment. */

/* Function to configure the development board to use the button as a sensor */
void init_button_sensor(void);

/* Function to sense when the button is pressed */
int read_button_sensor(void);

/* Function to read the relative position of the control knobs between their maximum and minimum values */
float read_knob_sensor(int knob);

/* Function to configure the development board to generate a PWM control signal for the main arm motors */
void init_arm_motors(void);


/*  ---------------------- DO NOT MODIFY BELOW THIS LINE! ---------------------- */

/* Function to configure the development board to use the control knobs as sensors */
void init_knob_sensors(void);

/* Function to configure the development board to generate a PWM control signal for the gripper motor */
void init_gripper_motor(void);

/* Function to send a PWM control signal to the gripper motor */
void write_gripper_motor(int open);

/* Function to send a PWM control signal to the main arm motors */
void write_arm_motors(float motor0, float motor1);

#endif
