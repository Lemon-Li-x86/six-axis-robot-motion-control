/*
 * Shared pure mathematical constants for model and motion algorithms.
 * These are mathematical unit conversions, NOT runtime configuration.
 * Keep the numerical value in a single definition site; do not create
 * module-specific copies of pi or degree/radian conversion constants.
 */
#ifndef ROBOT_MATH_CONSTANTS_H
#define ROBOT_MATH_CONSTANTS_H

#define ROBOT_PI_F 3.14159265358979323846F
#define ROBOT_TWO_PI_F (2.0F * ROBOT_PI_F)
#define ROBOT_DEG_TO_RAD_F (ROBOT_PI_F / 180.0F)
#define ROBOT_RAD_TO_DEG_F (180.0F / ROBOT_PI_F)

#endif
