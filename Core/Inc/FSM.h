/**
 * @file FSM.h
 * @brief Finite State Machine for the flight controller
 *
 * @author [Nate Hunter]
 * @date [07.05.2025]
 * @version 1.0
 */

#ifndef FSM_H
#define FSM_H

/**
 * @brief Initialize the Finite State Machine
 * @note This should be called once at system startup
 */
void FSM_Init(void);

/**
 * @brief Execute one cycle of the Finite State Machine
 * @note This should be called repeatedly in the main loop
 */
void FSM_Update(void);

#endif /* FSM_H */
