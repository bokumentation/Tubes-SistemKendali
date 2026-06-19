#pragma once

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compute the sum of two integers
 *
 * @param a first operand
 * @param b second operand
 * @return sum of a and b
 */
int example_compute_sum(int a, int b);

/**
 * @brief Compute the nth Fibonacci number (recursive)
 *
 * F(0) = 0, F(1) = 1, F(n) = F(n-1) + F(n-2)
 *
 * @param n position in the Fibonacci sequence (non-negative)
 * @return nth Fibonacci number, or 0 if n <= 0
 */
int example_fibonacci(int n);

/**
 * @brief Compute the factorial of n (recursive)
 *
 * n! = n * (n-1) * (n-2) * ... * 1, with 0! = 1
 *
 * @param n non-negative integer
 * @return factorial of n, or 1 if n <= 0
 */
int example_factorial(int n);

/**
 * @brief Configure a GPIO pin as an output for the LED
 *
 * Resets and sets the direction of the specified GPIO to output mode.
 *
 * @param gpio GPIO number to configure as LED output
 */
void example_configure_led(gpio_num_t gpio);

/**
 * @brief Blink the LED once (on for 500ms, off for 500ms)
 *
 * Assumes the specified GPIO has already been configured via example_configure_led().
 *
 * @param gpio GPIO number connected to the LED
 */
void example_blink_led(gpio_num_t gpio);

#ifdef __cplusplus
}
#endif
