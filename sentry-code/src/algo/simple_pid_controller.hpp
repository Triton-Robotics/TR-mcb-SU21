//
// Created by ariel on 10/4/21.
//

#ifndef SENTRY_SIMPLE_PID_CONTROLLER_HPP
#define SENTRY_SIMPLE_PID_CONTROLLER_HPP

#include <concepts>
#include <cstdint>
#include <cmath>

namespace tr::algo {
    template<typename V>
    concept arithmetic = std::integral<V> or std::floating_point<V>;

    /**
     * SimplePIDController is a generic PID controller class with tunable constant. It is capable of operating on any
     * numeric type using templating.
     * @tparam T The type to use for error and output value. Must be numeric.
     */
    template <typename T> requires arithmetic<T>
    class SimplePIDController {
    public:
        /**
         * Initialize PID controller class
         * @param kp Proportional constant
         * @param ki Integral constant
         * @param kd Derivative constant
         */
        SimplePIDController(float kp, float ki, float kd) :
            kp(kp), ki(ki), kd(kd), prev_error(0), p(0), i(0), d(0), output(0) {};

        /**
         * Compute the next iteration of the pid controller
         * @param error The error for time interval `dt`
         * @param dt The time elapsed since the previous iteration
         * @return The next output for the controlled system
         */
        T iteratePidController(T error, uint32_t dt) {
            p = kp * (float) error;
            i += ki * (float) error * (float) dt;
            d = kd / (float) dt * (error - prev_error);

            prev_error = error;
            output = p + i + d;

            if constexpr (std::is_floating_point<T>::value) {
                return output;
            } else {
                return static_cast<T>(std::round(output));
            }
        }

        /**
         * Resets the PID controller, zeroing out all terms
         */
        void reset() {
            prev_error = p = i = d = 0; // chained equality is satisfying
        }

    private:
        // god have mercy
        constexpr T clamp(T val, T min, T max) { return (val < min ? min : val) > max ? max : val; }

        const float kp;
        const float ki;
        const float kd;

        T prev_error;
        float p;
        float i;
        float d;
        float output;
    };
}

#endif //SENTRY_SIMPLE_PID_CONTROLLER_HPP
