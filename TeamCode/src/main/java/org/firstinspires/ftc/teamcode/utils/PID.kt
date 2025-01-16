package org.firstinspires.ftc.teamcode.utils

class PID(
    private val kp: Double,
    private val ki: Double,
    private val kd: Double
) {
    private var setpoint: Double = 0.0
    private var integral: Double = 0.0
    private var previousError: Double = 0.0
    private var lastTime: Long = System.currentTimeMillis()

    fun setSetpoint(setpoint: Double) {
        this.setpoint = setpoint
        integral = 0.0
        previousError = 0.0
        lastTime = System.currentTimeMillis()
    }

    fun getSetpoint(): Double {
        return setpoint
    }

    fun calculate(measurement: Double): Double {
        val currentTime = System.currentTimeMillis()
        val timeChange = (currentTime - lastTime) / 1000.0
        val error = setpoint - measurement
        integral += error * timeChange
        val derivative = (error - previousError) / timeChange

        val output = kp * error + ki * integral + kd * derivative

        previousError = error
        lastTime = currentTime

        return output
    }
}