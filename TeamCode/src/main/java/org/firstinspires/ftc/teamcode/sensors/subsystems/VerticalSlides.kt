package org.firstinspires.ftc.teamcode.sensors.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.PID
import kotlin.math.abs

class VerticalSlides(hwmap: HardwareMap, telem: Telemetry) {
    val v0 = hwmap.dcMotor["vertical0"]
    val v1 = hwmap.dcMotor["vertical1"]
    val telem = telem
    val encoder = hwmap.dcMotor["backRight"]
    val pid = PID(0.001, 0.0, 0.0)
    var FF = 0.1
    init {
        v0.direction = DcMotorSimple.Direction.REVERSE
        v0.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        v1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        v0.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        v1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun update() {
        telem.addData("Encoder Vertical", encoder.currentPosition)
        telem.addData("Error", abs(pid.getSetpoint() - encoder.currentPosition))
        if (abs(pid.getSetpoint() - encoder.currentPosition) < 1000) {
            v0.power = -FF
            v1.power = -FF
        } else {
            val power = pid.calculate(encoder.currentPosition.toDouble())
            v0.power = power
            v1.power = power
        }
    }
    fun setPower(power: Double) {
        v0.power = power - FF
        v1.power = power - FF
    }
    fun setSetpoint(setpoint: Double) {
        pid.setSetpoint(setpoint)
    }
    fun resetEncoder() {
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}