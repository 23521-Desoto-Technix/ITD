package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.PID

internal class HorizontalSlides(hwmap: HardwareMap, telem: Telemetry) {
    var horz = hwmap.dcMotor.get("horz")
    var encoder = hwmap.dcMotor.get("frontRight")
    val telem = telem
    var pid= PID(0.0005, 0.0, 0.000002)
    init {
        horz.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        pid.setSetpoint(-23_000.0)
        //encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        //encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
    //intake encoder is on 0

    fun update() {
        horz.power = -pid.calculate(encoder.currentPosition.toDouble())
        //telem.addData("Encoder Horz", encoder.currentPosition)
    }
    fun setPower(power: Double) {
        horz.power = power
    }
    fun setSetpoint(setpoint: Double) {
        pid.setSetpoint(setpoint)
    }
    fun getSetpoint(): Double {
        return pid.getSetpoint()
    }
    fun resetEncoder() {
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}