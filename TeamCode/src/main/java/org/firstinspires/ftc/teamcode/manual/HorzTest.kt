package org.firstinspires.ftc.teamcode.manual

import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.abs

@TeleOp
@Disabled
class HorzTest : LinearOpMode() {
    override fun runOpMode() {
        val horz = hardwareMap.dcMotor["horz"]
        horz.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        horz.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        val pid = PIDController(0.03,0.0,0.001)
        var target = 0.0
        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.a) {
                target = 200.0
            }
            if (gamepad1.b) {
                target = 15.0
            }
            if (abs(horz.currentPosition.toDouble() - target) > 2) {
                horz.power = pid.calculate(horz.currentPosition.toDouble(), target)
            } else {
                horz.power = 0.0
            }
            telemetry.addData("Encoder", horz.currentPosition)
            telemetry.addData("Power", horz.power)
            telemetry.update()
        }
    }

}