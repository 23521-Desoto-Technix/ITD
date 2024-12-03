package org.firstinspires.ftc.teamcode.manual

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class ServoTest : LinearOpMode() {
    override fun runOpMode() {
        val servo0 = hardwareMap.servo.get("clawO")
        val servo1 = hardwareMap.servo.get("clawI")
        waitForStart()
        while (opModeIsActive()) {
            servo0.position = (gamepad1.left_stick_y + 1.0) / 2
            servo1.position = (gamepad1.right_stick_y + 1.0) / 2
            telemetry.addData("Servo 0 Position", servo0.position)
            telemetry.addData("Servo 1 Position", servo1.position)
            telemetry.update()
        }
    }
}