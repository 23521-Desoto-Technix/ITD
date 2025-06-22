package org.firstinspires.ftc.teamcode.sensors

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class touchy : LinearOpMode() {
    override fun runOpMode() {
        val backTouch = hardwareMap.touchSensor.get("backTouch")
        val leftTouch = hardwareMap.touchSensor.get("leftTouch")
        val rightTouch = hardwareMap.touchSensor.get("rightTouch")
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("back", backTouch.isPressed)
            telemetry.addData("left", leftTouch.isPressed)
            telemetry.addData("right", rightTouch.isPressed)
            telemetry.update()
        }
    }
}