package org.firstinspires.ftc.teamcode.sensors

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class touchy : LinearOpMode() {
    override fun runOpMode() {
        val touch = hardwareMap.touchSensor.get("backTouch")
        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("Touch Sensor", touch.isPressed)
            telemetry.update()
        }
    }
}