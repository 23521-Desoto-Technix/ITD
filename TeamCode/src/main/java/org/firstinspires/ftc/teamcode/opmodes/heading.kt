package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class heading : LinearOpMode() {
    override fun runOpMode() {
        telemetry.addData("Heading", dataStorage.angle)
        telemetry.update()
        waitForStart()
    }
}