package org.firstinspires.ftc.teamcode.manual

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel
import kotlin.jvm.java

@TeleOp(name = "Limelight Light", group = "manual")
class limelightlight: LinearOpMode() {
    override fun runOpMode() {
        val ll2 = hardwareMap.get(DigitalChannel::class.java, "limelightlight")
        ll2.mode = DigitalChannel.Mode.OUTPUT
        waitForStart()
        while (opModeIsActive()) {
            if (gamepad1.a) {
                ll2.state = true
            } else {
                ll2.state = false
            }
        }
    }

}