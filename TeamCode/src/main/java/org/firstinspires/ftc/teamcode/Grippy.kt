package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.ServoSwap

@TeleOp
@Disabled
class Grippy : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val claw1 = hardwareMap.servo["claw"]
        val claw2 = hardwareMap.servo["claw2"]
        val wrist = hardwareMap.servo["wrist"]
        val c1ss = ServoSwap(claw1, 0.32, 0.75)
        val c2ss = ServoSwap(claw2, 0.32, 0.65)
        waitForStart()
        while (opModeIsActive()) {
            c2ss.update(gamepad1.a)
            c1ss.update(gamepad1.b)
            wrist.position = ((gamepad1.left_stick_x + 1) / 2).toDouble()
        }
    }
}
