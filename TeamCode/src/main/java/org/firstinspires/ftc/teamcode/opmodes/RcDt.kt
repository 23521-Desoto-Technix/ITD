package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.max

@TeleOp
class MecanumTeleOp : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        val frontLeftMotor = hardwareMap.dcMotor["frontLeft"]
        val backLeftMotor = hardwareMap.dcMotor["backLeft"]
        val frontRightMotor = hardwareMap.dcMotor["frontRight"]
        val backRightMotor = hardwareMap.dcMotor["backRight"]

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.direction = DcMotorSimple.Direction.REVERSE
        backRightMotor.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        if (isStopRequested) return

        while (opModeIsActive()) {
            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick value is reversed
            val x = gamepad1.left_stick_x * 1.1 // Counteract imperfect strafing
            val rx = gamepad1.right_stick_x.toDouble()

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            val denominator = max(abs(y) + abs(x) + abs(rx), 1.0)
            val frontLeftPower = (y + x + rx) / denominator
            val backLeftPower = (y - x + rx) / denominator
            val frontRightPower = (y - x - rx) / denominator
            val backRightPower = (y + x - rx) / denominator

            frontLeftMotor.power = frontLeftPower
            backLeftMotor.power = backLeftPower
            frontRightMotor.power = frontRightPower
            backRightMotor.power = backRightPower
        }
    }
}