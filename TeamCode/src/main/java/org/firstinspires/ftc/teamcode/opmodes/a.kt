package org.firstinspires.ftc.teamcode

import com.pedropathing.localization.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin


@TeleOp
class FieldCentricMecanumTeleOp : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        val frontLeft = hardwareMap.dcMotor["frontLeft"]
        val backLeft = hardwareMap.dcMotor["backLeft"]
        val frontRight = hardwareMap.dcMotor["frontRight"]
        val backRight = hardwareMap.dcMotor["backRight"]
        val odo = hardwareMap.get<GoBildaPinpointDriver>(GoBildaPinpointDriver::class.java, "odo")

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE

        odo.resetPosAndIMU();
        waitForStart()

        if (isStopRequested) return

        while (opModeIsActive()) {
            odo.update()
            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick value is reversed
            val x = gamepad1.left_stick_x.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                odo.resetPosAndIMU()
            }

            val botHeading = -odo.heading
            telemetry.addData("heading", botHeading)
            // Rotate the movement direction counter to the bot's rotation
            var rotX = x * cos(-botHeading) - y * sin(-botHeading)
            val rotY = x * sin(-botHeading) + y * cos(-botHeading)

            rotX *= 1.1 // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
            val frontLeftPower = (rotY + rotX + rx) / denominator
            val backLeftPower = (rotY - rotX + rx) / denominator
            val frontRightPower = (rotY - rotX - rx) / denominator
            val backRightPower = (rotY + rotX - rx) / denominator

            frontLeft.power = frontLeftPower
            backLeft.power = backLeftPower
            frontRight.power = frontRightPower
            backRight.power = backRightPower
            telemetry.update()
        }
    }
}