package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystems.Scoring
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin


@TeleOp
class TeleopV0 : LinearOpMode() {

    val DT_ACTIVE = true

    override fun runOpMode() {
        val allHubs : List<LynxModule> = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        val scoring = Scoring(hardwareMap, telemetry, gamepad1, gamepad2)
        val backLeft = CachingDcMotorEx(hardwareMap.dcMotor["backLeft"] as DcMotorEx)
        val backRight = CachingDcMotorEx(hardwareMap.dcMotor["backRight"] as DcMotorEx)
        val frontLeft = CachingDcMotorEx(hardwareMap.dcMotor["frontLeft"] as DcMotorEx)
        val frontRight = CachingDcMotorEx(hardwareMap.dcMotor["frontRight"] as DcMotorEx)

        val beginPose = Pose2d(0.0, 0.0, 0.0)
        val drive = SparkFunOTOSDrive(hardwareMap, beginPose)

        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE


        val horz = hardwareMap.dcMotor["horz"]
        horz.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        horz.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        var lastTime = 0.0

        waitForStart()
        while (opModeIsActive()) {
            scoring.update()
            drive.updatePoseEstimate()
            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick value is reversed
            val x = -gamepad1.left_stick_x * 1.1 // Counteract imperfect strafing
            val rx = -gamepad1.right_stick_x.toDouble()
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]

            val botHeading = -drive.pose.heading.toDouble()

            val rotX = x * cos(-botHeading) - y * sin(-botHeading)
            val rotY = x * sin(-botHeading) + y * cos(-botHeading)

            val denominator: Double = Math.max(abs(rotX) + abs(rotY) + abs(rx), 1.0)
            val mult: Double
            if (gamepad1.right_bumper) {
                mult = 0.5
            } else {
                mult = 1.0
            }
            if (DT_ACTIVE) {
                frontLeft.power = (rotY + rotX + rx) / denominator * mult
                backLeft.power = (rotY - rotX + rx) / denominator * mult
                frontRight.power = (rotY - rotX - rx) / denominator * mult
                backRight.power = (rotY + rotX - rx) / denominator * mult
            } else {
                frontLeft.power = 0.0
                backLeft.power = 0.0
                frontRight.power = 0.0
                backRight.power = 0.0
            }
            if (gamepad1.back) {
                drive.pose = Pose2d(0.0, 0.0, 0.0)
            }

            telemetry.addData("Time", System.currentTimeMillis() - lastTime)
            telemetry.update()
            lastTime = System.currentTimeMillis().toDouble()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.addData("Botheading", botHeading)
        }
    }

}
