package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Outtake
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Autonomous(name = "$^&%#&%!^& sample")
class sam : LinearOpMode() {
    override fun runOpMode() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        val follower = Follower(hardwareMap)
        val startPose = Pose(6.0, 114.5, Math.toRadians(0.0))
        val scorePose = Pose(10.0, 131.0, Math.toRadians(0.0))
        follower.setStartingPose(startPose)
        val leftRGB = hardwareMap.servo["LeftRGB"]
        val rightRGB = hardwareMap.servo["RightRGB"]
        leftRGB.position = 0.5
        rightRGB.position = 0.5
        val intake = Intake(hardwareMap)
        val outtake = Outtake(hardwareMap)
        intake.update(Intake.state.IDLE)
        outtake.update(Outtake.state.GRABBED)
        val vslides = VerticalSlides(hardwareMap, telemetry)
        val hslides = HorizontalSlides(hardwareMap, telemetry)
        hslides.setSetpoint(-0_000.0)
        hslides.resetEncoder()
        vslides.resetEncoder()
        val outtakeClaw = hardwareMap.servo["outtakeClaw"]
        val encoder = hardwareMap.dcMotor.get("frontRight")
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        val dig0 = hardwareMap.get(DigitalChannel::class.java, "dig0")
        val dig1 = hardwareMap.get(DigitalChannel::class.java, "dig1")

        val startToScore1 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(startPose),
                    //Point(30.0, 120.0, Point.CARTESIAN),
                    //Point(35.000, 110.000, Point.CARTESIAN),
                    Point(scorePose)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-45.0))
            .build()
        val score1topick = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(scorePose),
                    //Point(30.0, 120.0, Point.CARTESIAN),
                    //Point(35.000, 110.000, Point.CARTESIAN),
                    Point(15.000, 125.000, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-45.0), Math.toRadians(0.0))
            .build()
        outtakeClaw.position = 0.55
        waitForStart()
        leftRGB.position = 0.722
        rightRGB.position = 0.722
        vslides.setSetpoint(0.0)
        follower.followPath(startToScore1, true)
        val timer = ElapsedTime()
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            hslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        vslides.setSetpoint(-103_000.0)
        timer.reset()
        while (!isStopRequested && timer.seconds() < 1.0) {
            vslides.update()
            hslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        timer.reset()
        outtake.update(Outtake.state.TRANSFERED)
        while (!isStopRequested && timer.seconds() < 0.5) {
            vslides.update()
            hslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        outtakeClaw.position = 1.0
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        timer.reset()
        outtake.update(Outtake.state.TRANSFERING)

        vslides.setSetpoint(0.0)
        while (!isStopRequested && timer.seconds() < 0.5) {
            vslides.update()
            hslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        intake.update(Intake.state.SCANNING)
        timer.reset()
        follower.followPath(score1topick, true)
        while (!isStopRequested && follower.isBusy) {
            if (timer.seconds() > 0.2) {
                //hslides.setSetpoint(-23_000.0)
            }
            vslides.update()
            hslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
    }

}