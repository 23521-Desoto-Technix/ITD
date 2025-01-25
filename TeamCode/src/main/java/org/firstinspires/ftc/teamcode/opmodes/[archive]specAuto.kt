package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Outtake
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Autonomous
@Disabled
class ARCHIVEDspecAuto : LinearOpMode() {
    override fun runOpMode() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        val follower = Follower(hardwareMap)
        val startPose = Pose(6.0, 66.0, Math.toRadians(0.0))
        val scorePose = Pose(39.0, 66.0, Math.toRadians(0.0))
        val pickupPose = Pose(9.5, 13.0, Math.toRadians(0.0))
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
        val claw = hardwareMap.servo["outtakeClaw"]
        val encoder = hardwareMap.dcMotor.get("frontRight")
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        claw.position = 0.55
        val dig0 = hardwareMap.get(DigitalChannel::class.java, "dig0")
        val dig1 = hardwareMap.get(DigitalChannel::class.java, "dig1")

        val startToScore = follower.pathBuilder()
            .addPath( // Line 1
                BezierLine(
                    Point(startPose),
                    Point(scorePose)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val push1 = follower.pathBuilder()
            .addPath( // Line 2
                BezierCurve(
                    Point(scorePose),
                    Point(0.000, 35.000, Point.CARTESIAN),
                    Point(75.000, 35.000, Point.CARTESIAN),
                    Point(55.0, 25.0, Point.CARTESIAN),
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .addPath(
                    BezierLine(
                        Point(55.0, 35.0, Point.CARTESIAN),
                        Point(15.0, 25.0)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val push2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(15.000, 25.000, Point.CARTESIAN),
                    Point(55.000, 25.000, Point.CARTESIAN),
                    Point(55.000, 13.000, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .addPath(
                BezierCurve(
                    Point(55.000, 13.000, Point.CARTESIAN),
                    Point(pickupPose),
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val score1 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pickupPose),
                    Point(11.5, 70.0, Point.CARTESIAN),
                    Point(37.5, 70.0, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val backToHP = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(37.500, 70.000, Point.CARTESIAN),
                    Point(10.000, 70.000, Point.CARTESIAN),
                    Point(70.000, 13.000, Point.CARTESIAN),
                    Point(pickupPose)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val backToHPFinal = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(37.500, 70.000, Point.CARTESIAN),
                    Point(10.000, 70.000, Point.CARTESIAN),
                    Point(70.000, 13.000, Point.CARTESIAN),
                    Point(12.0,13.0, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val score2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pickupPose),
                    Point(9.0, 74.0, Point.CARTESIAN),
                    Point(37.5, 74.0, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val score3 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pickupPose),
                    Point(9.0, 78.0, Point.CARTESIAN),
                    Point(37.5, 78.0, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()

        waitForStart()
        leftRGB.position = 0.722
        rightRGB.position = 0.722
        follower.followPath(startToScore, true)
        vslides.setSetpoint(-33_000.0)
        val timer = ElapsedTime()

        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        timer.reset()
        vslides.setSetpoint(-60_000.0)
        while (timer.seconds() < 0.5) {
            vslides.update()
        }
        claw.position = 0.85
        vslides.setSetpoint(-0.0)
        follower.followPath(push1, true)
        outtake.update(Outtake.state.INTAKING)
        timer.reset()
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        follower.followPath(push2, true)
        timer.reset()
        while (!isStopRequested && follower.isBusy && (!dig0.state && !dig1.state)) {
            if (timer.seconds() > 1.0) {
                outtake.update(Outtake.state.INTAKING)
            }
            vslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        claw.position = 0.55
        timer.reset()
        while (timer.seconds() < 0.1) {
            vslides.update()
        }
        vslides.setSetpoint(-33_000.0)
        while (timer.seconds() < 0.3) {
            vslides.update()
        }
        outtake.update(Outtake.state.GRABBED)
        follower.followPath(score1, true)
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        timer.reset()
        vslides.setSetpoint(-60_000.0)
        while (timer.seconds() < 0.5) {
            vslides.update()
        }
        outtake.update(Outtake.state.INTAKING)
        claw.position = 0.85
        vslides.setSetpoint(-0.0)
        follower.followPath(backToHP, true)
        while (!isStopRequested && follower.isBusy && ((!dig0.state && !dig1.state) || (timer.seconds() < 1.0))) {
            vslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        claw.position = 0.55
        timer.reset()
        while (timer.seconds() < 0.1) {
            vslides.update()
        }
        vslides.setSetpoint(-33_000.0)
        while (timer.seconds() < 0.3) {
            vslides.update()
        }
        outtake.update(Outtake.state.GRABBED)
        follower.followPath(score2, true)
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        timer.reset()
        vslides.setSetpoint(-60_000.0)
        while (timer.seconds() < 0.5) {
            vslides.update()
        }
        outtake.update(Outtake.state.INTAKING)
        claw.position = 0.85
        vslides.setSetpoint(-0.0)
        follower.followPath(backToHP, true)
        //TODO 4 (add the sensor thingy)
        follower.followPath(backToHP, true)
        timer.reset()
        while (!isStopRequested && follower.isBusy && ((!dig0.state && !dig1.state) || (timer.seconds() < 1.0))) {
            vslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        claw.position = 0.55
        timer.reset()
        while (timer.seconds() < 0.1) {
            vslides.update()
        }
        vslides.setSetpoint(-33_000.0)
        while (timer.seconds() < 0.3) {
            vslides.update()
        }
        outtake.update(Outtake.state.GRABBED)
        follower.followPath(score3, true)
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
        timer.reset()
        vslides.setSetpoint(-60_000.0)
        while (timer.seconds() < 0.5) {
            vslides.update()
        }
        outtake.update(Outtake.state.INTAKING)
        claw.position = 0.85
        vslides.setSetpoint(-0.0)
        while (timer.seconds() < 1.5) {
            vslides.update()
        }
    }

}