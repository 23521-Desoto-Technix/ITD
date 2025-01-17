package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.sensors.subsystems.Intake
import org.firstinspires.ftc.teamcode.sensors.subsystems.Outtake
import org.firstinspires.ftc.teamcode.sensors.subsystems.VerticalSlides
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Autonomous
class sampleAuto : LinearOpMode() {
    override fun runOpMode() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        val follower = Follower(hardwareMap)
        val startPose = Pose(6.0, 66.0, Math.toRadians(0.0))
        val scorePose = Pose(38.5, 66.0, Math.toRadians(0.0))
        val pickupPose = Pose(10.0, 25.0, Math.toRadians(0.0))
        follower.setStartingPose(startPose)

        val intake = Intake(hardwareMap)
        val outtake = Outtake(hardwareMap)
        intake.update(Intake.state.IDLE)
        outtake.update(Outtake.state.GRABBED)
        val vslides = VerticalSlides(hardwareMap, telemetry)
        val claw = hardwareMap.servo["outtakeClaw"]

        val startToScore = follower.pathBuilder()
            .addPath( // Line 1
                BezierLine(
                    Point(startPose),
                    Point(scorePose)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
            /*
            .addPath( // Line 2
                BezierCurve(
                    Point(40.000, 66.000, Point.CARTESIAN),
                    Point(12.000, 40.000, Point.CARTESIAN),
                    Point(128.000, 15.000, Point.CARTESIAN),
                    Point(10.000, 25.000, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .addPath( // Line 3
                BezierCurve(
                    Point(10.000, 25.000, Point.CARTESIAN),
                    Point(77.000, 40.000, Point.CARTESIAN),
                    Point(85.000, -5.000, Point.CARTESIAN),
                    Point(10.000, 25.000, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))*/
        val push1 = follower.pathBuilder()
            .addPath( // Line 2
                BezierCurve(
                    Point(scorePose),
                    Point(12.000, 40.000, Point.CARTESIAN),
                    Point(128.000, 15.000, Point.CARTESIAN),
                    Point(pickupPose)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        claw.position = 0.58
        waitForStart()
        follower.followPath(startToScore)
        vslides.setSetpoint(-36_000.0)
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
        follower.followPath(push1)
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            follower.update()
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.update()
        }
    }

}