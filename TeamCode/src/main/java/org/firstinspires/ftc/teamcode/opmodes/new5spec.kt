package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.pathgen.Point
import com.pedropathing.util.Constants
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Outtake
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Autonomous(name = "new 5 specimen")
class new5spec : LinearOpMode() {
    override fun runOpMode() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        val follower = Follower(hardwareMap)
        val startPose = Pose(6.0, 66.0, Math.toRadians(0.0))
        val scorePose = Pose(42.0, 66.0, Math.toRadians(0.0))
        val pickupPose = Pose(06.0, 8.0, Math.toRadians(0.0))
        val pickupPose2 = Pose(08.0, 30.0, Math.toRadians(0.0))
        val telemetryA: Telemetry =
            MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
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
        hslides.resetEncoder()
        vslides.resetEncoder()
        hslides.setSetpoint(-1_000.0)
        val outtakeClaw = hardwareMap.servo["outtakeClaw"]
        val intakeClaw = hardwareMap.servo["intakeClaw"]
        outtakeClaw.position = 0.8
        intakeClaw.position = 0.68
        val dig0 = hardwareMap.get(DigitalChannel::class.java, "dig0")
        val dig1 = hardwareMap.get(DigitalChannel::class.java, "dig1")
        val allHubs = hardwareMap.getAll(
            LynxModule::class.java
        )
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        val startToScore = follower.pathBuilder()
            .addPath( // Line 1
                BezierLine(
                    Point(startPose),
                    Point(scorePose)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val pick0 = follower.pathBuilder()
            .addPath( // Line 1
                BezierCurve(
                    Point(scorePose),
                    Point(Pose(21.0, 66.0)),
                    Point(Pose(21.0, 25.0))
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val pick1 = follower.pathBuilder()
            .addPath( // Line 1
                BezierLine(
                    Point(Pose(21.0, 24.5)),
                    Point(Pose(21.0, 14.0))
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        waitForStart()
        leftRGB.position = 0.722
        rightRGB.position = 0.722
        follower.followPath(startToScore, 0.6, true)
        vslides.setSetpoint(-50_000.0)
        val timer = ElapsedTime()
        val hertz = ElapsedTime()
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.addData("Hertz", 1.0 / hertz.seconds())
            hertz.reset()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
            if (follower.currentTValue > 0.95) {
                break
            }
        }
        vslides.setSetpoint(0.0)
        outtakeClaw.position = 1.0
        timer.reset()
        follower.followPath(pick0, true)
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
            //telemetry.addData("X", follower.pose.x)
            //telemetry.addData("Y", follower.pose.y)
            //telemetry.addData("Heading", follower.pose.heading)
            telemetry.addData("Hertz", 1.0 / hertz.seconds())
            hertz.reset()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            if (follower.currentTValue > 0.9 && follower.currentTValue < 0.95) {
                intake.update(Intake.state.SCANNING)
                hslides.setSetpoint(-23_000.0)
                follower.setMaxPower(0.4)
            }
            telemetry.update()
        }
        outtake.update(Outtake.state.TRANSFERING)
        timer.reset()
        intake.update(Intake.state.DIVING)
        timer.reset()
        while (timer.seconds() < 0.2 && !isStopRequested) {}
        intakeClaw.position = 0.4
        timer.reset()
        while (timer.seconds() < 0.2 && !isStopRequested) {}
        intake.update(Intake.state.TRANSFERING_OUTSIDE)
        timer.reset()
        hslides.setSetpoint(0.0)
        while (timer.seconds() < 1.0) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            //telemetry.addData("X", follower.pose.x)
            //telemetry.addData("Y", follower.pose.y)
            //telemetry.addData("Heading", follower.pose.heading)
            //telemetry.addData("Hertz", 1.0 / hertz.seconds())
            hertz.reset()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        timer.reset()
        outtakeClaw.position = 0.8
        intakeClaw.position = 0.68
        while (timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            //telemetry.addData("X", follower.pose.x)
            //telemetry.addData("Y", follower.pose.y)
            //telemetry.addData("Heading", follower.pose.heading)
            //telemetry.addData("Hertz", 1.0 / hertz.seconds())
            hertz.reset()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtake.update(Outtake.state.TRANSFERED)
        intake.update(Intake.state.SCANNING)
        hslides.setSetpoint(-23_000.0)
        timer.reset()
        while (timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            //telemetry.addData("X", follower.pose.x)
            //telemetry.addData("Y", follower.pose.y)
            //telemetry.addData("Heading", follower.pose.heading)
            //telemetry.addData("Hertz", 1.0 / hertz.seconds())
            hertz.reset()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        timer.reset()
        outtakeClaw.position = 1.0
        follower.followPath(pick1, 0.4, true)
        leftRGB.position=0.3
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
            //telemetry.addData("X", follower.pose.x)
            //telemetry.addData("Y", follower.pose.y)
            //telemetry.addData("Heading", follower.pose.heading)
            telemetry.addData("Hertz", 1.0 / hertz.seconds())
            hertz.reset()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtake.update(Outtake.state.TRANSFERING)
        timer.reset()
        intake.update(Intake.state.DIVING)
        timer.reset()
        while (timer.seconds() < 0.2 && !isStopRequested) {}
        intakeClaw.position = 0.4
        timer.reset()
        while (timer.seconds() < 0.2 && !isStopRequested) {}
        intake.update(Intake.state.TRANSFERING_OUTSIDE)
        timer.reset()
        hslides.setSetpoint(0.0)
        while (timer.seconds() < 1.0) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            //telemetry.addData("X", follower.pose.x)
            //telemetry.addData("Y", follower.pose.y)
            //telemetry.addData("Heading", follower.pose.heading)
            //telemetry.addData("Hertz", 1.0 / hertz.seconds())
            hertz.reset()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        timer.reset()
        outtakeClaw.position = 0.8
        intakeClaw.position = 0.68
        while (timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            //telemetry.addData("X", follower.pose.x)
            //telemetry.addData("Y", follower.pose.y)
            //telemetry.addData("Heading", follower.pose.heading)
            //telemetry.addData("Hertz", 1.0 / hertz.seconds())
            hertz.reset()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtake.update(Outtake.state.TRANSFERED)
        intake.update(Intake.state.SCANNING)
        hslides.setSetpoint(-23_000.0)
        timer.reset()
        while (timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            //telemetry.addData("X", follower.pose.x)
            //telemetry.addData("Y", follower.pose.y)
            //telemetry.addData("Heading", follower.pose.heading)
            //telemetry.addData("Hertz", 1.0 / hertz.seconds())
            hertz.reset()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        timer.reset()
        outtakeClaw.position = 1.0
    }

}