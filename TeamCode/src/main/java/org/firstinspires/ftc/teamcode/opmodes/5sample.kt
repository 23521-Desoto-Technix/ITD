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
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Outtake
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Autonomous(name = "5 Sample")
class `5sample` : LinearOpMode() {
    override fun runOpMode() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        val follower = Follower(hardwareMap)
        val startPose = Pose(6.0, 116.0, Math.toRadians(0.0))
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
        val touch = hardwareMap.touchSensor.get("backTouch")
        hslides.resetEncoder()
        vslides.resetEncoder()
        hslides.setSetpoint(-1_000.0)
        val outtakeClaw = hardwareMap.servo["outtakeClaw"]
        val intakeClaw = hardwareMap.servo["intakeClaw"]
        outtakeClaw.position = 0.8
        intakeClaw.position = 0.68
        val allHubs = hardwareMap.getAll(
            LynxModule::class.java
        )
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        val scorePose = Pose(13.5, 131.0, Math.toRadians(-45.0))
        val parkPose = Pose(61.0, 94.0, Math.toRadians(0.0))
        val startToScore = follower.pathBuilder()
            .addPath( // Line 1
                BezierLine(
                    Point(startPose),
                    Point(scorePose)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-45.0))
            .build()

        val pick1 = follower.pathBuilder()
            .addPath( // Line 1
                BezierLine(
                    Point(scorePose),
                    Point(Pose(18.0, 124.0))
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-45.0), Math.toRadians(0.0))
            .build()
        val pick2 = follower.pathBuilder()
            .addPath( // Line 1
                BezierLine(
                    Point(scorePose),
                    Point(Pose(18.0, 133.0))
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-45.0), Math.toRadians(0.0))
            .build()
        val pick3 = follower.pathBuilder()
            .addPath( // Line 1
                BezierLine(
                    Point(scorePose),
                    Point(Pose(22.0, 130.0))
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-45.0), Math.toRadians(40.0))
            .build()
        val park = follower.pathBuilder()
            .addPath( // Line 1
                BezierCurve(
                    Point(scorePose),
                    Point(Pose(60.0, 125.0)),
                    Point(parkPose)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-45.0), Math.toRadians(-90.0))
            .build()
        val preload = Pose(8.0, 100.0, Math.toRadians(-90.0))
        waitForStart()
        intake.update(Intake.state.SCANNING)
        follower.followPath(startToScore,0.7, true)
        vslides.setSetpoint(-100_000.0)
        val timer = ElapsedTime()
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        while (vslides.encoder.currentPosition > -97_000.0) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtake.update(Outtake.state.TRANSFERED)
        timer.reset()
        while (timer.seconds() < 0.6) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtakeClaw.position = 1.0
        timer.reset()
        while (timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtake.update(Outtake.state.TRANSFERING)
        follower.followPath(pick1,0.5, true)
        timer.reset()
        hslides.setSetpoint(-25_000.0)
        while (!isStopRequested && (follower.isBusy || follower.velocity.magnitude > 1.0)) {
            if (timer.seconds() > 0.5) {
                vslides.setSetpoint(0.0)
            }
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        //START INTAKE+TRANSFER
        timer.reset()
        intake.update(Intake.state.DIVING)
        while (!isStopRequested && timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        intakeClaw.position = 0.4
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.1) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        intake.update(Intake.state.TRANSFERING_OUTSIDE)
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        hslides.setSetpoint(-1000.0)
        follower.holdPoint(scorePose)
        follower.setMaxPower(1.0)
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.6) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtakeClaw.position = 0.8
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        intakeClaw.position = 0.68
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        vslides.setSetpoint(-100_000.0)
        intake.update(Intake.state.SCANNING)
        while (!isStopRequested && vslides.encoder.currentPosition > -97_000.0) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtake.update(Outtake.state.TRANSFERED)
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.6) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtakeClaw.position = 1.0
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        //END INTAKE+TRANSFER
        outtake.update(Outtake.state.TRANSFERING)
        follower.followPath(pick2,0.5, true)
        timer.reset()
        hslides.setSetpoint(-25_000.0)
        while (!isStopRequested && (follower.isBusy || follower.velocity.magnitude > 1.0)) {
            if (timer.seconds() > 0.5) {
                vslides.setSetpoint(0.0)
            }
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        //START INTAKE+TRANSFER
        timer.reset()
        intake.update(Intake.state.DIVING)
        while (!isStopRequested && timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        intakeClaw.position = 0.4
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        intake.update(Intake.state.TRANSFERING_OUTSIDE)
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        hslides.setSetpoint(-1000.0)
        follower.holdPoint(scorePose)
        follower.setMaxPower(1.0)
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.6) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtakeClaw.position = 0.8
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        intakeClaw.position = 0.68
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        vslides.setSetpoint(-100_000.0)
        intake.update(Intake.state.SCANNING)
        while (!isStopRequested && vslides.encoder.currentPosition > -97_000.0) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtake.update(Outtake.state.TRANSFERED)
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.6) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtakeClaw.position = 1.0
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        //END INTAKE+TRANSFER
        outtake.update(Outtake.state.TRANSFERING)
        follower.followPath(pick3,0.5, true)
        timer.reset()
        hslides.setSetpoint(-25_000.0)
        intake.wrist(0.63)
        while (!isStopRequested && (follower.isBusy || follower.velocity.magnitude > 1.0)) {
            if (timer.seconds() > 0.5) {
                vslides.setSetpoint(0.0)
            }
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        //START INTAKE+TRANSFER
        timer.reset()
        intake.update(Intake.state.DIVING)
        while (!isStopRequested && timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        intakeClaw.position = 0.4
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        intake.update(Intake.state.TRANSFERING_OUTSIDE)
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        hslides.setSetpoint(-1000.0)
        follower.holdPoint(scorePose)
        follower.setMaxPower(1.0)
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.6) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtakeClaw.position = 0.8
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        intakeClaw.position = 0.68
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        vslides.setSetpoint(-100_000.0)
        intake.update(Intake.state.SCANNING)
        while (!isStopRequested && vslides.encoder.currentPosition > -97_000.0) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtake.update(Outtake.state.TRANSFERED)
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.6) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtakeClaw.position = 1.0
        timer.reset()
        while (!isStopRequested && timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        //END INTAKE+TRANSFER
        follower.holdPoint(preload)
        vslides.setSetpoint(0.0)
        intake.wrist(0.5)
        intake.update(Intake.state.SCANNING)
        timer.reset()
        while (!isStopRequested) {
            if (timer.seconds() > 0.8) {
                hslides.setSetpoint(-22_000.0)
            }
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            if (timer.seconds() > 2.0) {
                break
            }
            telemetry.update()
        }
        timer.reset()
        outtake.update(Outtake.state.TRANSFERING)
        intake.update(Intake.state.DIVING)
        while (!isStopRequested) {
            if (timer.seconds() > 0.1 && timer.seconds() < 0.2) {
                intakeClaw.position = 0.4
            }
            if (timer.seconds() > 0.2 && timer.seconds() < 0.3) {
                intake.update(Intake.state.TRANSFERING_OUTSIDE)
                hslides.setSetpoint(-1_000.0)
            }
            if (timer.seconds() > 0.5 && timer.seconds() < 0.6) {
                outtakeClaw.position = 0.73
            }
            if (timer.seconds() > 0.6 && timer.seconds() < 0.7) {
                intakeClaw.position = 0.63
                outtake.update(Outtake.state.TRANSFERED)
                vslides.setSetpoint(-100_000.0)
            }
            if (timer.seconds() > 0.8) {
                break
            }
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        follower.holdPoint(scorePose)
        timer.reset()
        while (!isStopRequested) {
            if (timer.seconds() > 2.0) {
                break
            }
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        outtakeClaw.position = 1.0
        follower.followPath(park, true)
        outtake.update(Outtake.state.TRANSFERING)
        intake.update(Intake.state.TRANSFERING_INSIDE)
        follower.setMaxPower(1.0)
        timer.reset()
        while (!isStopRequested && follower.isBusy) {
            dataStorage.angle = follower.pose.heading
            vslides.update()
            hslides.update()
            follower.update()
            follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            if (timer.seconds() > 0.4) {
                vslides.setSetpoint(-26_000.0)
            }
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        //dataStorage.angle = follower.pose.heading
    }

}