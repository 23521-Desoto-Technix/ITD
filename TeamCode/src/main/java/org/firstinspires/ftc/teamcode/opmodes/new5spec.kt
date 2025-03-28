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
        val pickupPose = Pose(5.0, 30.0)
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
        outtakeClaw.position = 0.65
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
        val push0 = follower.pathBuilder()
            .addPath( // Line 1
                BezierCurve(
                    Point(scorePose),
                    Point(Pose(2.0, 40.0)),
                    //Point(Pose(90.0, 23.0)),
                    Point(Pose(58.0, 27.0))
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val push1 = follower.pathBuilder()
            .addPath( // Line 1
                BezierCurve(
                    Point(Pose(48.0, 23.0)),
                    Point(Pose(20.0, 23.0))
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val push2 = follower.pathBuilder()
            .addPath( // Line 1
                BezierCurve(
                    Point(Pose(20.0, 23.0)),
                    Point(Pose(62.0, 28.0)),
                    Point(Pose(55.0, 13.0))
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val push3 = follower.pathBuilder()
            .addPath( // Line 1
                BezierCurve(
                    Point(Pose(55.0, 13.0)),
                    Point(Pose(20.0, 13.0))
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val push4 = follower.pathBuilder()
            .addPath( // Line 1
                BezierCurve(
                    Point(Pose(20.0, 13.0)),
                    Point(Pose(62.0, 13.0)),
                    Point(Pose(55.0, 7.0))
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val push5 = follower.pathBuilder()
            .addPath( // Line 1
                BezierCurve(
                    Point(Pose(55.0, 7.0)),
                    Point(Pose(22.0, 7.0))
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val push6 = follower.pathBuilder()
            .addPath( // Line 1
                BezierCurve(
                    Point(Pose(15.0, 7.0)),
                    Point(Pose(15.0, 30.0)),
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val push7 = follower.pathBuilder()
            .addPath( // Line 1
                BezierCurve(
                    Point(Pose(15.0, 30.0)),
                    //Point(Pose(18.0, 30.0)),
                    Point(pickupPose)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val score = follower.pathBuilder()
            .addPath( // Line 1
                BezierCurve(
                    Point(pickupPose),
                    Point(Pose(20.0, 74.0)),
                    Point(Pose(42.0, 74.0))
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val hp = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(Pose(42.0, 74.0)),
                    Point(Pose(10.0, 66.0)),
                    Point(Pose(30.0, 40.0)),
                    Point(pickupPose),
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        waitForStart()
        leftRGB.position = 0.722
        rightRGB.position = 0.722
        follower.followPath(startToScore, 0.6, true)
        vslides.setSetpoint(-51_000.0)
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
        follower.followPath(push0, true)
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
            if (follower.currentTValue > 0.9) {
                break
            }
            telemetry.update()
        }
        follower.followPath(push1, true)
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
            if (follower.currentTValue > 0.95) {
                break
            }
            telemetry.update()
        }
        follower.followPath(push2, true)
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
            if (follower.currentTValue > 0.95) {
                break
            }
            telemetry.update()
        }
        follower.followPath(push3, true)
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
            if (follower.currentTValue > 0.95) {
                break
            }
            telemetry.update()
        }
        follower.followPath(push4, true)
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
            if (follower.currentTValue > 0.95) {
                break
            }
            telemetry.update()
        }
        outtake.update(Outtake.state.INTAKING)
        vslides.setSetpoint(-25_600.0)
        follower.followPath(push5, true)
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
            if (follower.currentTValue > 0.9) {
                break
            }
            telemetry.update()
        }
        follower.followPath(push6, true)
        while (!isStopRequested) {
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
            if (follower.currentTValue > 0.5) {
                follower.setMaxPower(0.8)
            }
            if (follower.currentTValue > 0.9) {
                break
            }
            telemetry.update()
        }
        follower.followPath(push7, true)
        while (!isStopRequested) {
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
            follower.setMaxPower(0.7)
            if (touch.isPressed) {
                break
            }
            telemetry.update()
        }
        var i = 0
        //START SCORE LOOP
        while (!isStopRequested) {
            i += 1
            leftRGB.position = 0.3
            timer.reset()
            outtakeClaw.position = 0.65
            follower.followPath(score, true)
            follower.setMaxPower(1.0)
            while (timer.seconds() < 0.3) {

            }
            vslides.setSetpoint(-40_000.0)
            timer.reset()
            while (!isStopRequested) {
                vslides.update()
                hslides.update()
                follower.update()
                //telemetry.addData("T", follower.currentTValue)
                //telemetry.update()
                follower.telemetryDebug(telemetryA)
                hertz.reset()
                for (hub in allHubs) {
                    hub.clearBulkCache()
                }
                if (follower.pose.x > 15.0 && follower.pose.x < 16.0) {
                    outtake.update(Outtake.state.GRABBED)
                    vslides.setSetpoint(-51_000.0)
                }
                //TODO
                if (follower.pose.x > 43.0 || follower.velocity.magnitude < 0.3) {
                    break
                }
            }
            outtakeClaw.position = 1.0
            leftRGB.position = 0.7
            follower.followPath(hp, true)
            while (!isStopRequested) {
                vslides.update()
                hslides.update()
                follower.update()
                follower.telemetryDebug(telemetryA)
                //telemetry.addData("T", follower.currentTValue)
                //telemetry.update()
                hertz.reset()
                for (hub in allHubs) {
                    hub.clearBulkCache()
                }
                if (follower.currentTValue > 0.3 && follower.currentTValue < 0.4) {
                    if (i < 4) {
                        outtake.update(Outtake.state.INTAKING)
                        vslides.setSetpoint(-25_600.0)
                    } else {
                        outtake.update(Outtake.state.HOLDING)
                        vslides.setSetpoint(0.0)
                    }
                }
                if (follower.currentTValue > 0.7 && follower.currentTValue < 0.75 && i > 3) {
                    break
                }
                if (follower.currentTValue > 0.6 && follower.currentTValue < 0.65 && i < 4) {
                    follower.setMaxPower(0.6)
                }
                if (touch.isPressed) {
                    break
                }
            }
            if (i > 3) {
                break
            }
        }
    }

}