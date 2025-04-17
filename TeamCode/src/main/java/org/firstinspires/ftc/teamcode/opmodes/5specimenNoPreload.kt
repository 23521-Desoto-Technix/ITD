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

@Autonomous(name = "5 Specimen No Preload")
class `5specimenNoPreload` : LinearOpMode() {
    override fun runOpMode() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        val follower = Follower(hardwareMap)
        val startPose = Pose(6.0, 66.0, Math.toRadians(0.0))
        val scorePose = Pose(42.0, 66.0, Math.toRadians(0.0))
        val pickupPose = Pose(4.0, 30.0)
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
        outtake.update(Outtake.state.INIT)
        val vslides = VerticalSlides(hardwareMap, telemetry)
        val hslides = HorizontalSlides(hardwareMap, telemetry)
        val touch = hardwareMap.touchSensor.get("backTouch")
        hslides.resetEncoder()
        vslides.resetEncoder()
        hslides.setSetpoint(-1_000.0)
        val outtakeClaw = hardwareMap.servo["outtakeClaw"]
        val intakeClaw = hardwareMap.servo["intakeClaw"]
        outtakeClaw.position = 0.73
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
        val hp = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(Pose(42.0, 70.0)),
                    Point(Pose(10.0, 66.0)),
                    Point(Pose(30.0, 40.0)),
                    Point(pickupPose),
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val pick1 = Pose(26.0, 39.5, Math.toRadians(-45.0))
        val drop1 = Pose(26.0, 29.5, Math.toRadians(-220.0))
        val pick2 = Pose(26.0, 29.5, Math.toRadians(-45.0))
        val drop2 = Pose(26.0, 29.5, Math.toRadians(-220.0))
        val pick3 = Pose(25.0, 19.3, Math.toRadians(-45.0))
        val drop3 = Pose(30.0, 30.0, Math.toRadians(-220.0))
        val middle = Pose(15.0, 30.0)
        val samplePose = Pose(6.0, 119.0, Math.toRadians(-90.0))
        val awayPose = Pose(6.0, 0.0, Math.toRadians(-90.0))
        waitForStart()
        outtake.update(Outtake.state.GRABBED)
        leftRGB.position = 0.722
        rightRGB.position = 0.722
        follower.followPath(startToScore, 0.6, true)
        vslides.setSetpoint(-51_000.0)
        val timer = ElapsedTime()
        val hertz = ElapsedTime()
        follower.holdPoint(pick1)
        follower.setMaxPower(1.0)
        intake.wrist(0.37)
        while (!isStopRequested) {
            if (timer.seconds() > 0.9) {
                hslides.setSetpoint(-22_000.0)
                vslides.setSetpoint(0.0)
                intake.update(Intake.state.SCANNING)
            }
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
            if (follower.headingError < Math.toRadians(2.0) && follower.translationalError.magnitude < 0.5 && follower.velocity.magnitude < 0.5) {
                break
            }
            telemetry.update()
        }
        timer.reset()
        intake.update(Intake.state.DIVING)
        while (!isStopRequested) {
            if (timer.seconds() > 0.1) {
                intakeClaw.position = 0.4
            }
            if (timer.seconds() > 0.2) {
                intake.update(Intake.state.SCANNING)
                break
            }
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
        }
        follower.holdPoint(drop1)
        timer.reset()
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
            if (timer.seconds() > 0.7) {
                break
            }
            telemetry.update()
        }
        intakeClaw.position = 0.68
        follower.holdPoint(pick2)
        follower.setMaxPower(1.0)
        intake.wrist(0.37)
        while (!isStopRequested) {
            if (timer.seconds() > 0.9) {
                hslides.setSetpoint(-22_000.0)
                intake.update(Intake.state.SCANNING)
            }
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
            if (follower.headingError < Math.toRadians(2.0) && follower.translationalError.magnitude < 0.5 && follower.velocity.magnitude < 0.5) {
                break
            }
            telemetry.update()
        }
        timer.reset()
        intake.update(Intake.state.DIVING)
        while (!isStopRequested) {
            if (timer.seconds() > 0.1) {
                intakeClaw.position = 0.4
            }
            if (timer.seconds() > 0.2) {
                intake.update(Intake.state.SCANNING)
                break
            }
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
        }
        follower.holdPoint(drop2)
        timer.reset()
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
            if (timer.seconds() > 0.65) {
                break
            }
            telemetry.update()
        }
        intakeClaw.position = 0.68

        follower.holdPoint(pick3)
        follower.setMaxPower(1.0)
        intake.wrist(0.37)
        hslides.setSetpoint(0.0)
        while (!isStopRequested) {
            if (timer.seconds() > 1.1) {
                hslides.setSetpoint(-22_000.0)
                intake.update(Intake.state.SCANNING)
            }
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
            if (follower.headingError < Math.toRadians(2.0) && follower.translationalError.magnitude < 0.5 && follower.velocity.magnitude < 0.5) {
                break
            }
            telemetry.update()
        }
        timer.reset()
        intake.update(Intake.state.DIVING)
        while (!isStopRequested) {
            if (timer.seconds() > 0.1) {
                intakeClaw.position = 0.4
            }
            if (timer.seconds() > 0.2) {
                intake.update(Intake.state.SCANNING)
                break
            }
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
        }

        follower.holdPoint(drop3)
        outtake.update(Outtake.state.INTAKING)
        timer.reset()
        hslides.setSetpoint(0.0)
        while (!isStopRequested) {
            if (timer.seconds() > 0.3 && timer.seconds() < 0.4) {
                hslides.setSetpoint(-22_000.0)
            }
            if (timer.seconds() > 0.8 && timer.seconds() < 0.9) {
                intakeClaw.position = 0.63
                follower.holdPoint(pickupPose)
                hslides.setSetpoint(0.0)
                vslides.setSetpoint(-25_600.0)
                outtakeClaw.position = 1.0
            }
            if (timer.seconds() > 1.55 && timer.seconds() < 1.65) {
                follower.setMaxPower(0.5)
            }
            if (touch.isPressed) {
                break
            }
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
        }
        intake.update(Intake.state.SCANNING)
        //START SCORE LOOP
        var i = 0
        while (!isStopRequested) {
            i += 1
            leftRGB.position = 0.3
            timer.reset()
            outtakeClaw.position = 0.73
            follower.holdPoint(Pose(40.0, 72.0))
            follower.setMaxPower(1.0)
            while (timer.seconds() < 0.3) {

            }
            vslides.setSetpoint(-40_000.0)
            timer.reset()
            while (!isStopRequested) {
                vslides.update()
                hslides.update()
                follower.update()
                if (!isStopRequested) {
                    dataStorage.angle = follower.pose.heading
                }
                //telemetry.addData("T", follower.currentTValue)
                //telemetry.update()
                follower.telemetryDebug(telemetryA)
                hertz.reset()
                for (hub in allHubs) {
                    hub.clearBulkCache()
                }
                if (follower.pose.x > 15.0 && follower.pose.x < 16.0) {
                    outtake.update(Outtake.state.GRABBED)
                    vslides.setSetpoint(-48_000.0)
                }
                //TODO
                if (follower.pose.x > 39.5 || (follower.velocity.magnitude < 0.3 && timer.seconds() > 0.8)) {
                    break
                }
            }
            outtakeClaw.position = 1.0
            leftRGB.position = 0.7
            follower.holdPoint(middle)
            while (!isStopRequested) {
                vslides.update()
                hslides.update()
                follower.update()
                if (!isStopRequested) {
                    dataStorage.angle = follower.pose.heading
                }
                follower.telemetryDebug(telemetryA)
                //telemetry.addData("T", follower.currentTValue)
                //telemetry.update()
                hertz.reset()
                for (hub in allHubs) {
                    hub.clearBulkCache()
                }
                if (follower.pose.x < 30.0 && follower.pose.x > 29.0) {
                    if (i > 4) {
                        outtake.update(Outtake.state.TRANSFERING)
                        vslides.setSetpoint(0.0)
                    } else {
                        outtake.update(Outtake.state.INTAKING)
                        vslides.setSetpoint(-25_600.0)
                    }
                }
                if (follower.pose.x < 18.0 && follower.pose.x > 17.0) {
                    follower.holdPoint(pickupPose)
                }
                if (follower.pose.x < 18.0 && follower.pose.x > 0.0) {
                    follower.setMaxPower(0.5)
                }
                if (touch.isPressed) {
                    break
                }
            }
            sleep(200)
            if (i > 4) {
                break
            }
        }
        if (!isStopRequested) {
            dataStorage.angle = follower.pose.heading
        }
    }
}