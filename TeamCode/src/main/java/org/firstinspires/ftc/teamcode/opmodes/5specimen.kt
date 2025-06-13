package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.util.Constants
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Outtake
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Autonomous(name = "5 Specimen")
class `5specimen` : LinearOpMode() {
    override fun runOpMode() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        val follower = Follower(hardwareMap)

        val spikeOnePose = Pose(68.0, 22.0, Math.toRadians(0.0))
        val pushOnePose = Pose(10.0, 22.0, Math.toRadians(0.0))
        val spikeTwoPose = Pose(53.0, 16.0, Math.toRadians(0.0))
        val pushTwoPose = Pose(20.0, 16.0, Math.toRadians(0.0))
        val spikeThreePose = Pose(53.0, 8.2, Math.toRadians(0.0))
        val pushThreePose = Pose(0.0, 9.0, Math.toRadians(0.0))

        val startPose = Pose(6.0, 66.0, Math.toRadians(0.0))
        val scorePose = Pose(42.0, 70.0, Math.toRadians(0.0))
        val pickupPose = Pose(0.0, 38.0)
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
        val backTouch = hardwareMap.touchSensor.get("backTouch")
        val leftTouch = hardwareMap.touchSensor.get("leftTouch")
        val rightTouch = hardwareMap.touchSensor.get("rightTouch")
        hslides.resetEncoder()
        vslides.resetEncoder()
        val outtakeClaw = hardwareMap.servo["outtakeClaw"]
        val intakeClaw = hardwareMap.servo["intakeClaw"]
        outtakeClaw.position = 0.58
        intakeClaw.position = 0.68
        val dig0 = hardwareMap.get(DigitalChannel::class.java, "dig0")
        val dig1 = hardwareMap.get(DigitalChannel::class.java, "dig1")
        val allHubs = hardwareMap.getAll(
            LynxModule::class.java
        )
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        fun update() {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }
        val path1 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    startPose,
                    Pose(42.0, 66.0, Math.toRadians(0.0))
                )
            ).build()
        val path2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    scorePose,
                    Pose(15.0, 42.0, Math.toRadians(0.0)),
                    spikeOnePose,
                ),
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path3 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Pose(70.0, 22.0, Math.toRadians(0.0)),
                    pushOnePose
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path4 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(10.0, 22.0, Math.toRadians(0.0)),
                    spikeOnePose,
                    spikeTwoPose
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path5 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(60.0, 16.0, Math.toRadians(0.0)),
                    pushTwoPose
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path6 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    pushTwoPose,
                    spikeTwoPose,
                    spikeThreePose
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path7 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    spikeThreePose,
                    pushThreePose
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path8 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(5.0, 7.0),
                    Pose(12.0, 50.0),
                    scorePose,
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val pick = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    scorePose,
                    Pose(30.0, 70.0),
                    Pose(30.0, 30.0),
                    pickupPose,
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val score = follower.pathBuilder()
            .addPath(
                BezierLine(
                    pickupPose,
                    scorePose,
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        follower.followPath(path1)
        waitForStart()
        hslides.setSetpoint(0.0)
        vslides.setSetpoint(-49_000.0)
        //follower.setMaxPower(0.8)
        outtake.update(Outtake.state.GRABBED)
        while (!isStopRequested && follower.isBusy) {
            update()
            if (leftTouch.isPressed || rightTouch.isPressed) {
                outtakeClaw.position = 0.8
                break
            }
        }
        follower.setMaxPower(1.0)
        vslides.setSetpoint(0.0)
        outtake.update(Outtake.state.INIT)
        follower.followPath(path2)
        while (!isStopRequested && follower.isBusy) {
            update()
            telemetry.addData("Y raw", follower.pose.y)
            if (follower.pose.y < 33.0) {
                break
            }
        }
        follower.followPath(path3)
        while (!isStopRequested && follower.isBusy) {
            if (follower.pose.x < 25.0) {
                break
            }
            update()
        }
        follower.followPath(path4)
        while (!isStopRequested && follower.isBusy) {
            if (follower.currentTValue > 0.8) {
                break
            }
            update()
        }
        follower.followPath(path5)
        while (!isStopRequested && follower.isBusy) {
            if (follower.currentTValue > 0.9) {
                break
            }
            update()
        }
        follower.followPath(path6)
        while (!isStopRequested && follower.isBusy) {
            if (follower.currentTValue > 0.9) {
                break
            }
            update()
        }
        follower.followPath(path7)
        vslides.setSetpoint(-26_000.0)
        outtake.update(Outtake.state.INTAKING)
        while (!isStopRequested && follower.isBusy) {
            if (backTouch.isPressed) {
                break
            }
            update()
        }
        follower.setMaxPower(1.0)
        outtakeClaw.position = 0.58
        sleep(100)
        vslides.setSetpoint(-49_000.0)
        follower.followPath(path8)
        while (!isStopRequested && follower.isBusy) {
            if (follower.pose.x > 12.0) {
                outtake.update(Outtake.state.GRABBED)
            }
            update()
            if (leftTouch.isPressed || rightTouch.isPressed) {
                outtakeClaw.position = 0.8
                break
            }
        }
        var counter = 0
        while (!isStopRequested) {
            counter += 1
            if (counter == 5) {
                break
            }
            follower.followPath(pick)
            while (!isStopRequested && follower.isBusy) {
                if (follower.pose.x < 35.0 && follower.pose.x > 33.0) {
                    vslides.setSetpoint(-26_000.0)
                    outtake.update(Outtake.state.INTAKING)
                }
                update()
                if (backTouch.isPressed) {
                    outtakeClaw.position = 0.58
                    break
                }
            }
            sleep(100)
            vslides.setSetpoint(-49_000.0)
            follower.setMaxPower(1.0)
            follower.followPath(score)
            while (!isStopRequested && follower.isBusy) {
                update()
                if (follower.pose.x > 12.0) {
                    outtake.update(Outtake.state.GRABBED)
                }
                if (leftTouch.isPressed || rightTouch.isPressed) {
                    outtakeClaw.position = 0.8
                    break
                }
            }
        }
    }
}