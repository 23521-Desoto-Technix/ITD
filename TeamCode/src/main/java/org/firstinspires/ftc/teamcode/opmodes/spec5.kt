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
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Outtake
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants

@Autonomous(name = "5 specimen")
class spec5 : LinearOpMode() {
    override fun runOpMode() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        val follower = Follower(hardwareMap)
        val startPose = Pose(6.0, 66.0, Math.toRadians(0.0))
        val scorePose = Pose(39.0, 66.0, Math.toRadians(0.0))
        val pickupPose = Pose(06.0, 8.0, Math.toRadians(0.0))
        val pickupPose2 = Pose(08.0, 30.0, Math.toRadians(0.0))
        val telemetryA: Telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
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
        val claw = hardwareMap.servo["outtakeClaw"]
        claw.position = 0.55
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
                    Point(55.0, 25.0, Point.CARTESIAN),
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
                    Point(55.000, 14.000, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .addPath(
                BezierCurve(
                    Point(55.000,14.000, Point.CARTESIAN),
                    Point(15.000,14.000, Point.CARTESIAN),
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val push3 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(15.000, 14.000, Point.CARTESIAN),
                    Point(55.000, 14.000, Point.CARTESIAN),
                    Point(55.000, 8.000, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .addPath(
                BezierCurve(
                    Point(55.000, 8.000, Point.CARTESIAN),
                    Point(pickupPose),
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val backToHP = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(39.0, 67.000, Point.CARTESIAN),
                    //Point(25.000, 50.000, Point.CARTESIAN),
                    //Point(30.000, 30.000, Point.CARTESIAN),
                    Point(pickupPose2)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val backToHPFinal = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(37.500, 70.000, Point.CARTESIAN),
                    //Point(25.000, 50.000, Point.CARTESIAN),
                    Point(30.000, 30.000, Point.CARTESIAN),
                    Point(9.0,30.0, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val score1 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pickupPose2),
                    //Point(11.5, 70.0, Point.CARTESIAN),
                    Point(39.0, 67.0, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val score2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pickupPose2),
                    //Point(9.0, 74.0, Point.CARTESIAN),
                    Point(39.0, 67.0, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val score3 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pickupPose2),
                    //Point(9.0, 78.0, Point.CARTESIAN),
                    Point(39.0, 67.0, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0.0))
            .build()
        val score4 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Point(pickupPose2),
                    //Point(9.0, 78.0, Point.CARTESIAN),
                    Point(39.0, 67.0, Point.CARTESIAN)
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
        val hertz = ElapsedTime()
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
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
        timer.reset()
        vslides.setSetpoint(-54_000.0)
        while (timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        claw.position = 1.0
        vslides.setSetpoint(-0.0)
        follower.followPath(push1, true)
        outtake.update(Outtake.state.INTAKING)
        timer.reset()
        while (!isStopRequested && follower.isBusy) {
            if (follower.pose.x < 20.0 && timer.seconds() > 1.0) {
                break
            }
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
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
        follower.followPath(push2, true)
        timer.reset()
        while (!isStopRequested && follower.isBusy && (!dig0.state && !dig1.state)) {
            if (timer.seconds() > 1.0) {
                outtake.update(Outtake.state.INTAKING)
            }
            if (follower.pose.x < 20.0 && timer.seconds() > 1.0) {
                break
            }
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
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
        follower.followPath(push3, true)
        timer.reset()
        while (!isStopRequested && follower.isBusy && (!dig0.state && !dig1.state)) {
            if (timer.seconds() > 1.0) {
                outtake.update(Outtake.state.INTAKING)
            }
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
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
        follower.breakFollowing()
        claw.position = 0.55
        timer.reset()
        while (timer.seconds() < 0.1) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        vslides.setSetpoint(-33_000.0)
        while (timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        outtake.update(Outtake.state.GRABBED)
        follower.followPath(score1, true)
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
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
        timer.reset()
        vslides.setSetpoint(-54_000.0)
        while (timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        outtake.update(Outtake.state.INTAKING)
        claw.position = 1.0
        vslides.setSetpoint(-0.0)
        follower.followPath(backToHP, true)
        while (!isStopRequested && follower.isBusy && ((!dig0.state && !dig1.state) || (timer.seconds() < 1.0))) {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
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
        follower.breakFollowing()
        claw.position = 0.55
        timer.reset()
        while (timer.seconds() < 0.1) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        vslides.setSetpoint(-33_000.0)
        while (timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        outtake.update(Outtake.state.GRABBED)
        follower.followPath(score2, true)
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
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
        timer.reset()
        vslides.setSetpoint(-54_000.0)
        while (timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        outtake.update(Outtake.state.INTAKING)
        claw.position = 1.0
        vslides.setSetpoint(-0.0)
        follower.followPath(backToHP, true)
        //TODO 4 (add the sensor thingy)
        timer.reset()
        while (!isStopRequested && follower.isBusy && ((!dig0.state && !dig1.state) || (timer.seconds() < 1.0))) {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
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
        follower.breakFollowing()
        claw.position = 0.55
        timer.reset()
        while (timer.seconds() < 0.1) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        vslides.setSetpoint(-33_000.0)
        while (timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        outtake.update(Outtake.state.GRABBED)
        follower.followPath(score3, true)
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
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
        timer.reset()
        vslides.setSetpoint(-54_000.0)
        while (timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        outtake.update(Outtake.state.INTAKING)
        claw.position = 1.0
        vslides.setSetpoint(-0.0)
        //while (timer.seconds() < 1.5) {
        //    vslides.update()
        //}
        follower.followPath(backToHP, true)
        //TODO 4 (add the sensor thingy)
        timer.reset()
        while (!isStopRequested && follower.isBusy && ((!dig0.state && !dig1.state) || (timer.seconds() < 1.0))) {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
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
        follower.breakFollowing()
        claw.position = 0.55
        timer.reset()
        while (timer.seconds() < 0.1) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        vslides.setSetpoint(-33_000.0)
        while (timer.seconds() < 0.2) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        outtake.update(Outtake.state.GRABBED)
        follower.followPath(score4, true)
        while (!isStopRequested && follower.isBusy) {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
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
        timer.reset()
        vslides.setSetpoint(-54_000.0)
        while (timer.seconds() < 0.3) {
            vslides.update()
            hslides.update()
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
        }
        vslides.setSetpoint(-0.0)
        outtake.update(Outtake.state.INTAKING)
        claw.position = 1.0
        timer.reset()
        while (!isStopRequested) {
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            if (timer.seconds() < 1.0) {
                vslides.update()
            } else {
                vslides.setPower(0.0)
            }
            hslides.update()
        }
    }

}