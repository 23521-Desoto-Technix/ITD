package org.firstinspires.ftc.teamcode.rr.opmodes

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive
import org.firstinspires.ftc.teamcode.rr.actions.Outtake

@Autonomous
class AUTOv0 : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val beginPose = Pose2d(0.0, 0.0, 0.0)
        val clawO = hardwareMap.servo["clawO"]
        val drive = SparkFunOTOSDrive(hardwareMap, beginPose)
        val outtake = Outtake(hardwareMap)
        clawO.position = 0.3
        waitForStart()
        runBlocking(
            drive.actionBuilder(beginPose)
                .afterTime(0.2, outtake.up())
                .splineToLinearHeading(Pose2d(22.0, -2.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                .splineToLinearHeading(Pose2d(26.0, -2.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                .build()
        )
        runBlocking(outtake.upMore())
        runBlocking(
            drive.actionBuilder(Pose2d(26.0, 0.0, Math.toRadians(180.0)))
                .afterTime(0.2, outtake.down())
                .splineToLinearHeading(Pose2d(12.0, -30.0, Math.toRadians(0.0)), Math.toRadians(180.0))
                .build()
        )
        runBlocking(outtake.flip(false))
        runBlocking(SleepAction(1.0))
        runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToLinearHeading(Pose2d(6.0, -30.0, Math.toRadians(0.0)), Math.toRadians(180.0))
                .build()
        )
        runBlocking(outtake.grab())
        runBlocking(
            drive.actionBuilder(drive.pose)
                .afterTime(0.3, outtake.up())
                .afterTime(0.6, outtake.flip())
                .splineToLinearHeading(Pose2d(10.0, -30.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .splineToLinearHeading(Pose2d(26.0, 10.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                .build()
        )
        runBlocking(outtake.upMore())
        runBlocking(
            drive.actionBuilder(drive.pose)
                .afterTime(0.2, SequentialAction(outtake.down(), outtake.finish()))
                .splineToLinearHeading(Pose2d(6.0, -30.0, Math.toRadians(0.0)), Math.toRadians(180.0))
                .build()
        )
        runBlocking(SleepAction(1.0))
    }
}