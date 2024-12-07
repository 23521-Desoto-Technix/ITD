package org.firstinspires.ftc.teamcode.rr.opmodes

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive
import org.firstinspires.ftc.teamcode.rr.actions.Outtake
import com.acmerobotics.roadrunner.SleepAction
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class port : LinearOpMode() {

    override fun runOpMode() {
        val beginPose = Pose2d(0.0, 0.0, 0.0)
        val drive = SparkFunOTOSDrive(hardwareMap, beginPose)
        val outtake = Outtake(hardwareMap)
        waitForStart()
        runBlocking(SleepAction(5.0))
        runBlocking(outtake.up())
    }
}