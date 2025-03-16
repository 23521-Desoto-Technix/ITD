package org.firstinspires.ftc.teamcode.vision

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Outtake
import org.firstinspires.ftc.vision.VisionPortal

@Autonomous
class SampleDetect : LinearOpMode() {
    override fun runOpMode() {
        val processor = BlockDetectorProcessor(telemetry)
        processor.detectionMode = BlockDetectorProcessor.DetectionMode.RED
        val visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(processor)
            .build()
        val intake = Intake(hardwareMap)
        intake.update(Intake.state.CAMERA)
        waitForStart()

        while (opModeIsActive()) {
            intake.update(Intake.state.CAMERA)
            val block = processor.closestBlock
            val angle = processor.angle
        }
    }
}