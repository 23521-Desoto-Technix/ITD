package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides

@TeleOp(name = "Reset heading + Encoders")
class cleanStart : LinearOpMode() {
    override fun runOpMode() {
        dataStorage.angle = 0.0
        val vslides = VerticalSlides(hardwareMap, telemetry)
        val hslides = HorizontalSlides(hardwareMap, telemetry)
        hslides.resetEncoder()
        vslides.resetEncoder()
    }
}