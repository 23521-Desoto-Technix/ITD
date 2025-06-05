package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.subsystems.Intake

class IntakeV2(hwmap: HardwareMap) {
    val arm = hwmap.servo["intakeArm"]
    val elbow = hwmap.servo["intakeElbow"]
    val wrist = hwmap.servo["intakeWrist"]
    enum class state {
        SCANNING,
        DIVING,
        TRANSFERING_INSIDE,
        TRANSFERING_OUTSIDE,
        OUT,
        IDLE,
        CAMERA,
        PASSTHROUGH_INSIDE,
        PASSTHROUGH_OUTSIDE,
    }

    fun update(state: Intake.state) {
        when (state) {
            Intake.state.SCANNING -> {
                arm.position = 0.19
                elbow.position = 0.24
            }
            Intake.state.DIVING -> {
                arm.position = 0.32
            }
            Intake.state.TRANSFERING_INSIDE -> {
                arm.position = 0.12
                elbow.position = 0.8
                wrist.position = 0.77
            }
            Intake.state.TRANSFERING_OUTSIDE -> {
                arm.position = 0.14
                elbow.position = 0.8
                wrist.position = 0.5
            }
            Intake.state.OUT -> {
                arm.position = 0.19
                elbow.position = 0.5
            }
            Intake.state.IDLE -> {
                arm.position = 0.2
                elbow.position = 0.97
                wrist.position = 0.5
            }
            Intake.state.PASSTHROUGH_INSIDE -> {
                arm.position = 0.00
                elbow.position = .88
                wrist.position = 1.0
            }
            Intake.state.PASSTHROUGH_OUTSIDE -> {
                arm.position = 0.0
                elbow.position = 0.88
                wrist.position = 0.0
            }
            Intake.state.CAMERA -> {
                arm.position = 0.00
                elbow.position = 0.55
            }
        }
    }
    fun wrist(position: Double) {
        wrist.position = position
    }
}