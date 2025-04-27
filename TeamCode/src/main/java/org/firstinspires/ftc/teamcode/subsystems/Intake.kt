package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap

class Intake(hwmap: HardwareMap) {
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

    fun update(state: state) {
        when (state) {
            Intake.state.SCANNING -> {
                arm.position = 0.19
                elbow.position = 0.25
            }
            Intake.state.DIVING -> {
                arm.position = 0.3
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
                wrist.position = 0.55
            }
            Intake.state.PASSTHROUGH_OUTSIDE -> {
                arm.position = 0.00
                elbow.position = .88
                wrist.position = 0.65
            }
            Intake.state.CAMERA -> {
                arm.position = 0.0
                elbow.position = 0.57
            }
        }
    }
    fun wrist(position: Double) {
        wrist.position = position
    }
}