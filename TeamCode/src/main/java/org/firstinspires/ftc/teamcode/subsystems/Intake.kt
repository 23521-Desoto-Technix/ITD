package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap

class Intake(hwmap: HardwareMap) {
    val arm = hwmap.servo["intakeArm"]
    val elbow = hwmap.servo["intakeElbow"]
    val wrist = hwmap.servo["intakeWrist"]
    enum class state {
        SCANNING,
        DIVING,
        TRANSFERING,
        OUT,
        IDLE
    }

    fun update(state: state) {
        when (state) {
            Intake.state.SCANNING -> {
                arm.position = 0.19
                elbow.position = 0.27
            }
            Intake.state.DIVING -> {
                arm.position = 0.3
            }
            Intake.state.TRANSFERING -> {
                arm.position = 0.1
                elbow.position = 0.96
                wrist.position = 0.77
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
        }
    }
    fun wrist(position: Double) {
        wrist.position = position
    }
}