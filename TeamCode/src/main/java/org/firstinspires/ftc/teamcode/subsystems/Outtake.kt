package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap

class Outtake(hwmap: HardwareMap) {
    val armRight = hwmap.servo["outtakeArmRight"]
    val armLeft = hwmap.servo["outtakeArmLeft"]
    val elbow = hwmap.servo["outtakeElbow"]
    enum class state {
        TRANSFERING,
        TRANSFERED,
        INTAKING,
        GRABBED,
        HOLDING
    }

    fun update(state: state) = when (state) {
        Outtake.state.TRANSFERING -> {
            armRight.position = 0.79
            armLeft.position = 0.79
            elbow.position = 0.42
        }
        Outtake.state.HOLDING -> {
            armRight.position = 0.74
            armLeft.position = 0.74
            elbow.position = 0.43
        }
        Outtake.state.TRANSFERED -> {
            armRight.position = 0.48
            armLeft.position = 0.48
            elbow.position = 0.08
        }
        Outtake.state.INTAKING -> {
            armRight.position = 0.115
            armLeft.position = 0.115
            elbow.position = 0.11
        }
        Outtake.state.GRABBED -> {
            armRight.position = 0.85
            armLeft.position = 0.85
            elbow.position = 0.34
        }
    }
}