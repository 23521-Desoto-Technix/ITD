package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap

class OuttakeV2(hwmap: HardwareMap) {
    val armRight = hwmap.servo["outtakeArmRight"]
    val armLeft = hwmap.servo["outtakeArmLeft"]
    val elbow = hwmap.servo["outtakeElbow"]
    enum class state {
        TRANSFERING,
        TRANSFERED,
        INTAKING,
        GRABBED,
        HOLDING,
        INIT
    }

    fun update(state: Outtake.state) = when (state) {
        Outtake.state.TRANSFERING -> {
            armRight.position = 0.79
            armLeft.position = 0.79
            elbow.position = 0.41
        }
        Outtake.state.HOLDING -> {
            armRight.position = 0.74
            armLeft.position = 0.74
            elbow.position = 0.38
        }
        Outtake.state.TRANSFERED -> {
            armRight.position = 0.48
            armLeft.position = 0.48
            elbow.position = 0.22
        }
        Outtake.state.INTAKING -> {
            armRight.position = 0.35
            armLeft.position = 0.35
            elbow.position = 0.0
        }
        Outtake.state.GRABBED -> {
            armRight.position = 0.85
            armLeft.position = 0.85
            elbow.position = 0.34
        }
        Outtake.state.INIT -> {
            armRight.position = 0.85
            armLeft.position = 0.85
            elbow.position = 0.2
        }
    }
}