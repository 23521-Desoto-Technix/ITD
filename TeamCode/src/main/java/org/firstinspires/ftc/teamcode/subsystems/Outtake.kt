package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap

class Outtake(hwmap: HardwareMap) {
    val armRight = hwmap.servo["outtakeArmRight"]
    val armLeft = hwmap.servo["outtakeArmLeft"]
    val elbow = hwmap.servo["outtakeElbow"]
    val wrist = hwmap.servo["outtakeWrist"]
    enum class state {
        TRANSFERING,
        TRANSFERED,
        INTAKING,
        GRABBED
    }

    fun update(state: state) {
        when (state) {
            Outtake.state.TRANSFERING -> {
                armRight.position = 0.65
                armLeft.position = 0.65
                elbow.position = 0.75
                wrist.position = 0.228
            }
            Outtake.state.TRANSFERED -> {
                armRight.position = 0.4
                armLeft.position = 0.4
                elbow.position = 0.07
                wrist.position = 0.228
            }
            Outtake.state.INTAKING -> {
                armRight.position = 0.05
                armLeft.position = 0.1
                elbow.position = 0.47
                wrist.position = 0.228
            }
            Outtake.state.GRABBED -> {
                armRight.position = 0.8
                armLeft.position = 0.8
                elbow.position = 0.3
                wrist.position = 0.228
            }
        }
    }
    fun wrist(position: Double) {
        wrist.position = position
    }
}