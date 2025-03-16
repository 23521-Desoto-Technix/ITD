package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap

class Lights(hwmap: HardwareMap) {
    val left = hwmap.servo["LeftRGB"]
    val right = hwmap.servo["RightRGB"]
    enum class state {
        AUTO,
        AUTO_INIT,
        TELEOP, //Solid White
        TELEOP_INIT,
        HEADING_LOCK,
        SLIDES_MANUAL,
        RED,
        YELLOW,
        BLUE,
    }
}