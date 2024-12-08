package org.firstinspires.ftc.teamcode.rr.actions
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

internal class Outtake(hardwareMap: HardwareMap) {
    private val vert0: DcMotor = hardwareMap.dcMotor["vert0"]
    private val vert1: DcMotor = hardwareMap.dcMotor["vert1"]

    private val CV4B0O = hardwareMap.servo["CV4B0O"]
    private val CV4B1O = hardwareMap.servo["CV4B1O"]

    private val wristO = hardwareMap.servo["wristO"]

    private val clawO = hardwareMap.servo["clawO"]

    init {
        vert0.direction = DcMotorSimple.Direction.REVERSE
        vert0.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        vert1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        vert0.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        vert1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private val PID = PIDController(0.01, 0.0, 0.0)
    fun down(): Action {
        return object: Action {
            private val target = 0.0

            override fun run(packet: TelemetryPacket): Boolean {
                CV4B0O.position = 0.0
                CV4B1O.position = 0.93
                wristO.position = 0.98
                vert0.power = PID.calculate(vert0.currentPosition.toDouble(), target)
                vert1.power = PID.calculate(vert0.currentPosition.toDouble(), target)
                packet.put("position", vert0.currentPosition.toDouble())
                if (!(vert0.currentPosition.toDouble() >= target+5)) {
                    vert0.power=0.0
                    vert1.power=0.0
                }
                return vert0.currentPosition.toDouble() >= target+5
            }
        }
    }
    fun grab(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                clawO.position = 0.3
                return false
            }
        }
    }
    fun up(): Action {
        return object: Action {
            private val target = 1600.0

            override fun run(packet: TelemetryPacket): Boolean {
                CV4B0O.position = 0.0
                CV4B1O.position = 0.9
                wristO.position = 0.98
                vert0.power = PID.calculate(vert0.currentPosition.toDouble(), target)
                vert1.power = PID.calculate(vert0.currentPosition.toDouble(), target)
                packet.put("position", vert0.currentPosition.toDouble())
                if (vert0.currentPosition.toDouble() >= target-10) {
                    vert0.power=0.1
                    vert1.power=0.1
                }
                return vert0.currentPosition.toDouble() <= target-5
            }
        }
    }

    fun upMore(): Action {
        return object: Action {
            private val target = 2400.0

            override fun run(packet: TelemetryPacket): Boolean {
                CV4B0O.position = 0.0
                CV4B1O.position = 0.9
                wristO.position = 0.98
                vert0.power = PID.calculate(vert0.currentPosition.toDouble(), target)
                vert1.power = PID.calculate(vert0.currentPosition.toDouble(), target)
                packet.put("position", vert0.currentPosition.toDouble())
                if (vert0.currentPosition.toDouble() >= target-10) {
                    clawO.position = 0.65
                }
                return vert0.currentPosition.toDouble() <= target-5
            }
        }
    }
    fun flip(way: Boolean = true): Action {
        return object: Action {
            private val target = 2400.0

            override fun run(packet: TelemetryPacket): Boolean {
                if (way) {
                    wristO.position = 0.98
                } else {
                    wristO.position = 0.41
                }
                return false
            }
        }
    }
    fun finish(): Action {
        return object: Action {
            override fun run(packet: TelemetryPacket): Boolean {
                CV4B0O.position = 1.0
                CV4B1O.position = 1.0
                return false
            }
        }
    }

}