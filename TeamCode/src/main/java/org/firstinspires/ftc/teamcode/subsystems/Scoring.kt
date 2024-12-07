package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx
import dev.frozenmilk.dairy.cachinghardware.CachingServo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utils.Detector
import org.firstinspires.ftc.teamcode.utils.ServoSwap
import org.firstinspires.ftc.teamcode.vision.BlockDetectorProcessor
import org.firstinspires.ftc.vision.VisionPortal
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min


class Scoring(
    hardwareMap: HardwareMap,
    private var telemetry: Telemetry,
    private val gamepad1: Gamepad,
    private val gamepad2: Gamepad) {

    private val timeSinceStateChange = ElapsedTime()
    private val timer = ElapsedTime()

    private val inspection = false

    private val wristI: CachingServo = CachingServo(hardwareMap.servo["wristI"])
    private val clawI: CachingServo = CachingServo(hardwareMap.servo["clawI"])
    private val clawISwap = ServoSwap(clawI, 1.0, 0.73, true)

    private val CV4B0I = hardwareMap.servo["CV4B0I"]
    private val CV4B1I = hardwareMap.servo["CV4B1I"]

    private val CV4B0O = hardwareMap.servo["CV4B0O"]
    private val CV4B1O = hardwareMap.servo["CV4B1O"]

    private val clawO: CachingServo = CachingServo(hardwareMap.servo["clawO"])
    private val wristO = hardwareMap.servo["wristO"]
    private val clawOSwap = ServoSwap(clawO, 0.3, 0.67, true)

    val blockDetector = BlockDetectorProcessor(telemetry)
    private var wristPosition = 0.5
    /*
    var visionPortal: VisionPortal = org.firstinspires.ftc.vision.VisionPortal.Builder()
        .setCamera(hardwareMap.get<WebcamName>(WebcamName::class.java, "Webcam 1"))
        .addProcessor(blockDetector)
        .build()*/

    private enum class State {
        IDLE,
        GRABBING_SAMPLE,
        GRABBED_SAMPLE,
        TRANSFERRED,
        GRABBING_SPECIMEN,
        GRABBED_SPECIMEN,
        EXPANDED,
        HANGING
    }
    val horz = CachingDcMotorEx(hardwareMap.dcMotor["horz"] as DcMotorEx)
    val vert1 = CachingDcMotorEx(hardwareMap.dcMotor["vert1"] as DcMotorEx)
    val vert0 = CachingDcMotorEx(hardwareMap.dcMotor["vert0"] as DcMotorEx)

    val pid = PIDController(0.04,0.0,0.001)
    var target = 0.0
    private var state = State.IDLE
    var pin0 = hardwareMap.digitalChannel["i0"]
    var pin1 = hardwareMap.digitalChannel["i1"]

    init {
        horz.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        horz.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        vert0.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        vert0.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        vert0.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        vert1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        horz.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        vert1.direction = DcMotorSimple.Direction.REVERSE
        vert0.direction = DcMotorSimple.Direction.FORWARD
    }

    private val randomDetector = Detector()

    fun update() {

        val block = blockDetector.closestBlock
        val angle = blockDetector.angle
        val size = blockDetector.size
        randomDetector.update(gamepad2.ps)
        if (randomDetector.risingEdge() && state != State.HANGING && state != State.EXPANDED) {
            state = if (inspection) {
                State.EXPANDED
            } else {
                State.HANGING
            }
            timeSinceStateChange.reset()
        }
        if (abs(gamepad2.right_trigger) < 0.1) {
            if (abs(horz.currentPosition.toDouble() - target) > 5) {
                val tp: Double = pid.calculate(horz.currentPosition.toDouble(), target)
                if (tp > 0.0) {
                    horz.power = max(0.2, tp)
                } else {
                    horz.power = min(-0.2, tp)
                }
                horz.power = tp
            } else {
                horz.power = 0.0
            }
        } else {
            horz.power = -gamepad2.right_trigger.toDouble()
        }
        if (gamepad2.back) {
            horz.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            horz.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        clawISwap.update(gamepad2.right_bumper)
        clawOSwap.update(gamepad2.left_bumper)
        wristI.position = wristPosition
        //horz.power = gamepad2.right_stick_y.toDouble()
        if (gamepad2.circle) {
            state = State.GRABBING_SAMPLE
            timeSinceStateChange.reset()
            clawISwap.set(false)
        }
        if (gamepad2.dpad_down) {
            state = State.GRABBING_SPECIMEN
            timeSinceStateChange.reset()
            clawOSwap.set(false)
        }
        when (state) {
            State.GRABBING_SAMPLE -> {
                CV4B0O.position = 0.6
                CV4B1O.position = 0.36
                if (gamepad2.triangle) {
                    CV4B1I.position = 0.3
                } else {
                    CV4B1I.position = 0.05
                }
                clawOSwap.set(false)
                target = 200.0
                var deltaW = 0.0
                if (size > 5000) {
                    if (angle in 90.0..180.0) {
                        deltaW = (90 - (angle - 90)) / 90 / 20 / 2
                        wristPosition -= deltaW
                    } else if (angle in 0.0..90.0) {
                        deltaW = angle / 90 / 20 / 2
                        wristPosition += deltaW
                    }
                }
                if (horz.currentPosition.toDouble() < 150) {
                    wristPosition = 0.5
                }
                if (abs(gamepad2.left_stick_x) > 0.02) {
                    wristPosition = (gamepad2.left_stick_x.toDouble() + 1) / 2
                }
                if (gamepad2.cross) {
                    timer.reset()
                    CV4B0I.position = 0.85
                } else {
                    CV4B0I.position = 0.63
                }

                if (gamepad2.square) {
                    state = State.GRABBED_SAMPLE
                    timeSinceStateChange.reset()
                }
                //wrist.position = (gamepad2.left_stick_x.toDouble() + 1.0) / 2
            }
            State.GRABBED_SAMPLE -> {
                if (timeSinceStateChange.seconds() > 0.2) {
                    target = 0.0
                }
                if (gamepad2.start) {
                    target = 100.0
                }
                CV4B0I.position = 0.0
                CV4B1I.position = 0.8

                wristPosition = 0.0
                //wristPosition = 0.5
                if (gamepad2.dpad_left) {
                    clawOSwap.set(true)
                    clawISwap.set(false)
                    state = State.TRANSFERRED

                    timeSinceStateChange.reset()
                }
                if (gamepad2.circle) {
                    state = State.GRABBING_SAMPLE
                }
            }

            State.TRANSFERRED -> {
                if (timeSinceStateChange.seconds() > 0.1) {
                    target = 50.0
                    CV4B0O.position = 0.3
                    CV4B1O.position = 0.8
                }
                if (gamepad1.left_bumper || gamepad2.dpad_up) {
                    clawOSwap.set(false)
                    state = State.IDLE
                    timeSinceStateChange.reset()
                }
            }

            State.IDLE -> {
                CV4B0I.position = 0.0
                CV4B1I.position = 0.8
                wristO.position = 0.41
                if (timeSinceStateChange.seconds() > 0.2) {
                    CV4B0O.position = 0.6
                    CV4B1O.position = 0.34
                }

                target = 50.0
            }

            State.GRABBING_SPECIMEN -> {
                CV4B0O.position = 0.0
                target = 0.0
                if (gamepad2.left_bumper) {
                    clawOSwap.set(true)
                    state = State.GRABBED_SPECIMEN
                    timeSinceStateChange.reset()
                } else {
                    wristO.position = 0.41
                }
                if (gamepad1.left_bumper || gamepad2.dpad_up) {
                    CV4B1O.position = 1.0
                } else {
                    CV4B1O.position = 0.9
                }
            }
            State.GRABBED_SPECIMEN -> {
                if (timeSinceStateChange.seconds() > 1.5) {
                    wristO.position = 0.98
                }
                //CV4B0O.position = 0.196
                //CV4B1O.position = 0.6
                if (gamepad2.left_bumper && timeSinceStateChange.seconds() > 2.0) {
                    state = State.IDLE
                    timeSinceStateChange.reset()
                }
            }

            State.EXPANDED -> {
                if (randomDetector.risingEdge() && timeSinceStateChange.seconds() > 0.5) {
                    state = State.IDLE
                }
                CV4B0O.position = 0.0
                CV4B1O.position = 0.9

                CV4B0I.position = 0.6
                CV4B1I.position = 0.05

                target = 200.0
            }
            State.HANGING -> {
                if (randomDetector.risingEdge() && timeSinceStateChange.seconds() > 0.5) {
                    state = State.IDLE
                }
                target = 50.0
                vert0.power = 0.35
                vert1.power = 0.35
                CV4B0I.position = 0.0
                CV4B1I.position = 0.8

                CV4B0O.position = 0.6
                CV4B1O.position = 0.34
            }
        }
        telemetry.addData("State", state)
        telemetry.addData("Wrist Position", wristPosition)
        telemetry.addData("Block Angle", angle)
        telemetry.addData("Block Size", size)
        telemetry.addData("Time", timeSinceStateChange.seconds())
        telemetry.addData("Horz Encoder", horz.currentPosition)
        if (state != State.HANGING) {
            if (vert0.currentPosition > 20 && !gamepad1.dpad_down) {
                vert0.power = -0.2
                vert1.power = -0.2
            } else {
                vert0.power = gamepad2.right_stick_y.toDouble()
                vert1.power = gamepad2.right_stick_y.toDouble()
            }
        }
        if (gamepad1.dpad_right) {
            vert0.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            vert1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            vert0.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            vert1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        telemetry.addData("Vertical Power", vert0.power)
        telemetry.addData("Vert pos", vert0.currentPosition)
        telemetry.addData("Horz Power", horz.power)
        telemetry.addData("Time since state change", timeSinceStateChange.seconds())
        //telemetry.addData("FPS", visionPortal.fps)

    }
}