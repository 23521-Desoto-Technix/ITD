package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.localization.GoBildaPinpointDriver
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Outtake
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides
import org.firstinspires.ftc.teamcode.utils.Detector
import org.firstinspires.ftc.teamcode.utils.ServoSwap
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin


@TeleOp
class v2 : LinearOpMode() {
    enum class SlideMode {
        NORMAL,
        MANUAL,
        INSPECTION,
    }
    enum class State {
        SCANNING,
        OUT,
        TRANSFERRING_SAM,
        TRANSFERED_SAM,
        DROPPING_SAM,
        IDLE,
        INTAKING_SPEC,
        GRABBED_SPEC,
        DELIVERING_SPEC,
        LOCKED
    }
    val man = Detector()
    val tssc = ElapsedTime()

    var state = State.IDLE
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val frontLeft = hardwareMap.dcMotor["frontLeft"]
        val backLeft = hardwareMap.dcMotor["backLeft"]
        val frontRight = hardwareMap.dcMotor["frontRight"]
        val backRight = hardwareMap.dcMotor["backRight"]
        val leftRGB = hardwareMap.servo["LeftRGB"]
        val rightRGB = hardwareMap.servo["RightRGB"]
        val intakeClaw = hardwareMap.servo["intakeClaw"]
        val intakeSS = ServoSwap(intakeClaw, 0.68, 0.4)
        val outtakeClaw = hardwareMap.servo["outtakeClaw"]
        val outtakeSS = ServoSwap(outtakeClaw, 0.53, 1.0)
        val odo = hardwareMap.get(GoBildaPinpointDriver::class.java, "odo")
        val vslides = VerticalSlides(hardwareMap, telemetry)
        val hslides = HorizontalSlides(hardwareMap, telemetry)
        val intake = Intake(hardwareMap)
        val outtake = Outtake(hardwareMap)
        //TODO
        var slideMode = SlideMode.NORMAL

        val allHubs = hardwareMap.getAll(
            LynxModule::class.java
        )
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        val locker = Detector()


        frontRight.direction = DcMotorSimple.Direction.REVERSE
        backRight.direction = DcMotorSimple.Direction.REVERSE

        leftRGB.position = 0.5
        rightRGB.position = 0.5

        odo.resetPosAndIMU();
        waitForStart()

        if (isStopRequested) return

        while (opModeIsActive()) {
            if (slideMode == SlideMode.MANUAL) {
                leftRGB.position = 0.35
                rightRGB.position = 0.35
            } else {
                leftRGB.position = 1.0
                rightRGB.position = 1.0
            }
            odo.update()
            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick value is reversed
            val x = gamepad1.left_stick_x.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()

            if (gamepad1.options) {
                odo.resetPosAndIMU()
            }
            if (man.risingEdge()) {
                slideMode = when (slideMode) {
                    SlideMode.NORMAL -> SlideMode.MANUAL
                    SlideMode.MANUAL -> SlideMode.NORMAL
                    SlideMode.INSPECTION -> TODO()
                }
            }
            man.update(gamepad2.options)
            val botHeading = odo.heading

            var rotX = x * cos(-botHeading) - y * sin(-botHeading)
            val rotY = x * sin(-botHeading) + y * cos(-botHeading)
            rotX *= 1.1
            val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
            var mult = 1.0
            if (gamepad1.right_bumper) {
                mult = 0.5
                frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            } else {
                frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                backLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                backRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            }
            val frontLeftPower = (rotY + rotX + rx) / denominator * mult
            val backLeftPower = (rotY - rotX + rx) / denominator * mult
            val frontRightPower = (rotY - rotX - rx) / denominator * mult
            val backRightPower = (rotY + rotX - rx) / denominator * mult

            frontLeft.power = frontLeftPower
            backLeft.power = backLeftPower
            frontRight.power = frontRightPower
            backRight.power = backRightPower

            if (state == State.LOCKED) {
                vslides.setPower(0.45)
                hslides.update()
            }
            else if (slideMode == SlideMode.NORMAL) {
                vslides.update()
                hslides.update()

            } else if (slideMode == SlideMode.MANUAL) {
                vslides.setPower(gamepad2.left_stick_y.toDouble())
                hslides.setPower(gamepad2.right_stick_y.toDouble())
            } else {
                TODO()
            }
            if (gamepad2.left_trigger > 0.9 && slideMode == SlideMode.MANUAL) {
                vslides.resetEncoder()
            }
            if (gamepad2.right_trigger > 0.9 && slideMode == SlideMode.MANUAL) {
                hslides.resetEncoder()
            }
            locker.update(gamepad2.ps)
            if (locker.risingEdge()) {
                if (state == State.LOCKED) {
                    state = State.IDLE
                } else {
                    state = State.LOCKED
                }
            }
            if (gamepad2.square) {
                tssc.reset()
                state = State.TRANSFERRING_SAM
            }
            if (gamepad2.circle) {
                tssc.reset()
                state = State.SCANNING

            }
            if (gamepad2.triangle) {
                tssc.reset()
                state = State.OUT
            }
            if (gamepad2.dpad_left) {
                tssc.reset()
                state = State.INTAKING_SPEC
                outtakeSS.set(false)
            }
            when (state) {
                State.SCANNING -> {
                    vslides.setSetpoint(0.0)
                    intake.wrist((gamepad2.left_stick_x.toDouble()+1)/2)
                    hslides.setSetpoint(-23_000.0)
                    outtake.update(Outtake.state.TRANSFERING)
                    if (gamepad2.cross) {
                        intake.update(Intake.state.DIVING)
                    } else {
                        intake.update(Intake.state.SCANNING)
                    }
                }
                State.TRANSFERRING_SAM -> {
                    vslides.setSetpoint(0.0)
                    outtakeSS.set(false)
                    outtake.update(Outtake.state.TRANSFERING)
                    intake.update(Intake.state.TRANSFERING)
                    if (tssc.seconds() > 0.1) {
                        hslides.setSetpoint(0_200.0)
                    }
                    if (gamepad2.dpad_right) {
                        tssc.reset()
                        state = State.TRANSFERED_SAM
                    }
                    /*
                    if (hslides.encoder.currentPosition > -1_000.0){
                        hslides.setSetpoint(0.0)
                    } else {
                        hslides.setSetpoint(-1_000.0)
                    }*/
                }

                State.OUT -> {
                    hslides.setSetpoint(-23_000.0)
                    vslides.setSetpoint(0.0)
                    intake.update(Intake.state.OUT)
                }
                State.TRANSFERED_SAM -> {
                    vslides.setSetpoint(0.0)
                    if (tssc.seconds() > 0.1) {
                        intakeSS.set(false)
                        outtake.update(Outtake.state.TRANSFERED)
                    } else {
                        outtakeSS.set(true)
                    }
                    if (gamepad2.dpad_up) {
                        tssc.reset()
                        state = State.DROPPING_SAM
                    }
                }

                State.DROPPING_SAM -> {
                    vslides.setSetpoint(-103_000.0)
                    if (gamepad2.dpad_down) {
                        tssc.reset()
                        state = State.IDLE
                    }
                }

                State.IDLE -> {
                    vslides.setSetpoint(0.0)
                    hslides.setSetpoint(-1_000.0)
                    intake.update(Intake.state.IDLE)
                    outtake.update(Outtake.state.TRANSFERING)
                }
                State.INTAKING_SPEC -> {
                    vslides.setSetpoint(0.0)
                    hslides.setSetpoint(-1_000.0)
                    intake.update(Intake.state.IDLE)
                    outtake.update(Outtake.state.INTAKING)
                    if (gamepad2.left_bumper) {
                        tssc.reset()
                        state = State.GRABBED_SPEC
                    }
                }

                State.GRABBED_SPEC -> {
                    if (tssc.seconds() > 0.2) {
                        vslides.setSetpoint(-33_000.0)
                        outtake.update(Outtake.state.GRABBED)
                    }
                    if (gamepad2.dpad_up) {
                        tssc.reset()
                        state = State.DELIVERING_SPEC
                    }
                }
                State.DELIVERING_SPEC -> {
                    vslides.setSetpoint(-52_000.0)
                    if (tssc.seconds() > 0.4) {
                        outtakeSS.set(false)
                        tssc.reset()
                        state = State.IDLE
                    }
                }
                State.LOCKED -> {
                    hslides.setSetpoint(-1_000.0)
                    intake.update(Intake.state.IDLE)
                    outtake.update(Outtake.state.TRANSFERING)
                }
            }
            intakeSS.update(gamepad2.right_bumper)
            outtakeSS.update(gamepad2.left_bumper)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.addData("State", state)
            telemetry.addData("heading", botHeading)
            telemetry.update()
        }
    }
}