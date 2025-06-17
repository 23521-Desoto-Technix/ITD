package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathgen.BezierCurve
import com.pedropathing.pathgen.BezierLine
import com.pedropathing.util.Constants
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.IntakeV2
import org.firstinspires.ftc.teamcode.subsystems.Outtake
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlides
import org.firstinspires.ftc.teamcode.utils.Detector
import org.firstinspires.ftc.teamcode.utils.PID
import pedroPathing.constants.FConstants
import pedroPathing.constants.LConstants
import kotlin.math.abs
import kotlin.math.roundToInt
import kotlin.math.tan

@Autonomous(name = "5 Specimen")
class `5specimen` : LinearOpMode() {

    enum class Mode {
        RED,
        BLUE,
        FIVE
    }

    override fun runOpMode() {
        Constants.setConstants(FConstants::class.java, LConstants::class.java)
        val follower = Follower(hardwareMap)

        val spikeOnePose = Pose(68.0, 22.0, Math.toRadians(0.0))
        val pushOnePose = Pose(10.0, 22.0, Math.toRadians(0.0))
        val spikeTwoPose = Pose(53.0, 16.0, Math.toRadians(0.0))
        val pushTwoPose = Pose(20.0, 16.0, Math.toRadians(0.0))
        val spikeThreePose = Pose(53.0, 8.2, Math.toRadians(0.0))
        val pushThreePose = Pose(0.0, 9.0, Math.toRadians(0.0))

        val startPose = Pose(6.0, 66.0, Math.toRadians(0.0))
        val scorePose = Pose(42.0, 70.0, Math.toRadians(0.0))
        val pickupPose = Pose(4.0, 37.0)
        val telemetryA: Telemetry =
            MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())
        follower.setStartingPose(startPose)
        val leftRGB = hardwareMap.servo["LeftRGB"]
        val rightRGB = hardwareMap.servo["RightRGB"]
        leftRGB.position = 0.5
        rightRGB.position = 0.5

        val intake = IntakeV2(hardwareMap)
        val outtake = Outtake(hardwareMap)
        intake.update(Intake.state.IDLE)
        outtake.update(Outtake.state.INIT)
        val vslides = VerticalSlides(hardwareMap, telemetry)
        val hslides = HorizontalSlides(hardwareMap, telemetry)
        val backTouch = hardwareMap.touchSensor.get("backTouch")
        val leftTouch = hardwareMap.touchSensor.get("leftTouch")
        val rightTouch = hardwareMap.touchSensor.get("rightTouch")

        val leftFront = hardwareMap.get(DcMotor::class.java, "frontLeft")
        val rightFront = hardwareMap.get(DcMotor::class.java, "frontRight")
        val leftRear = hardwareMap.get(DcMotor::class.java, "backLeft")
        val rightRear = hardwareMap.get(DcMotor::class.java, "backRight")

        hslides.resetEncoder()
        vslides.resetEncoder()
        val outtakeClaw = hardwareMap.servo["outtakeClaw"]
        val intakeClaw = hardwareMap.servo["intakeClaw"]
        outtakeClaw.position = 0.58
        intakeClaw.position = 0.85
        val dig0 = hardwareMap.get(DigitalChannel::class.java, "dig0")
        val dig1 = hardwareMap.get(DigitalChannel::class.java, "dig1")
        val allHubs = hardwareMap.getAll(
            LynxModule::class.java
        )

        var mode = Mode.FIVE

        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        fun update() {
            vslides.update()
            hslides.update()
            follower.update()
            //follower.telemetryDebug(telemetryA)
            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            for (hub in allHubs) {
                hub.clearBulkCache()
            }
            telemetry.update()
        }

        val ll2 = hardwareMap.get(DigitalChannel::class.java, "limelightlight")
        ll2.mode = DigitalChannel.Mode.OUTPUT
        ll2.state = false

        val headingPID = PID(2.0, 0.0, 0.1)

        val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(0)

        val path1 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    startPose,
                    Pose(42.0, 66.0, Math.toRadians(0.0))
                )
            ).build()
        val path2 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    scorePose,
                    Pose(15.0, 42.0, Math.toRadians(0.0)),
                    spikeOnePose,
                ),
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path3 = follower.pathBuilder()
            .addPath(
                BezierLine(
                    Pose(70.0, 22.0, Math.toRadians(0.0)),
                    pushOnePose
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path4 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(10.0, 22.0, Math.toRadians(0.0)),
                    spikeOnePose,
                    spikeTwoPose
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path5 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(60.0, 16.0, Math.toRadians(0.0)),
                    pushTwoPose
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path6 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    pushTwoPose,
                    spikeTwoPose,
                    spikeThreePose
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path7 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    spikeThreePose,
                    pushThreePose
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val path8 = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    Pose(5.0, 7.0),
                    Pose(12.0, 50.0),
                    scorePose,
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val pick = follower.pathBuilder()
            .addPath(
                BezierCurve(
                    scorePose,
                    Pose(30.0, 70.0),
                    Pose(30.0, 30.0),
                    pickupPose,
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        val score = follower.pathBuilder()
            .addPath(
                BezierLine(
                    pickupPose,
                    scorePose,
                )
            )
            .setConstantHeadingInterpolation(0.0)
            .build()
        follower.followPath(path1)

        while (opModeInInit()) {
            if (gamepad1.dpad_up) {
                mode = Mode.RED
            }
            if (gamepad1.dpad_down) {
                mode = Mode.BLUE
            }
            if (gamepad1.dpad_left) {
                mode = Mode.FIVE
            }
            if (mode == Mode.RED) {
                leftRGB.position = 0.279
                rightRGB.position = 0.279
            } else if (mode == Mode.BLUE) {
                leftRGB.position = 0.6
                rightRGB.position = 0.6
            } else {
                leftRGB.position = 0.5
                rightRGB.position = 0.5
            }
            telemetry.addData("Mode", mode)
            telemetry.update()
        }
        if (mode != Mode.FIVE) {
            limelight.start()
            ll2.state = true
        }
        hslides.setSetpoint(0.0)
        vslides.setSetpoint(-49_000.0)
        follower.setMaxPower(0.8)
        outtake.update(Outtake.state.GRABBED)
        while (!isStopRequested && follower.isBusy) {
            update()
            if (leftTouch.isPressed || rightTouch.isPressed) {
                outtakeClaw.position = 0.8
                break
            }
        }
        if (mode != Mode.FIVE) {
            var milimetersVertical = 0.0
            var milimetersLateral = 0.0
            var angle = 0.0
            var out = false
            var dived = false
            var grabbed = false
            val grabTimer = ElapsedTime()
            val slideTimer = ElapsedTime()
            var success = false
            var readynt = false
            val readyForSlides = Detector()

            val pid = PID(0.15,0.0,0.009)

            val stableDrivetrainTimer = ElapsedTime()
            stableDrivetrainTimer.reset()
            val stableLateralTimer = ElapsedTime()
            stableLateralTimer.reset()
            follower.breakFollowing()
            while (!isStopRequested) {
                if (stableDrivetrainTimer.milliseconds() > 500000) {
                    break
                }
                hslides.update()
                val result: LLResult? = limelight!!.getLatestResult()
                if (result != null) {
                    for (hub in allHubs) {
                        hub.clearBulkCache()
                    }
                    telemetry.update()
                    // Access general information
                    val captureLatency = result.getCaptureLatency()
                    val targetingLatency = result.getTargetingLatency()
                    val parseLatency = result.getParseLatency()
                    telemetry.addData("LL Latency", captureLatency + targetingLatency)
                    telemetry.addData("Parse Latency", parseLatency)
                    telemetry.addData("valid", result.isValid())
                    telemetry.addData("python", result.pythonOutput.get(7))
                    var tx = result.pythonOutput.get(5)
                    var ty = result.pythonOutput.get(6)
                    ty -= 480/2
                    tx -= 640/2
                    ty /= 480/2
                    tx /= 640/2
                    tx *= 54.5
                    ty *= 42.0
                    //tx -= 18.0
                    ty *= -1
                    ty /= 2
                    tx /= 2

                    telemetry.addData("tx", tx)
                    telemetry.addData("ty", ty)

                    if (result.pythonOutput.get(7) != 0.0) {
                        milimetersLateral = ((325 * tan(Math.toRadians(35 + ty))) * tan(Math.toRadians(tx))) - 50
                        if (abs(milimetersLateral) < 2.0 && hslides.getSetpoint() == 0.0 && slideTimer.milliseconds() > 500) {
                            slideTimer.reset()
                        }
                        readyForSlides.update(abs(milimetersLateral) < 15.0 && hslides.getSetpoint() == 0.0)
                        if (readyForSlides.risingEdge() && result.pythonOutput.get(7) != 0.0) {
                            milimetersVertical = (325 * tan(Math.toRadians(35 + ty))) + (325 * tan(Math.toRadians(325.0)))
                        }
                        telemetry.addData("raw", result.pythonOutput.get(7))
                        telemetry.addData("Lateral", milimetersLateral)
                        if (hslides.getSetpoint() == 0.0) {
                            angle = result.pythonOutput.get(7) / 90 / 4
                        }
                        //telemetry.addLine((round(325 * tan(Math.toRadians(35 + ty))) + (325 * tan(Math.toRadians(325.0)))).toString())
                        if (hslides.getSetpoint() == 0.0) {
                            // Simple proportional control for strafing
                            // Adjust Kp as needed
                            var drivePower = pid.calculate(milimetersLateral * 0.3)
                            val minimum = 0.13
                            if (drivePower < minimum && drivePower > 0.0) {
                                drivePower = minimum
                            }
                            if (drivePower > -minimum && drivePower < 0.0) {
                                drivePower = -minimum
                            }

                            telemetry.addData("valid", result.pythonOutput.get(0))
                            if (result.pythonOutput.get(0).toInt() == 0) {
                                leftFront?.power = 0.0
                                rightFront?.power = 0.0
                                leftRear?.power = 0.0
                                rightRear?.power = 0.0
                            } else {
                                // Mecanum drive logic for strafing
                                if (stableDrivetrainTimer.milliseconds() > 500) {
                                    if (!drivePower.isNaN()) {
                                        leftFront?.power = -drivePower + (Math.abs(drivePower) * 0.2).coerceAtLeast(0.2)
                                        rightFront?.power = drivePower + (Math.abs(drivePower) * 0.2).coerceAtLeast(0.2)
                                        leftRear?.power = drivePower + (Math.abs(drivePower) * 0.2).coerceAtLeast(0.2)
                                        rightRear?.power = -drivePower + (Math.abs(drivePower) * 0.2).coerceAtLeast(0.2)
                                    } else {
                                        leftFront?.power = 0.2
                                        rightFront?.power = 0.2
                                        leftRear?.power = 0.2
                                        rightRear?.power = 0.2
                                    }
                                } else {
                                    leftFront?.power = 0.2
                                    rightFront?.power = 0.2
                                    leftRear?.power = 0.2
                                    rightRear?.power = 0.2
                                }
                            }
                        } else {
                        }
                    }
                } else {
                }
                if (readyForSlides.risingEdge() && !readynt) {
                    readynt = true
                    stableLateralTimer.reset()
                }
                if ((stableLateralTimer.milliseconds() > 1000) && readyForSlides.value() && !out) {
                    //125.6 mm circumference
                    //8192 CPR
                    leftFront?.power = 0.0
                    rightFront?.power = 0.0
                    leftRear?.power = 0.0
                    rightRear?.power = 0.0
                    val ticks = 8192.0 * (milimetersVertical / 125.6)
                    if (ticks != 0.0) {
                        hslides.setSetpoint(-9_000.0 - ticks)
                        grabTimer.reset()
                        intake.update(Intake.state.SCANNING)
                        out = true
                    }

                }
                if (grabTimer.milliseconds() > 300 && !dived && out) {
                    intake.update(Intake.state.DIVING)
                    dived = true
                }
                if (grabTimer.milliseconds() > 600 && dived && !grabbed && out) {
                    grabbed = true
                    intakeClaw.position = 0.34
                }
                if (grabTimer.milliseconds() > 900 && dived && out) {
                    intake.update(Intake.state.OUT)
                    hslides.setSetpoint(0.0)
                    success = true
                    break
                }
                if (out) {
                    intake.wrist(0.5 + angle)
                }
                telemetry.addData("Milimeters", milimetersVertical.roundToInt())
                telemetry.addData("setpoint", hslides.getSetpoint().roundToInt())
                telemetry.addData("angle", angle)
            }

            follower.followPath(pick)
            val drop = ElapsedTime()
            while (!isStopRequested && follower.isBusy) {
                if (follower.pose.x < 30.0 && follower.pose.x > 28.0) {
                    vslides.setSetpoint(-26_000.0)
                    outtake.update(Outtake.state.INTAKING)
                    intake.update(Intake.state.PASSTHROUGH_OUTSIDE)
                }
                if (drop.milliseconds() > 2000) {
                    intakeClaw.position = 0.85
                }
                update()
                if (backTouch.isPressed) {
                    outtakeClaw.position = 0.58
                    break
                }
            }
            sleep(100)
            vslides.setSetpoint(-49_000.0)
            follower.setMaxPower(1.0)
            follower.followPath(score)
            while (!isStopRequested && follower.isBusy) {
                update()
                if (follower.pose.x > 12.0) {
                    outtake.update(Outtake.state.GRABBED)
                }
                if (leftTouch.isPressed || rightTouch.isPressed) {
                    outtakeClaw.position = 0.8
                    break
                }
            }
        }
        hslides.setSetpoint(0.0)
        ll2.state = false
        limelight.stop()
        follower.setMaxPower(1.0)
        vslides.setSetpoint(0.0)
        outtake.update(Outtake.state.INIT)
        follower.followPath(path2)
        while (!isStopRequested && follower.isBusy) {
            update()
            telemetry.addData("Y raw", follower.pose.y)
            if (follower.pose.y < 33.0) {
                break
            }
        }
        follower.followPath(path3)
        while (!isStopRequested && follower.isBusy) {
            if (follower.pose.x < 25.0) {
                break
            }
            update()
        }
        follower.followPath(path4)
        while (!isStopRequested && follower.isBusy) {
            if (follower.currentTValue > 0.8) {
                break
            }
            update()
        }
        follower.followPath(path5)
        while (!isStopRequested && follower.isBusy) {
            if (follower.currentTValue > 0.9) {
                break
            }
            update()
        }
        follower.followPath(path6)
        while (!isStopRequested && follower.isBusy) {
            if (follower.currentTValue > 0.9) {
                break
            }
            update()
        }
        follower.followPath(path7)
        vslides.setSetpoint(-26_000.0)
        outtake.update(Outtake.state.INTAKING)
        while (!isStopRequested && follower.isBusy) {
            if (backTouch.isPressed) {
                break
            }
            update()
        }
        follower.setMaxPower(1.0)
        outtakeClaw.position = 0.58
        sleep(100)
        vslides.setSetpoint(-49_000.0)
        follower.followPath(path8)
        while (!isStopRequested && follower.isBusy) {
            if (follower.pose.x > 12.0) {
                outtake.update(Outtake.state.GRABBED)
            }
            update()
            if (leftTouch.isPressed || rightTouch.isPressed) {
                outtakeClaw.position = 0.8
                break
            }
        }
        var counter = 0
        val max = 4
        while (!isStopRequested) {
            counter += 1
            follower.followPath(pick)
            while (!isStopRequested && follower.isBusy) {
                if (follower.pose.x < 35.0 && follower.pose.x > 33.0) {
                    if (counter == max) {
                        vslides.setSetpoint(0.0)
                        outtake.update(Outtake.state.INIT)
                    } else {
                        vslides.setSetpoint(-26_000.0)
                        outtake.update(Outtake.state.INTAKING)
                    }
                }
                update()
                if (backTouch.isPressed) {
                    outtakeClaw.position = 0.58
                    break
                }
            }
            if (counter == max) {
                break
            }
            sleep(100)
            vslides.setSetpoint(-49_000.0)
            follower.setMaxPower(1.0)
            follower.followPath(score)
            while (!isStopRequested && follower.isBusy) {
                update()
                if (follower.pose.x > 12.0) {
                    outtake.update(Outtake.state.GRABBED)
                }
                if (leftTouch.isPressed || rightTouch.isPressed) {
                    outtakeClaw.position = 0.8
                    break
                }
            }
        }
    }
}