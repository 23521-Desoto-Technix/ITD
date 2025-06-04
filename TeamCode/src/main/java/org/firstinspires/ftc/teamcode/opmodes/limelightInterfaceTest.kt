package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLStatus
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.IntakeV2
import org.firstinspires.ftc.teamcode.utils.Detector
import org.firstinspires.ftc.teamcode.utils.PID
import org.firstinspires.ftc.teamcode.utils.ServoSwap
import kotlin.math.abs
import kotlin.math.roundToInt
import kotlin.math.tan

@TeleOp
class limelight : LinearOpMode() {
    private var limelight: Limelight3A? = null
    private var leftFront: DcMotor? = null
    private var rightFront: DcMotor? = null
    private var leftRear: DcMotor? = null
    private var rightRear: DcMotor? = null

    @Throws(InterruptedException::class)
    public override fun runOpMode() {
        val pid = PID(0.15,0.0,0.009)
        val hslides = HorizontalSlides(hardwareMap, telemetry)
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        leftFront = hardwareMap.get(DcMotor::class.java, "frontLeft")
        rightFront = hardwareMap.get(DcMotor::class.java, "frontRight")
        leftRear = hardwareMap.get(DcMotor::class.java, "backLeft")
        rightRear = hardwareMap.get(DcMotor::class.java, "backRight")

        telemetry.setMsTransmissionInterval(11)
        val intakeClaw = hardwareMap.servo["intakeClaw"]
        val intakeSS = ServoSwap(intakeClaw, 0.85, 0.35)
        intakeSS.update(true)

        limelight!!.pipelineSwitch(0)
        val readyForSlides = Detector()
        val intake = IntakeV2(hardwareMap)
        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight!!.start()

        telemetry.addData(">", "Robot Ready.  Press Play.")
        telemetry.update()
        hslides.resetEncoder()
        intake.update(Intake.state.CAMERA)
        intake.wrist(0.25)
        hslides.setSetpoint(0.0)
        var milimetersVertical = 0.0
        var milimetersLateral = 0.0
        var angle = 0.0
        var out = false
        var dived = false
        var grabbed = false
        val grabTimer = ElapsedTime()
        val slideTimer = ElapsedTime()
        intakeSS.swap()

        waitForStart()
        intakeSS.swap()
        while (opModeIsActive()) {
            val status: LLStatus = limelight!!.getStatus()
            hslides.update()
            telemetry.addData(
                "Name",
                status.getName()
            )
            telemetry.addData(
                "index", status.getPipelineIndex()
            )
            telemetry.addData("type",status.getPipelineType())
            val result: LLResult? = limelight!!.getLatestResult()
            if (result != null) {
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
                    if (abs(milimetersLateral) < 5.0 && hslides.getSetpoint() == 0.0 && slideTimer.milliseconds() > 500) {
                        slideTimer.reset()
                    }
                    readyForSlides.update(abs(milimetersLateral) < 5.0 && hslides.getSetpoint() == 0.0 && slideTimer.milliseconds() > 200)
                    if (readyForSlides.risingEdge() && result.pythonOutput.get(7) != 0.0) {
                        milimetersVertical = (325 * tan(Math.toRadians(35 + ty))) + (325 * tan(Math.toRadians(325.0)))
                    }
                    telemetry.addData("raw", result.pythonOutput.get(7))
                    telemetry.addData("Lateral", milimetersLateral)
                    if (hslides.getSetpoint() == 0.0) {
                        angle = result.pythonOutput.get(7) / 90 / 4
                    }
                    //telemetry.addLine((round(325 * tan(Math.toRadians(35 + ty))) + (325 * tan(Math.toRadians(325.0)))).toString())
                    if (gamepad1.a && hslides.getSetpoint() == 0.0) {
                        // Simple proportional control for strafing
                        // Adjust Kp as needed
                        var drivePower = pid.calculate(milimetersLateral * 0.3)
                        val minimum = 0.17
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
                            leftFront?.power = drivePower - (Math.abs(drivePower) * 0.2).coerceAtLeast(0.1)
                            rightFront?.power = drivePower + (Math.abs(drivePower) * 0.2).coerceAtLeast(0.1)
                            leftRear?.power = -drivePower - (Math.abs(drivePower) * 0.2).coerceAtLeast(0.1)
                            rightRear?.power = -drivePower + (Math.abs(drivePower) * 0.2).coerceAtLeast(0.1)
                        }
                    } else {
                        leftFront?.power = -0.1
                        rightFront?.power = 0.1
                        leftRear?.power = -0.1
                        rightRear?.power = 0.1
                    }
                }
            } else {
                // Stop motors if no valid target or gamepad1.a is not pressed
                leftFront?.power = 0.0
                rightFront?.power = 0.0
                leftRear?.power = 0.0
                rightRear?.power = 0.0
                //telemetry.addData("Limelight", "No data available")
            }
            if (readyForSlides.risingEdge() && gamepad1.a && abs(pid.derivative) < 75.0) {
                //125.6 mm circumference
                //8192 CPR
                val ticks = 8192.0 * (milimetersVertical / 125.6)
                if (ticks != 0.0) {
                    hslides.setSetpoint(-10_000.0 - ticks)
                    grabTimer.reset()
                    intake.update(Intake.state.SCANNING)
                    out = true
                }

            }
            if (grabTimer.milliseconds() > 300 && !dived && out) {
                intake.update(Intake.state.DIVING)
                dived = true
            }
            if (grabTimer.milliseconds() > 500 && dived && !grabbed && out) {
                intakeSS.swap()
                grabbed = true
            }
            if (grabTimer.milliseconds() > 600 && dived && out) {
                intake.update(Intake.state.OUT)
            }
            if (grabTimer.milliseconds() > 1600 && dived && out) {
                dived = false
                out = false
                grabbed = false
                intakeSS.swap()
                intake.update(Intake.state.CAMERA)
                hslides.setSetpoint(0.0)
                intake.wrist(0.25)
            }
            telemetry.addData("error", abs(hslides.pid.error))
            telemetry.addData("speed", hslides.pid.derivative)
            if (out) {
                intake.wrist(0.5 + angle)
            }
            telemetry.addData("Milimeters", milimetersVertical.roundToInt())
            telemetry.addData("setpoint", hslides.getSetpoint().roundToInt())
            telemetry.addData("angle", angle)
            telemetry.update()
        }
        limelight!!.stop()
    }
}
