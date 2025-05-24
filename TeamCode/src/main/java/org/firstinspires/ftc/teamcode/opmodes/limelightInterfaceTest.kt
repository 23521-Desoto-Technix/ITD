package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.LLStatus
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlides
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.IntakeV2
import org.firstinspires.ftc.teamcode.utils.Detector
import org.firstinspires.ftc.teamcode.utils.PID
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
        val pid = PID(0.12,0.0,0.006)
        val hslides = HorizontalSlides(hardwareMap, telemetry)
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        leftFront = hardwareMap.get(DcMotor::class.java, "frontLeft")
        rightFront = hardwareMap.get(DcMotor::class.java, "frontRight")
        leftRear = hardwareMap.get(DcMotor::class.java, "backLeft")
        rightRear = hardwareMap.get(DcMotor::class.java, "backRight")

        telemetry.setMsTransmissionInterval(11)

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
        waitForStart()
        hslides.setSetpoint(0.0)
        var milimeters = 0.0
        while (opModeIsActive()) {
            intake.update(Intake.state.SCANNING)
            val status: LLStatus = limelight!!.getStatus()
            hslides.update()
            /*telemetry.addData(
                "Name", "%s",
                status.getName()
            )
            telemetry.addData(
                "LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), status.getFps()
            )
            telemetry.addData(
                "Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType()
            )*/

            val result: LLResult? = limelight!!.getLatestResult()
            if (result != null) {
                // Access general information
                val captureLatency = result.getCaptureLatency()
                val targetingLatency = result.getTargetingLatency()
                val parseLatency = result.getParseLatency()
                //telemetry.addData("LL Latency", captureLatency + targetingLatency)
                //telemetry.addData("Parse Latency", parseLatency)

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx())
                    telemetry.addData("txnc", result.getTxNC())
                    telemetry.addData("ty", result.getTy())
                    telemetry.addData("tync", result.getTyNC())
                    if (readyForSlides.risingEdge() && result.ty != 0.0) {
                        milimeters = (325 * tan(Math.toRadians(35 + result.getTy()))) + (325 * tan(Math.toRadians(325.0)))
                    }
                    readyForSlides.update((abs(result.tx) < 0.7 || gamepad1.b) && result.isValid())
                    if (gamepad1.a && hslides.getSetpoint() > -2_000.0) {
                        val tx = result.getTx()
                        // Simple proportional control for strafing
                        // Adjust Kp as needed
                        val drivePower = pid.calculate(tx)

                        // Mecanum drive logic for strafing
                        leftFront?.power = drivePower - (Math.abs(drivePower) * 0.2).coerceAtLeast(0.1)
                        rightFront?.power = drivePower + (Math.abs(drivePower) * 0.2).coerceAtLeast(0.1)
                        leftRear?.power = -drivePower - (Math.abs(drivePower) * 0.2).coerceAtLeast(0.1)
                        rightRear?.power = -drivePower + (Math.abs(drivePower) * 0.2).coerceAtLeast(0.1)
                    } else {
                        leftFront?.power = -0.1
                        rightFront?.power = 0.1
                        leftRear?.power = -0.1
                        rightRear?.power = 0.1
                    }

                    // Access color results
                    val colorResults: MutableList<LLResultTypes.ColorResult> = result.getColorResults()
                    for (cr in colorResults) {
                        //telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees())
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
            if (readyForSlides.risingEdge() && gamepad1.a) {
                //125.6 mm circumference
                //8192 CPR
                val ticks = 8192.0 * (milimeters / 125.6)
                if (ticks != 0.0) {
                    hslides.setSetpoint(-11_000.0 - ticks)
                }
            }
            telemetry.addData("Milimeters", milimeters.roundToInt())
            telemetry.addData("setpoint", hslides.getSetpoint().roundToInt())
            telemetry.update()
        }
        limelight!!.stop()
    }
}
