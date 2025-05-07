package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.LLStatus
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp
class SensorLimelight3A : LinearOpMode() {
    private var limelight: Limelight3A? = null

    @Throws(InterruptedException::class)
    public override fun runOpMode() {
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")

        telemetry.setMsTransmissionInterval(11)

        limelight!!.pipelineSwitch(0)

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight!!.start()

        telemetry.addData(">", "Robot Ready.  Press Play.")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            val status: LLStatus = limelight!!.getStatus()
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

                    // Access color results
                    val colorResults: MutableList<LLResultTypes.ColorResult> = result.getColorResults()
                    for (cr in colorResults) {
                        //telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees())
                    }
                }
            } else {
                //telemetry.addData("Limelight", "No data available")
            }

            telemetry.update()
        }
        limelight!!.stop()
    }
}
