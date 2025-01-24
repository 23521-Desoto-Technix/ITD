package org.firstinspires.ftc.teamcode.sensors;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class bllcolor extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        DigitalChannel dig0 = hardwareMap.digitalChannel.get("dig0");
        DigitalChannel dig1 = hardwareMap.digitalChannel.get("dig1");

        waitForStart();
        while (opModeIsActive()) {
            // read all 3 color channels in one I2C transmission:
            telemetry.addData("digital 0", dig0.getState());
            telemetry.addData("digital 1", dig1.getState());
            telemetry.update();
        }
    }
}
