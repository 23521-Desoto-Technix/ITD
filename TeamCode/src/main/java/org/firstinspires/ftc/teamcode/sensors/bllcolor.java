package org.firstinspires.ftc.teamcode.sensors;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class bllcolor extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "Color");

        waitForStart();
        while (opModeIsActive()) {
            // read all 3 color channels in one I2C transmission:
            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
