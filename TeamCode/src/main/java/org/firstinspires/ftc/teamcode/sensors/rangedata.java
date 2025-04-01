package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
@Disabled
public class rangedata extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        LaserRangefinder left = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "leftRange"));
        LaserRangefinder right = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "rightRange"));
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Distance Left", left.getDistance(DistanceUnit.MM));
            telemetry.addData("Status Left", left.getStatus());
            telemetry.addData("Distance Right", right.getDistance(DistanceUnit.MM));
            telemetry.addData("Status Right", right.getStatus());
            telemetry.update();
        }
    }
}
