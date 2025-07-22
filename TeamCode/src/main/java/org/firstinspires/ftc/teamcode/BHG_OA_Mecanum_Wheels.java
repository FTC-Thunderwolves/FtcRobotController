package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Ben Object Avoidance Mecanum Wheels", group = "Autonomous")
public class BHG_OA_Mecanum_Wheels extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight  = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        // Set motor direction (adjust based on physical setup)

        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);

            if (Double.isNaN(distance)) {
                telemetry.addData("Distance", "Invalid");
                telemetry.update();
                continue; // Skip loop iteration if the distance is invalid
            }
            double x = 0;
            double y = 0;

            if (distance > 12) {
                x = -0.35;
                y = -0.35;
            } else if (distance <= 12) {
               x = 0.25;
               y = 0.25;
               sleep(1500);
               x = -0.5;
               y = 0;
               sleep(1000);
            }
            frontLeft.setPower(x);
            frontRight.setPower(y);
            backLeft.setPower(x);
            backRight.setPower(y);
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
    }
}
