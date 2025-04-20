package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "BHG_Test_Full_Directional", group = "TeleOp")
public class BHG_Test_Full_Directional extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    TouchSensor touchSensor; //Have to do this somewhere I guess....
    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Set motor direction (adjust based on physical setup)
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for driver to press the start button
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor"); //You could instead do TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


            // Get gamepad input
            if (gamepad1.x) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);

                break;
            }
            double forward = gamepad1.left_stick_y; // Forward/Backward
            double turn = gamepad1.left_stick_x;   // Left/Right turning
            // Calculate motor power
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            double leftPower = forward + turn;
            double rightPower = forward - turn;

            // Apply power to motors
            if (leftBumper && rightBumper) {
                    leftDrive.setPower(leftPower * 1);
                    rightDrive.setPower(rightPower * 1);
            } else if (leftBumper) {
                leftDrive.setPower(leftPower * 0.25);
                rightDrive.setPower(rightPower * 0.25);
            }  else {
                leftDrive.setPower(leftPower * 0.5);
                rightDrive.setPower(rightPower * 0.5);
            }


            // Check if the sensor is pressed
            boolean isPressed = touchSensor.isPressed();
            if (isPressed) {
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
                sleep(3000);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }


            // Display on Driver Station
            handleDistance();
            telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Touch Sensor", isPressed ? "PRESSED" : "NOT PRESSED");

            // Send telemetry data

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();

        }
    }
    private void handleDistance() {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        while (distance < 12 && opModeIsActive()) {
            leftDrive.setPower(0.7);
            rightDrive.setPower(0.7);
            sleep(500);
            leftDrive.setPower(0.5);
            sleep(1000);

            // Update the distance for the next iteration
            distance = distanceSensor.getDistance(DistanceUnit.INCH);
        }
    }
}