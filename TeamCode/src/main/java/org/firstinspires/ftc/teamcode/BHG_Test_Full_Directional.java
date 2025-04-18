package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "BHG_Test_Full_Directional", group = "TeleOp")
public class BHG_Test_Full_Directional extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;

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
        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);
        sleep(3000);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        // Wait for driver to press the start button
        waitForStart();

        while (opModeIsActive()) {
            // Get gamepad input
            if (gamepad1.x) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);

                break;
            }
            double forward = gamepad1.left_stick_y; // Forward/Backward
            double turn = gamepad1.left_stick_x;   // Left/Right turning
            // Calculate motor power
            boolean leftTrigger = gamepad1.left_bumper;
            boolean rightTrigger = gamepad1.right_bumper;
            double leftPower = forward + turn;
            double rightPower = forward - turn;

            // Apply power to motors
            if (leftTrigger) {
                if (rightTrigger) {
                    leftDrive.setPower(leftPower * 1);
                    rightDrive.setPower(rightPower * 1);
                } else {
                    leftDrive.setPower(leftPower * 0.25);
                    rightDrive.setPower(rightPower * 0.25);
                }
            } else if (!leftTrigger) {
                leftDrive.setPower(leftPower * 0.5);
                rightDrive.setPower(rightPower * 0.5);
            }
            TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
            telemetry.addData("Status", "Waiting for Start");
            telemetry.update();
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
            telemetry.addData("Touch Sensor", isPressed ? "PRESSED" : "NOT PRESSED");
            telemetry.update();

            // Send telemetry data
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();

        }
    }
}