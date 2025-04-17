package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "JLG_Full_Directional", group = "TeleOp")
public class JLG_Full_Directional extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        //Resetting and Initializing Encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motor direction (adjust based on physical setup)
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for driver to press the start button
        waitForStart();

        while (opModeIsActive()) {
            // Get gamepad input
            double forward = gamepad1.left_stick_y; // Forward/Backward
            double turn = gamepad1.right_stick_x;   // Left/Right turning

            // Get encoder counts
            int leftEncoder = leftDrive.getCurrentPosition();
            int rightEncoder = rightDrive.getCurrentPosition();

            // Calculate motor power
            double leftPower = forward + turn;
            double rightPower = forward - turn;

            // Apply power to motors
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            //Display Encoder Values on Drive Station
            telemetry.addData("Left Motor Encoder", leftEncoder);
            telemetry.addData("Right Motor Encoder", rightEncoder);

            // Send telemetry data
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
    }
}
