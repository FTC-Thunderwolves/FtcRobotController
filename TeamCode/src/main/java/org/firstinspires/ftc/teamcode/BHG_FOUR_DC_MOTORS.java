package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Ben 4 Motors", group = "Teleop")
public class BHG_FOUR_DC_MOTORS extends LinearOpMode{
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        hardwareStart();
        double speed = 0.5;

        waitForStart();

        while (opModeIsActive()) {
            distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

            double forward = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x/2;
            double strafe = gamepad1.left_stick_x*1.5;

            double frontLeftPower = (forward + turn + strafe) * speed;
            double frontRightPower = (forward - turn - strafe) * speed;
            double backLeftPower = (forward - turn + strafe) * speed;
            double backRightPower = (forward + turn - strafe) * speed;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            if(gamepad1.right_trigger > 0) {
                speed = 1;
            } else if(gamepad1.left_stick_button) {
                speed = 1;
            } else if(gamepad1.right_bumper) {
                strafe = strafe/1.5;
            }

            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
            if (Double.isNaN(distance)) {
                telemetry.addData("Distance", "Invalid");
                telemetry.update();
                continue;
            }
            if(distance <= 12) {
                speed = 0.25; //Makes the robot slower so I can move it away from stuff
            }

            if(speed > 0.5) {
                telemetry.addData("State of Speed", "Boosted");
                telemetry.update();
            } else if(speed < 0.5) {
                telemetry.addData("State of Speed", "Diminshed");
                telemetry.update();
            } else {
                telemetry.addData("State of Speed", "Normal");
                telemetry.update();
            }

            telemetry.addData("Speed Multiplier Level", speed*2);
            telemetry.update();

        }
    }
    private void hardwareStart() {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight  = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status","Initialized");
        telemetry.update();
    }
}
