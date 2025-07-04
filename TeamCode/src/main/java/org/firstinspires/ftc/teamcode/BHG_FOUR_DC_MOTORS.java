package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Ben 4 Motors", group = "Teleop")
public class BHG_FOUR_DC_MOTORS extends LinearOpMode{
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    @Override
    public void runOpMode() {
        hardwareStart();


        waitForStart();

        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x/2;
            double strafe = gamepad1.left_stick_x;
            double speed = 0.5;

            double frontLeftPower = (forward + turn + strafe) * speed;
            double frontRightPower = (forward - turn - strafe) * speed;
            double backLeftPower = (forward - turn + strafe) * speed;
            double backRightPower = (forward + turn - strafe) * speed;

            if(gamepad1.right_bumper) {
                speed = 1;
            } else if(gamepad1.left_bumper) {
                speed = 0.25;
            }
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
