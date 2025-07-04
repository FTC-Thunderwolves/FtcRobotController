package org.firstinspires.ftc.teamcode; //This line was not in the code that you put in discord. Idk if its in your code or not, but this is a required line.


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//Line 10 was automatically commented out because it has no use.

@TeleOp(name = "MCM_Test", group = "TeleOp")
public class MCM_Test extends LinearOpMode{
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode() {
        starter();
        //I commented out all of handleDistance just because distance sensors take a little more time to understand and there were a few things you forgot to put in this code in order to initialize the distance sensor.
        //Ex. You had to make a variable for the distance sensor, put it into the config, and name it. Also, you forgot to put the handleDistance method into the main code, so it wouldn't have worked anyway.
        telemetry.addData("Status", "Ready to start");
        telemetry.addData("Hello", "World"); //Fixed Hello World
        telemetry.update();

        float forward;
        float turn;
        float strafe;
        double speed;
        // 1 left, -1 right (i think)
        // turning works ish?


        waitForStart();

        while (opModeIsActive()) {

            forward = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x/2; // so the difference between forward + and - turn is turn
            strafe = gamepad1.left_stick_x; // maybe + for front - for back, - for left

            if (gamepad1.left_bumper) {
                speed = 1; //turboooo
            } else if (strafe >= 0.3 && !gamepad1.right_bumper) {
                speed = 0.5 + strafe/3;
            } else if (strafe <= -0.3 && !gamepad1.right_bumper) {
                speed = 0.5 + -strafe/3; // strafing is slow so this speeds it up (right bumper to stay slow)
            } else {
                speed = 0.5; //normal
            }


            //This is Maddie's code.
            frontRight.setPower((forward+turn+strafe)*speed); //I changed the power value to 0.5 just because that i usually avoid using full speed, you can change it if you want
            frontLeft.setPower((forward-turn-strafe)*speed);
            backRight.setPower((forward+turn-strafe)*speed); //I changed the power value to 0.5 just because that i usually avoid using full speed, you can change it if you want
            backLeft.setPower((forward-turn+strafe)*speed);


            telemetry.addData("Speed", speed);
            telemetry.update();
        }
    }
    private void starter() {


        frontRight  = hardwareMap.get(DcMotor.class, "FR");  //make sure these match with config
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");

        // Set motor direction (adjust based on physical setup)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }
    /*private void handleDistance() {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        while (distance <= 12 && opModeIsActive()) {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            sleep(500);
            leftDrive.setPower(gamepad1.left_stick_y);
            rightDrive.setPower(gamepad1.left_stick_y);

            // Update the distance for the next iteration
            distance = distanceSensor.getDistance(DistanceUnit.INCH);
        }

    } */
}