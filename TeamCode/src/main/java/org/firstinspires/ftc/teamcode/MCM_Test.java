//I put comments on lines 3, 11, 15, 21, 22, 35, and 52 where I attempted to explain what I changed and why. If you have any more questions, just ask and I'll explain it the best I can.

package org.firstinspires.ftc.teamcode; //This line was not in the code that you put in discord. Idk if its in your code or not, but this is a required line.


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//Line 10 was automatically commented out because it has no use.

@TeleOp(name = "MCM_Test", group = "TeleOp")
public class MCM_Test extends LinearOpMode{
    private DcMotor frontLeft;
    //You needed to rename these variables
    private DcMotor frontRight;

    @Override
    public void runOpMode() {
        starter();
        //I commented out all of handleDistance just because distance sensors take a little more time to understand and there were a few things you forgot to put in this code in order to initialize the distance sensor.
        //Ex. You had to make a variable for the distance sensor, put it into the config, and name it. Also, you forgot to put the handleDistance method into the main code, so it wouldn't have worked anyway.
        telemetry.addData("Status", "Ready to start");
        telemetry.addData("me", " hello world");
        telemetry.update();

        frontRight  = hardwareMap.get(DcMotor.class, "Front Right");
        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                frontRight.setPower(0.5); //I changed the power value to 0.5 just because that i usually avoid using full speed, you can change it if you want
            } else {
                frontRight.setPower(0);
            }





        }


    }
    private void starter() {

        // Set motor direction (adjust based on physical setup)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        //Because you changed the name of the variables, you had to change these lines too.
        frontRight.setDirection(DcMotor.Direction.FORWARD);


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

