package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal; // FTC Vision Portal for handling camera input
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor; // AprilTag processing library
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection; // Handles individual AprilTag detections
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import java.util.List; // Imports List functionality for storing multiple detections
/*Second Vision Portal (by Sophie)*/

@TeleOp(name = "BHG Full Directional", group = "TeleOp")
public class BHG_Test_Full_Directional extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    public Servo servo;

    TouchSensor touchSensor; //Have to do this somewhere I guess....
    DistanceSensor distanceSensor;

    public VisionPortal visionPortal; // Declares the VisionPortal for handling camera input
    public AprilTagProcessor aprilTagProcessor; // Declares the AprilTag processor for detecting AprilTags

    @Override
    public void runOpMode() {
        // Initialize motors
        hardwareStart();



        waitForStart();

        while (opModeIsActive()) {
            touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor"); //You could instead do TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");





            double forward = gamepad1.left_stick_y; // Forward/Backward
            double turn = gamepad1.right_stick_x;   // Left/Right turning
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
            // Get game pad input
            if (gamepad1.x) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                sleep(1000);
                leftDrive.setPower(gamepad1.left_stick_y);
                rightDrive.setPower(gamepad1.left_stick_y);
            }

            servoStuff();

            // Check if the sensor is pressed
            boolean isPressed = touchSensor.isPressed();
            if (isPressed) {
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
                sleep(3000);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
            if (Double.isNaN(distance)) {
                telemetry.addData("Distance", "Invalid");
                telemetry.update();
                continue; // Skip loop iteration if the distance is invalid
            }
            // Display on Driver Station
            handleDistance();

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections(); // Retrieves detected AprilTags
            if (detections.isEmpty()) {
                telemetry.addData("NO TAG DETECTED", "No AprilTags found");
            } else {


                for (AprilTagDetection tag : detections) { // Loops through all detected AprilTags
                    telemetry.addData("AprilTag Detected!", "ID: %d", tag.id);



                }


            }
            telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Touch Sensor", isPressed ? "PRESSED" : "NOT PRESSED");

            // Send telemetry data

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
        visionPortal.close(); // Closes the VisionPortal when the OpMode ends

    }
    private void handleDistance() {
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

   }
   private void hardwareStart() {
       leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
       rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
       servo = hardwareMap.get(Servo.class, "servo");


       // Set motor direction (adjust based on physical setup)
       leftDrive.setDirection(DcMotor.Direction.REVERSE);
       rightDrive.setDirection(DcMotor.Direction.FORWARD);

       aprilTagProcessor = new AprilTagProcessor.Builder().build(); // Creates an AprilTagProcessor instance

       // Create VisionPortal with webcam
       visionPortal = new VisionPortal.Builder()
               .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Uses the configured webcam
               .addProcessor(aprilTagProcessor) // Adds the AprilTag processor to handle detections
               .build(); // Builds the VisionPortal instance

       telemetry.addData("Status", "Initialized");
       telemetry.update();

       // Wait for driver to press the start button
       telemetry.addData("Status", "Waiting for Start");
       telemetry.update();
   }
   private void servoStuff() {
       double servoPosition = 0;
       servo.setPosition(servoPosition);
       if(gamepad1.dpad_down) {
           servoPosition = 0;
           servo.setPosition(servoPosition);
       } else if(gamepad1.dpad_up) {
           servoPosition = 1;
           servo.setPosition(servoPosition);
       } else if(gamepad1.dpad_right) {
           if(servoPosition >= 1) {
               servo.setPosition(1);
           } else {
               servoPosition = servoPosition + 0.1;
               servo.setPosition(servoPosition);
           }
       } else if(gamepad1.dpad_left) {
           if(servoPosition <= 0)  {
               servo.setPosition(0);
           } else {
               servoPosition = servoPosition - 0.1;
               servo.setPosition(servoPosition);
           }
       }
       telemetry.addData("Servo Position", servoPosition);
   }
}