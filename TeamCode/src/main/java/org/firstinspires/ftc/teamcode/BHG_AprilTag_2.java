package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous; // Marks this class as an Autonomous OpMode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Allows the program to run in a linear sequence
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.VisionPortal; // FTC Vision Portal for handling camera input
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor; // AprilTag processing library
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection; // Handles individual AprilTag detections
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import java.util.List; // Imports List functionality for storing multiple detections
//Second Vision Portal (by Sophie)

@Autonomous(name="AprilTag Detection", group="Vision") // Defines this OpMode as an autonomous program
public class BHG_AprilTag_2 extends LinearOpMode { // Creates a class extending LinearOpMode
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    public VisionPortal visionPortal; // Declares the VisionPortal for handling camera input
    public AprilTagProcessor aprilTagProcessor; // Declares the AprilTag processor for detecting AprilTags

    @Override
    public void runOpMode() { // Main function that runs the Autonomous program
        //Initialize Motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Set motor direction (adjust based on physical setup)
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder().build(); // Creates an AprilTagProcessor instance

        // Create VisionPortal with webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Uses the configured webcam
                .addProcessor(aprilTagProcessor) // Adds the AprilTag processor to handle detections
                .build(); // Builds the VisionPortal instance

        telemetry.addData("Status", "Initialized"); // Displays "Initialized" on the Driver Station telemetry
        telemetry.update(); // Refreshes telemetry data

        waitForStart(); // Waits for the driver to press the start button

        while (opModeIsActive()) { // Runs continuously while the OpMode is active
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections(); // Retrieves detected AprilTags
            if (detections.isEmpty()) {
                telemetry.addData("NO TAG DETECTED", "No AprilTags found");
            } else {
                for (AprilTagDetection tag : detections) { // Loops through all detected AprilTags
                    telemetry.addData("AprilTag Detected!", "ID: %d", tag.id);

                    leftDrive.setPower(0.5);
                    rightDrive.setPower(0.5);

                }

                telemetry.update(); // Updates the telemetry display with the detection info
            }
        }

        visionPortal.close(); // Closes the VisionPortal when the OpMode ends

    }
}


