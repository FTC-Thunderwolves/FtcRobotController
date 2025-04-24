package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; // Marks this class as an Autonomous OpMode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Allows the program to run in a linear sequence
import org.firstinspires.ftc.vision.VisionPortal; // FTC Vision Portal for handling camera input
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor; // AprilTag processing library
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection; // Handles individual AprilTag detections
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import java.util.List; // Imports List functionality for storing multiple detections
//Second Vision Portal (by Sophie)

@Autonomous
public class BHG_Logitech_AprilTag_Detect_2 extends LinearOpMode { // Creates a class extending LinearOpMode
    public VisionPortal visionPortal; // Declares the VisionPortal for handling camera input
    public AprilTagProcessor aprilTagProcessor; // Declares the AprilTag processor for detecting AprilTags

    @Override
    public void runOpMode() { // Main function that runs the Autonomous program
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
                telemetry.update(); // Displays message for no tag detection
            } else {


                for (AprilTagDetection tag : detections) { // Loops through all detected AprilTags
                    int TagId = tag.id;
                    if (TagId == 11) { // Checks if the detected tag is ID 11 (Into the Deep AprilTag)
                        telemetry.addData("Correct AprilTag Detected!", "ID: %d", tag.id);
                        telemetry.update();// Displays AprilTag detection info
                    } else if (TagId != 11) {
                        telemetry.addData("Wrong AprilTag", "Looking for TagId 11");
                        telemetry.update();
                    }


                }

                telemetry.update(); // Updates the telemetry display with the detection info
            }
        }

        visionPortal.close(); // Closes the VisionPortal when the OpMode ends

    }
}

