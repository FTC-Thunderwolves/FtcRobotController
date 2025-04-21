package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class JLG_Testing_Camera extends LinearOpMode {
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Initialize the webcam
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create a VisionPortal instance
        visionPortal = VisionPortal.easyCreateWithDefaults(webcam);

        telemetry.addData("Status", "Webcam Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Streaming", "Active");
            telemetry.update();
        }

        // Close the VisionPortal when done
        visionPortal.close();
    }
}