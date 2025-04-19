package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "JLG_Touch_Sensor_Testing", group = "TeleOp")
public class JLG_Touch_Sensor_Testing extends LinearOpMode {

    TouchSensor touchSensor;
    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check if the sensor is pressed
            boolean isPressed = touchSensor.isPressed();
                if(isPressed) {

                }
            // Display on Driver Station
            telemetry.addData("Touch Sensor", isPressed ? "PRESSED" : "NOT PRESSED");
            telemetry.update();
        }
    }
}