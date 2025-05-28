package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="JLG_Servo_Test", group="TeleOp")
public class JLG_Servo_Test extends LinearOpMode {

    private Servo servo;
    private double servoPosition = 0.0;
    private boolean dPadRightStatus = false; //Here we are declaring a new variable for the previous state of the dpad_right button and starting it at FALSE
    private boolean dPadLeftStatus = false;  //Here we are declaring a new variable for the previous state of the dpad_left button and starting it at FALSE

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0.0); // Set servo position to 0 when init is pressed

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            // Move servo to max position (1.0) when dpad_up is pressed
            if (gamepad1.dpad_up) {
                servoPosition = 1.0;
            }
            // Move servo to min position (0.0) when dpad_down is pressed
            else if (gamepad1.dpad_down) {
                servoPosition = 0.0;
            }
            // Increase servo position by 0.1 each time dpad_right is pressed
            //  So here we need two conditions to be TRUE for the servo position to go up by 0.1
            //  First, the dpadright must be TRUE (its being pressed)
            //   Second, the previous dpadright must be FALSE (to prevent holding the button from moving it)
            else if (gamepad1.dpad_right && !dPadRightStatus) {
                servoPosition += 0.1;  // Simple way to say take the value and add 0.1
                servoPosition = Math.min(1.0, servoPosition); // Limit to max 1.0, why is it MIN?  Idk let's figure that out
            }
            // Decrease servo position by 0.1 each time dpad_left is pressed
            else if (gamepad1.dpad_left && !dPadLeftStatus) {
                servoPosition -= 0.1;
                servoPosition = Math.max(0.0, servoPosition); // Limit to min 0.0
            }

            // Update previous button states to prevent repeated incrementing while held
            dPadRightStatus = gamepad1.dpad_right;
            dPadLeftStatus = gamepad1.dpad_left;

            // Apply servo position
            servo.setPosition(servoPosition);

            // Telemetry for debugging
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        }
    }
}