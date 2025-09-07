package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Ben Shooter", group = "Teleop")
public class BHG_Shooter extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;


    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive  = hardwareMap.get(DcMotor.class, "rightDrive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.right_bumper) {
                leftDrive.setPower(0.5);
                rightDrive.setPower(0.5);
            } else if(gamepad1.left_bumper) {
                leftDrive.setPower(1);
                leftDrive.setPower(1);
            }
        }
    }
}

