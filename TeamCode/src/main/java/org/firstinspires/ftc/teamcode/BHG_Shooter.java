package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Ben Shooter", group = "Teleop")
public class BHG_Shooter extends LinearOpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor shooter;

    @Override
    public void runOpMode() {
        hardwareStart();
        waitForStart();

        while(opModeIsActive()) {
           if (gamepad1.a) {
               shooter.setPower(1);
               telemetry.addData("Shooter Speed", 1);
           } else if (gamepad1.b) {
               shooter.setPower(0.75);
               telemetry.addData("Shooter Speed", 0.75);
           } else if (gamepad1.y) {
               shooter.setPower(0.5);
               telemetry.addData("Shooter Speed", 0.5);
           } else {
               shooter.setPower(0);
               telemetry.addData("Shooter Speed", 0);
           }
        }
        telemetry.update();
    }
    private void hardwareStart() {
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight  = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");
        shooter = hardwareMap.get(DcMotor.class, "SM");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
    }
}