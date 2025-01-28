package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "teleopTANKclaw1", group = "Robot")
public class teleopTANKclaw1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor LeftDrive = hardwareMap.dcMotor.get("LeftDrive"); // port #0
        DcMotor RightDrive = hardwareMap.dcMotor.get("RightDrive"); // port #1
        DcMotor LinearSlide = hardwareMap.dcMotor.get("LinearSlide"); // port #2
        Servo sigmaClawServo = hardwareMap.servo.get("sigmaClawServo"); // servo port #0

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        RightDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double ly = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double ry = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            boolean up = gamepad2.right_bumper;
            boolean down = gamepad2.left_bumper;
            boolean sigmaclaw = gamepad2.a;
            if (sigmaclaw){
                sigmaClawServo.setPosition(1.0);
            }
            else {
                sigmaClawServo.setPosition(0.0);
            }
            double LeftPower = .6 * ly;
            double RightPower = .6 * ry;
           LeftDrive.setPower(LeftPower);
           RightDrive.setPower(RightPower);
           if (up && !down)
           {
               LinearSlide.setPower(.5);
           }
           else if (down && !up)
           {
                LinearSlide.setPower(-.5);
           }
           else {
               LinearSlide.setPower(0.0);
           }
        }
    }
}