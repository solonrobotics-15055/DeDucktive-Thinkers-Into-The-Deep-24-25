package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "TestEncoder", group = "Robot")
public class TestEncoder extends LinearOpMode {

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
        LeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("LD: ", LeftDrive.getCurrentPosition());
            telemetry.addData("RD: ", RightDrive.getCurrentPosition());
            telemetry.addData("SL: ", LinearSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}