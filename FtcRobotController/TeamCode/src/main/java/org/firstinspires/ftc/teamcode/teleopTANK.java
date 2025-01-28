package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "teleopTANK", group = "Robot")
public class teleopTANK extends LinearOpMode {
    private static DistanceSensor skibidiSensor;
    private static DcMotor LeftDrive;
    private static DcMotor RightDrive;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        // Declare our motors
        // Make sure your ID's match your configuration
        LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        //port 0
        RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");
        //port 1
        skibidiSensor = hardwareMap.get(DistanceSensor.class, "skibidiSensor");
        //I^2C 0

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        RightDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;

            double LeftPower = 0;
            double RightPower = 0;
           LeftPower = .6 * (y + x);
           RightPower = .6 * (y + -x);
           RightDrive.setPower(RightPower);
           LeftDrive.setPower(LeftPower);
           disTelem();
        }
    }
    public void disTelem()
    {
        telemetry.addData("Distance in IN", "%.2f", skibidiSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
}