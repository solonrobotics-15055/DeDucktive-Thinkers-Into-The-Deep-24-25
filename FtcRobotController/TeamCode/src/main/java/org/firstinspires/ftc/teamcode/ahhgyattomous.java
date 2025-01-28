package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous (name = "ahhgyattomous", group = "Robot")
public class ahhgyattomous extends LinearOpMode {

    private static DcMotor LeftDrive;
    private static DcMotor RightDrive;
    private static DcMotor LinearSlide;
    private static Servo sigmaClawServo;
    double FORWARD_SPEED = 0.5;
    double TURN_SPEED = 0.5;
    final double CPMR = 560;
    final double diam = 3.5;
    final double CPInch = CPMR /
            (diam * 3.141515926);
    final double sCPI = 200;
    private static DistanceSensor skibidiSensor;

    @Override
    //begin auto
    public void runOpMode() throws InterruptedException {
        LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        //port 0
        RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");
        //port 1
        LinearSlide = hardwareMap.get(DcMotor.class, "LinearSlide");
        //port 2
        sigmaClawServo = hardwareMap.get(Servo.class, "sigmaClawServo");
        //port 0
        skibidiSensor = hardwareMap.get(DistanceSensor.class, "skibidiSensor");
        //I^2C 0
        // Declare our motors
        // Make sure your ID's match your configuration
        //ElapsedTime runtime = new ElapsedTime();


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        LeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) {
            return;
        }
        while (opModeIsActive())
        {
            disTelem();
            sigmaClawServo.setPosition(1.0);
            encoderDrive(0, .8, 0, 0, 25);
            while (skibidiSensor.getDistance(DistanceUnit.INCH) > 10) {
                    dualDrive(FORWARD_SPEED);
                    disTelem();
                }
            dualDrive(0);
            LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderDrive(0, -.9, 0, 0, -8);
            sigmaClawServo.setPosition(0);
            return;
        }
        /*sigmaClawServo.setPosition(1.0);
        encoderDrive(0, .8, 0, 0, 25);
        LeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(FORWARD_SPEED, 0, 25, 25, 0);
        LeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(0, -.9, 0, 0, -8);
        LeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sigmaClawServo.setPosition(0);
        return;*/
    }

    //end auto
    //begin method
    public void disTelem()
    {
        telemetry.addData("Distance in IN", "%.2f", skibidiSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    public void dualDrive(double speed)
    {
        LeftDrive.setPower(speed);
        RightDrive.setPower(speed);
    }
    //begin method
    public void encoderDrive(double speed, double slideSpeed, double leftInches, double rightInches, double slideInches) {
        int newLeftTarget;
        int newRightTarget;
        int newSlideTarget;
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = LeftDrive.getCurrentPosition() + (int) ((leftInches * CPInch) * (1));
            newRightTarget = RightDrive.getCurrentPosition() + (int) (rightInches * CPInch);
            newSlideTarget = LinearSlide.getCurrentPosition() + (int) (slideInches * sCPI);
            LeftDrive.setTargetPosition(newLeftTarget);
            RightDrive.setTargetPosition(newRightTarget);
            LinearSlide.setTargetPosition(newSlideTarget);

            // Turn On RUN_TO_POSITION
            LeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            LeftDrive.setPower(speed);
            RightDrive.setPower(speed);
            LinearSlide.setPower(slideSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            /*while (opModeIsActive() && (LeftDrive.getCurrentPosition() != newLeftTarget || RightDrive.getCurrentPosition() != newRightTarget || LinearSlide.getCurrentPosition() != newSlideTarget)) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        LeftDrive.getCurrentPosition(), RightDrive.getCurrentPosition(), LinearSlide.getCurrentPosition());
                telemetry.update();
            }*/
            sleep(3000);

            // Stop all motion;
            LeftDrive.setPower(0);
            RightDrive.setPower(0);
            LinearSlide.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    } //end method
//runtime.reset();
/*
        //Slide UP
        while (opModeIsActive() && runtime.seconds() < 3.0) {
            LinearSlide.setPower(.64);
        }

        LinearSlide.setPower(0.0);

        //wait
        while (opModeIsActive() && runtime.seconds() < 4.0) {

        }

        //FORWARD
        while (opModeIsActive() && runtime.seconds() < 4.9) {
            LeftDrive.setPower(FORWARD_SPEED);
            RightDrive.setPower(FORWARD_SPEED);
        }
        LeftDrive.setPower(0.0);
        RightDrive.setPower(0.0);

        //Place Specimen
        while (opModeIsActive() && runtime.seconds() < 6.7) {
            LinearSlide.setPower(-.4);
        }
        sigmaClawServo.setPosition(0);

        //Backwards
        while (opModeIsActive() && runtime.seconds() < 7.7) {
            LeftDrive.setPower(-FORWARD_SPEED);
            RightDrive.setPower(-FORWARD_SPEED);
        }
        LeftDrive.setPower(0.0);
        RightDrive.setPower(0.0);

        //Turn
        while (opModeIsActive() && runtime.seconds() < 9.0)
        {
            LeftDrive.setPower(TURN_SPEED);
            RightDrive.setPower(0.0);
        }
        LeftDrive.setPower(0.0);
        RightDrive.setPower(0.0);
        LinearSlide.setPower(0.0);
        //forward
        while (opModeIsActive() && runtime.seconds() < 10.1)
        {
            LeftDrive.setPower(FORWARD_SPEED);
            RightDrive.setPower(FORWARD_SPEED);
        }
        while (opModeIsActive() && runtime.seconds() < 11.1)
        //wait

        {
            LeftDrive.setPower(0.0);
            RightDrive.setPower(0.0);
        }
        //turn
        while (opModeIsActive() && runtime.seconds() < 12.05)
        {
            LeftDrive.setPower(TURN_SPEED);
            RightDrive.setPower(-.2);
        }
        LeftDrive.setPower(0.0);
        RightDrive.setPower(0.0);
        while (opModeIsActive() && runtime.seconds() < 12.13)
        {
            LeftDrive.setPower(FORWARD_SPEED);
            RightDrive.setPower(FORWARD_SPEED);
        }
        LeftDrive.setPower(0.0);
        RightDrive.setPower(0.0);
        while (opModeIsActive() && runtime.seconds() < 12.25  ) {
            LinearSlide.setPower(-.65);
        }
        LinearSlide.setPower(0.0);
    }*/

}