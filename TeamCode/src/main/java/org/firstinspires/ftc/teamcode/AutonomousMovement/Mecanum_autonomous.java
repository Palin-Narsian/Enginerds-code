package org.firstinspires.ftc.teamcode.AutonomousMovement;


import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by robot on 9/19/2018.
 */
@Autonomous(name = "9356 Old Autonomous 2019")
@Disabled
public class Mecanum_autonomous extends LinearOpMode {
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    //HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.dcMotor.get("lw");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // leftclaw = hardwareMap.servo.get("LeftClaw");
        //rightclaw = hardwareMap.servo.get("RightClaw");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                lf.getCurrentPosition(),
                rf.getCurrentPosition(),
                lb.getCurrentPosition(),
                rb.getCurrentPosition());
        telemetry.update();

        waitForStart();


        encoderDrive(DRIVE_SPEED, 5, 5, 500);
        encoderDrive(DRIVE_SPEED, 9, -9, 500);



        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftfTarget;
        int newRightfTarget;
        int newLeftbTarget;
        int newRightbTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftfTarget = lf.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightfTarget = rf.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftbTarget = lb.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightbTarget = rb.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            lf.setTargetPosition(newLeftfTarget);
            rf.setTargetPosition(newRightfTarget);
            lb.setTargetPosition(newLeftbTarget);
            rb.setTargetPosition(newRightbTarget);

            // Turn On RUN_TO_POSITION
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lf.setPower(Math.abs(speed));
            rf.setPower(Math.abs(speed));
            lb.setPower(Math.abs(speed));
            rb.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lf.isBusy() && rf.isBusy() && lb.isBusy() && rb.isBusy()))

                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftfTarget,  newRightfTarget, newLeftbTarget,newRightbTarget);
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    lf.getCurrentPosition(),
                    rf.getCurrentPosition(),
                    lb.getCurrentPosition(),
                    rb.getCurrentPosition());
            telemetry.update();

            {

                // Display it for the driver.

            }

            // Stop all motion;
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);

            // Turn off RUN_TO_POSITION
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }

    public void Stop() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
    public void SlideRight() {
        lf.setPower(0.4);
        rf.setPower(-0.4);
        lb.setPower(-0.4);
        rb.setPower(0.4);
    }
}

