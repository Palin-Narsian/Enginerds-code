package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;
//Saiisawesome no one else is. Palin if u see this version control works

//All I see are lies. Saiisawesome is cringe nae nae baby

/**
 * Created by robot on 9/17/2018.
 */
@TeleOp(name = "9356 MecanumTeleOp 2019")

public class MecanumTeleop extends OpMode
{

    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;

    //double rightclawpower;
    @Override
    public void init()  {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");

        // Linear slide

        //rotatels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rotatels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // leftclaw = hardwareMap.servo.get("LeftClaw");
        //rightclaw = hardwareMap.servo.get("RightClaw");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        lf.setPower(v1);
        rf.setPower(v2);
        lb.setPower(v3);
        rb.setPower(v4);





    }


}