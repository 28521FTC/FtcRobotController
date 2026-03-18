package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class ServoContTest extends OpMode {
    private Servo ballStopper;

    @Override
    public void init() {
        ballStopper = hardwareMap.get(Servo.class, "ballStopper");
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper)
            ballStopper.setPosition(1);
        else {
            ballStopper.setPosition(0.5);
        }
    }
    }