package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Drive extends OpMode {
    public double highVelocity = 2200;
    public double lowVelocity = 1900;
    double curTargetVelocity = highVelocity;
    double F = -13.300;
    double P = -13;

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor inTake;
    private DcMotor transferMotor;
    private DcMotorEx flywheelMotor;
    private Servo hoodServo;
    private Servo ballStopper;


    private void do_something4() {
        double y;
        double x;
        double rx;
        double dominator;
        double frontLeftpower;
        double backLeftpower;
        double frontRightpower;
        double backRightpower;

        y = gamepad1.left_stick_y * 0.8;
        x = -gamepad1.left_stick_x * 0.8;
        rx = -gamepad1.right_stick_x * 0.8;
        dominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftpower = (y + x + rx) / dominator;
        backLeftpower = (y - (x - rx)) / dominator;
        frontRightpower = (y - (x + rx)) / dominator;
        backRightpower = (y + (x - rx)) / dominator;
        backLeft.setPower(backLeftpower);
        backRight.setPower(backRightpower);
        frontLeft.setPower(frontLeftpower);
        frontRight.setPower(frontRightpower);
    }

    private void do_something() {
        float StickRightY = gamepad2.right_stick_y;
        double StickLeftY = gamepad2.left_stick_y;

        if (Math.abs(StickLeftY) >= 0.2) {
            flywheelMotor.setPower(0.75);
        } else if (Math.abs(StickRightY) >= 0.2) {
            flywheelMotor.setPower(1);
        } else {
            flywheelMotor.setPower(0);
        }
    }
    @Override
    public void init() {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        ballStopper = hardwareMap.get(Servo.class, "ballStopper");
        inTake = hardwareMap.get(DcMotor.class, "inTake");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "topRight");
        inTake.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        do_something4();
        do_something();
        if (gamepad2.dpad_up) {
            hoodServo.setPosition(0.12);
        }
        if (gamepad2.dpad_down) {
            hoodServo.setPosition(0.34);
        }
        if (gamepad2.left_bumper) {
            transferMotor.setPower(0.4);
        } else if (gamepad2.right_bumper) {
            transferMotor.setPower(-0.4);
        } else {
            transferMotor.setPower(0);
        }
        if (gamepad2.left_trigger >= 0.1) {
            inTake.setPower(1);
        }else if (gamepad2.right_trigger >= 0.1) {
            inTake.setPower(-0.5);
        } else {
            inTake.setPower(0);
        }
    }
}
