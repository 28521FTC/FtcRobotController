package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Flywheel extends OpMode {
    public double highVelocity = 1300;
    public double lowVelocity = 900;
    double curTargetVelocity = highVelocity;
    double F = 28;
    double P = 30;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor inTake;
    private DcMotorEx flywheelMotor;
    private DcMotorEx flywheelMotor2;

    private DcMotor transferMotor;

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

    @Override
    public void init() {
        ballStopper = hardwareMap.get(Servo.class, "ballStopper");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        inTake = hardwareMap.get(DcMotor.class, "inTake");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "topRight");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        inTake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor2.setDirection(DcMotor.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        do_something4();
        if (gamepad2.left_bumper) {
            transferMotor.setPower(0.4);
            ballStopper.setPosition(1);
        } else if (gamepad2.right_bumper) {
            transferMotor.setPower(-0.4);
            ballStopper.setPosition(0);
        } else {
            transferMotor.setPower(0);
            ballStopper.setPosition(0.5);
        }
        if (gamepad2.dpad_up) {
            hoodServo.setPosition(0.05);
        }
        if (gamepad2.dpad_down) {
            hoodServo.setPosition(0.4);
        }
        if (gamepad2.left_trigger >= 0.1) {
            inTake.setPower(1);
        }else if (gamepad2.right_trigger >= 0.1) {
            inTake.setPower(-1);
        } else {
            inTake.setPower(0);
        }
        if (gamepad2.right_stick_y != 0) {
            flywheelMotor.setVelocity(curTargetVelocity);
            curTargetVelocity = highVelocity;
        } else if (gamepad2.left_stick_y != 0) {
            flywheelMotor.setVelocity(curTargetVelocity);
            curTargetVelocity = lowVelocity;
        } else { flywheelMotor.setPower(0);}


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;
    }
}
