package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Drivetest extends OpMode {
    public double highVelocity = 2300;
    public double lowVelocity = 1900;
    double curTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor inTake;
    private DcMotorEx flywheelMotor;
    private Servo hoodServo;

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
        rx = gamepad1.right_stick_x * 0.8;
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
        float StickRightY;
        double StickLeftY;

        StickRightY = gamepad2.right_stick_y * 1;
        StickLeftY = gamepad2.left_stick_y * 0.85;
        if (StickRightY < StickLeftY) {
            flywheelMotor.setPower(StickLeftY);
        } else if (StickLeftY < StickRightY) {
            flywheelMotor.setPower(StickRightY);
        }
    }
    @Override
    public void init() {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        inTake = hardwareMap.get(DcMotor.class, "inTake");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "topRight");
        inTake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        do_something4();
        do_something();
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity; }
        }
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        if (gamepad1.left_bumper) {
            hoodServo.setPosition(0.02);
        }
        if (gamepad1.right_bumper) {
            hoodServo.setPosition(0.4);
        }
        if (gamepad2.left_trigger >= 0.1) {
            inTake.setPower(1);
        }else if (gamepad2.right_trigger >= 0.1) {
            inTake.setPower(-1);
        } else {
            inTake.setPower(0);
        }



        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelMotor.setVelocity(curTargetVelocity);

        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("----------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
