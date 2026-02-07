package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Auto", group="Linear Opmode")
public class Auto extends LinearOpMode {
    private DcMotor backLeft, backRight, frontLeft, frontRight, topRight, bottomLeft;
    private static double turn = 0;
    private static double forward = 0;
    private static double strafe = 0;
    private Servo bottomRightServo;
    private Servo bottomLeftServo;
    private Servo topRightServo;
    private Servo topLeftServo;

    private void do_something4() {
        double y = -forward * 1.1;
        double x = -strafe * 1.1;
        double rx = turn * 1.1;
        double dominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftpower = (y + x + rx) / dominator;
        double backLeftpower = (y - (x - rx)) / dominator;
        double frontRightpower = (y - (x + rx)) / dominator;
        double backRightpower = (y + (x - rx)) / dominator;

        backLeft.setPower(backLeftpower);
        backRight.setPower(backRightpower);
        frontLeft.setPower(frontLeftpower);
        frontRight.setPower(frontRightpower);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        bottomLeft = hardwareMap.get(DcMotor.class, "bottomLeft");
        bottomRightServo = hardwareMap.get(Servo.class, "bottomRightServo");
        bottomLeftServo = hardwareMap.get(Servo.class, "bottomLeftServo");
        topRightServo = hardwareMap.get(Servo.class, "topRightServo");
        topLeftServo = hardwareMap.get(Servo.class, "topLeftServo");

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        bottomLeft.setDirection(DcMotor.Direction.REVERSE);
        topRight.setDirection(DcMotor.Direction.REVERSE);

        // Wait for start button
        waitForStart();

        // Example autonomous action
        turn = -0.05;
        forward = 0.1;
        do_something4();
        sleep(100);
        turn = 0;
        forward = 0;
        topRight.setPower(1);
        sleep(2000);
        topLeftServo.setPosition(0.56);
        topRightServo.setPosition(1);
        sleep(2000);
        topRightServo.setPosition(0.4);
        topLeftServo.setPosition(0.9);
        sleep(500);
        bottomRightServo.setPosition(0.8);
        bottomLeftServo.setPosition(0.45);
        topRightServo.setPosition(0.3);        topLeftServo.setPosition(1);
        sleep(400);
        topRightServo.setPosition(0.4);
        topLeftServo.setPosition(0.9);
        sleep(1000);
        topLeftServo.setPosition(0.56);
        topRightServo.setPosition(1);
        sleep(500);
        topRight.setPower(0);
        do_something4();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (bottomRightServo.getPosition() == 0.8) {
                    sleep(1000);
                    bottomRightServo.setPosition(0.1);
                }
                if (bottomLeftServo.getPosition() == 0.45) {
                    bottomLeftServo.setPosition(0.85);
                }
                if (topRightServo.getPosition() == 1) {
                    sleep(1000);
                    topRightServo.setPosition(0.4);
                }
                if (topLeftServo.getPosition() == 0.56) {
                    topLeftServo.setPosition(0.9);
                }
            }
        }
    }
}
