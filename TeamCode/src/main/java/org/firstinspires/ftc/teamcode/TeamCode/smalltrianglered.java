package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
@Disabled

public class smalltrianglered extends  LinearOpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    double forward;
    double strafe;
    double turn;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            backRight.setPower(0.5);
            frontRight.setPower(-0.5);
            sleep(2000);
            backRight.setPower(0);
            frontRight.setPower(0);
            sleep(3000000);
            stop();
        }

    }
    public void do_something4() {
        double y;
        double x;
        double rx;
        double dominator;
        double frontLeftpower;
        double backLeftpower;
        double frontRightpower;
        double backRightpower;

        y = forward * 0.8;
        x = -strafe * 0.8;
        rx = turn * 0.8;
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
}
