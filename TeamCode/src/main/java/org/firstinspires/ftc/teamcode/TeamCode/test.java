package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class test  extends OpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "backRight");
        backRight = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
    }
    public void loop() {


        if (gamepad1.a) {
            backLeft.setPower(1);
            backRight.setPower(1);
            frontLeft.setPower(1);
            frontRight.setPower(1);
        } else if (gamepad1.right_bumper) {
            backLeft.setPower(1);
            frontLeft.setPower(1);
        } else if (gamepad1.left_bumper) {
            frontRight.setPower(1);
            backRight.setPower(1);
        } else {
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }
        telemetry.addData("RL", backLeft.getDirection());
        telemetry.addData("FL", frontLeft.getDirection());
        telemetry.addData("RR", backRight.getDirection());
        telemetry.addData("FR", frontRight.getDirection());
    }
}
