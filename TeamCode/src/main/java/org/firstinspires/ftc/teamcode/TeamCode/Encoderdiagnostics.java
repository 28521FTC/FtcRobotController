package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class Encoderdiagnostics  extends OpMode {
    private DcMotorEx LR;
    private DcMotorEx RR;
    private DcMotorEx LF;
    private DcMotorEx RF;

    @Override
    public void init() {
        LR = hardwareMap.get(DcMotorEx.class, "frontRight");
        RR = hardwareMap.get(DcMotorEx.class, "frontLeft");
        LF = hardwareMap.get(DcMotorEx.class, "backRight");
        RF = hardwareMap.get(DcMotorEx.class, "backLeft");
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setDirection(DcMotorEx.Direction.REVERSE);
        LF.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("LF", LF.getCurrentPosition());
        telemetry.addData("LR", LR.getCurrentPosition());
        telemetry.addData("RR", RR.getCurrentPosition());
        telemetry.addData("RF", RF.getCurrentPosition());
        telemetry.update();
    }
}
