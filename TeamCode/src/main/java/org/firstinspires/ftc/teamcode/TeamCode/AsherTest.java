package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class AsherTest extends OpMode {
    public DcMotorEx flywheelMotor;
    @Override
    public void loop(){
        //Make if gamepad2 right trigger pressed, flywheelMotor power 0.8
        if (gamepad2.right_trigger_pressed) {

            flywheelMotor.setPower(0.8);
        }
    }

    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "topRight");

    }
}
