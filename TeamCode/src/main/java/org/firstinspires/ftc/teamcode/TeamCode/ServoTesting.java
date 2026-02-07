package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ServoTesting extends OpMode {
    public Servo hoodServo;
    public Servo kickerServo;
    double[] stepSizes = {10, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;
    double HP;
    double KP;


    @Override
    public void init() {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            hoodServo.setPosition(0.02);
        }
        if (gamepad1.right_bumper) {
            hoodServo.setPosition(0.4);
        }
        if (gamepad2.left_bumper) {
            kickerServo.setPosition(0);
        }
        if (gamepad2.right_bumper) {
            kickerServo.setPosition(0.3);
        }
        telemetry.addData("Hood Postion", HP);
        telemetry.addData("Kicker Postion", KP);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
    }
}
