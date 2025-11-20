package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "_28521drivecode (Blocks to Java)")
public class _28521drivecode extends LinearOpMode {

  private DcMotor backLeft;
  private DcMotor backRight;
  private DcMotor frontLeft;
  private DcMotor frontRight;
  private DcMotor bottomLeft;
  private DcMotor topRight;
  private DcMotor bottomRight;
  private Servo bottomRightServo;
  private Servo bottomLeftServo;
  private Servo topRightServo;
  private Servo topLeftServo;

  /**
   * Describe this function...
   */
  private void do_something4() {
    double y;
    double x;
    double rx;
    double dominator;
    double frontLeftpower;
    double backLeftpower;
    double frontRightpower;
    double backRightpower;

    y = gamepad1.left_stick_y * 1.1;
    x = -gamepad1.left_stick_x * 1.1;
    rx = gamepad1.right_stick_x * 1.1;
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

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    backRight = hardwareMap.get(DcMotor.class, "backRight");
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    bottomLeft = hardwareMap.get(DcMotor.class, "bottomLeft");
    topRight = hardwareMap.get(DcMotor.class, "topRight");
    bottomRight = hardwareMap.get(DcMotor.class, "bottomRight");
    bottomRightServo = hardwareMap.get(Servo.class, "bottomRightServo");
    bottomLeftServo = hardwareMap.get(Servo.class, "bottomLeftServo");
    topRightServo = hardwareMap.get(Servo.class, "topRightServo");
    topLeftServo = hardwareMap.get(Servo.class, "topLeftServo");

    // Put initialization blocks here.
    backRight.setDirection(DcMotor.Direction.REVERSE);
    frontLeft.setDirection(DcMotor.Direction.REVERSE);
    frontRight.setDirection(DcMotor.Direction.REVERSE);
    bottomLeft.setDirection(DcMotor.Direction.REVERSE);
    topRight.setDirection(DcMotor.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        do_something4();
        topRight.setPower(gamepad2.right_stick_y * 1.2);
        bottomLeft.setPower(gamepad2.left_trigger);
        bottomRight.setPower(gamepad2.left_trigger);
        if (gamepad2.left_trigger > 0.3) {
          bottomRightServo.setPosition(0.01);
        }
        if (gamepad2.dpad_down) {
          bottomRightServo.setPosition(0.8);
        }
        if (gamepad2.dpad_down) {
          bottomLeftServo.setPosition(0.35);
          topRightServo.setPosition(0.3);
          topLeftServo.setPosition(1);
          sleep(400);
          topRightServo.setPosition(0.4);
          topLeftServo.setPosition(0.9);
        }
        if (gamepad2.dpad_up) {
          topRightServo.setPosition(1);
        }
        if (gamepad2.dpad_up) {
          topLeftServo.setPosition(0.6);
        }
        if (bottomRightServo.getPosition() == 0.8) {
          sleep(1000);
          bottomRightServo.setPosition(0.1);
        }
        if (bottomRightServo.getPosition() == 0.01) {
          sleep(1000);
          bottomRightServo.setPosition(0.1);
        }
        if (bottomLeftServo.getPosition() == 0.35) {
          bottomLeftServo.setPosition(0.82);
        }
        if (topRightServo.getPosition() == 1) {
          sleep(1000);
          topRightServo.setPosition(0.4);
        }
        if (topLeftServo.getPosition() == 0.6) {
          topLeftServo.setPosition(0.95);
        }
      }
    }
  }
}
