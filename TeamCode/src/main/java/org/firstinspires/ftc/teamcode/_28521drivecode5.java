package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "28521 drivecode")
public class _28521drivecode5 extends LinearOpMode {

  private DcMotor backLeft;
  private DcMotor backRight;
  private DcMotor frontLeft;
  private DcMotor frontRight;
  private DcMotor topRight;
  private DcMotor bottomLeft;
  private DcMotor bottomRight;
  private Servo bottomRightServo;
  private Servo bottomLeftServo;
  private Servo topRightServo;
  private Servo topLeftServo;
  private ColorSensor colorLeft;
  private ColorSensor colorLeft_REV_ColorRangeSensor;

  AprilTagProcessor myAprilTagProcessor;
  boolean USE_WEBCAM;

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

  /**
   * Describe this function...
   */
  private void do_something() {
    float StickRightY;
    double StickLeftY;

    StickRightY = -gamepad2.right_stick_y * 1;
    StickLeftY = -gamepad2.left_stick_y * 0.85;
    if (StickRightY < StickLeftY) {
      topRight.setPower(StickLeftY);
    } else if (StickLeftY < StickRightY) {
      topRight.setPower(StickRightY);
    }
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
    int Blue;
    int Green;
    int Red;

    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    backRight = hardwareMap.get(DcMotor.class, "backRight");
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    topRight = hardwareMap.get(DcMotor.class, "topRight");
    bottomLeft = hardwareMap.get(DcMotor.class, "bottomLeft");
    bottomRight = hardwareMap.get(DcMotor.class, "bottomRight");
    bottomRightServo = hardwareMap.get(Servo.class, "bottomRightServo");
    bottomLeftServo = hardwareMap.get(Servo.class, "bottomLeftServo");
    topRightServo = hardwareMap.get(Servo.class, "topRightServo");
    topLeftServo = hardwareMap.get(Servo.class, "topLeftServo");
    colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
    colorLeft_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorLeft");

    // Put initialization blocks here.
    backRight.setDirection(DcMotor.Direction.REVERSE);
    frontLeft.setDirection(DcMotor.Direction.REVERSE);
    frontRight.setDirection(DcMotor.Direction.REVERSE);
    bottomLeft.setDirection(DcMotor.Direction.REVERSE);
    topRight.setDirection(DcMotor.Direction.REVERSE);
    USE_WEBCAM = true;
    // Initialize AprilTag before waitForStart.
    initAprilTag();
    // Wait for the match to begin.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch START to start OpMode");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        do_something4();
        do_something();
        bottomLeft.setPower(gamepad2.left_trigger);
        bottomRight.setPower(gamepad2.left_trigger);
        if (gamepad2.left_trigger > 0.2) {
          bottomRightServo.setPosition(0.03);
        } else {
          bottomRightServo.setPosition(0.1);
        }
        if (gamepad2.dpad_down) {
          frontRight.setPower(0);
          backRight.setPower(0);
          frontLeft.setPower(0);
          backLeft.setPower(0);
        }
        if (gamepad2.dpad_down) {
          topRightServo.setPosition(0.3);
          topLeftServo.setPosition(1);
          sleep(100);
          bottomLeftServo.setPosition(0.3);
          bottomRightServo.setPosition(0.8);
          sleep(600);
          topRightServo.setPosition(0.4);
          topLeftServo.setPosition(0.9);
        }
        if (gamepad2.dpad_up) {
          topRightServo.setPosition(1);
          frontRight.setPower(0);
          backRight.setPower(0);
          frontLeft.setPower(0);
          backLeft.setPower(0);
        }
        if (gamepad2.dpad_up) {
          topLeftServo.setPosition(0.56);
        }
        if (bottomRightServo.getPosition() == 0.8) {
          sleep(1000);
          bottomRightServo.setPosition(0.1);
        }
        if (bottomLeftServo.getPosition() == 0.3) {
          bottomLeftServo.setPosition(0.85);
        }
        if (topRightServo.getPosition() == 1) {
          sleep(1000);
          topRightServo.setPosition(0.4);
        }
        if (topLeftServo.getPosition() == 0.56) {
          topLeftServo.setPosition(0.9);
        }
        telemetry.addLine("");

        telemetry.update();
        telemetryAprilTag();
        // Share the CPU.
        sleep(20);
      }
    }
  }

  /**
   * Describe this function...
   */
  private void Webcamstart() {
  }

  /**
   * Initialize AprilTag Detection.
   */
  private void initAprilTag() {
    VisionPortal.Builder myVisionPortalBuilder;
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    VisionPortal myVisionPortal;

    // First, create an AprilTagProcessor.Builder.
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    // Create an AprilTagProcessor by calling build.
    myAprilTagProcessor = myAprilTagProcessorBuilder.build();
    // Next, create a VisionPortal.Builder and set attributes related to the camera.
    myVisionPortalBuilder = new VisionPortal.Builder();
    if (USE_WEBCAM) {
      // Use a webcam.
      myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      // Use the device's back camera.
      myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    // Add myAprilTagProcessor to the VisionPortal.Builder.
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
    // Create a VisionPortal by calling build.
    myVisionPortal = myVisionPortalBuilder.build();
  }

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  private void telemetryAprilTag() {
    AprilTagDetection myAprilTagDetection;
    List<AprilTagDetection> myAprilTagDetections;

    // Get a list of AprilTag detections.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
      telemetry.addLine("");
      if (myAprilTagDetection.metadata != null) {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
        telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
        telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
        telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
      } else {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
        telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
      }
    }
    telemetry.addLine("");
    telemetry.addLine("key:");
    telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");
  }
}
