package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class Flywheel2 extends OpMode {
    final double TURN_GAIN     = 0.04;
    final double MAX_AUTO_TURN = 0.2;

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
    private static final boolean USE_WEBCAM     = true;
    private static final int     DESIRED_TAG_ID = 20;


    private DcMotor transferMotor;

    private Servo hoodServo;
    private Servo ballStopper;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // -------------------------------------------------------------------------
    // State variables (previously local to runOpMode)
    // -------------------------------------------------------------------------
    private boolean targetFound = false;
    private double  drive       = 0;
    private double  strafe      = 0;
    private double  turn        = 0;


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
        initAprilTag();

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
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor2.setDirection(DcMotor.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");

    }

    @Override
    public void loop() {
        targetFound = false;
        desiredTag  = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    targetFound = true;
                    desiredTag  = detection;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
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
            hoodServo.setPosition(0.2);
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
        if (targetFound) {
            telemetry.addData("\n>", "HOLD [A] to aim & spin up\n");
            telemetry.addData("Found",   "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",   "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw",     "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }
        if (gamepad1.a && targetFound) {
            double range = desiredTag.ftcPose.range;

            drive  = 0;
            strafe = 0;
            turn   = Range.clip(desiredTag.ftcPose.bearing * TURN_GAIN,
                    -MAX_AUTO_TURN, MAX_AUTO_TURN);

            telemetry.addData("Auto-Aim",   "Turn %5.2f", turn);
            telemetry.addData("Range",      "%5.1f in", range);
            telemetry.addData("Target Vel", "%6.0f ticks/sec", curTargetVelocity);
            telemetry.addData("Actual Vel", "%6.0f ticks/sec", flywheelMotor.getVelocity());

        } else {
            drive  = -gamepad1.left_stick_y  / 1.0;
            strafe = -gamepad1.left_stick_x  / 1.0;
            turn   = -gamepad1.right_stick_x / 1.5;

            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f",
                    drive, strafe, turn);
        }
        moveRobot(drive,strafe,turn);
    }
    public void moveRobot(double x, double y, double yaw) {
        double frontLeftPower  =  x - y - yaw;
        double frontRightPower =  x + y + yaw;
        double backLeftPower   =  x + y - yaw;
        double backRightPower  =  x - y + yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        // Busy-wait is not allowed in OpMode — poll in init_loop() instead.
        // Here we just fire-and-forget if already streaming; otherwise it will
        // be retried automatically when the camera becomes ready.
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        ExposureControl exposureControl =
                visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }
}
