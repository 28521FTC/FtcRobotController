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
public class opmode extends OpMode
{
    // -------------------------------------------------------------------------
    // Turning gain & clamp
    // -------------------------------------------------------------------------
    final double TURN_GAIN     = 0.04;
    final double MAX_AUTO_TURN = 0.2;

    final double RANGE_MIN = 24.0;
    final double RANGE_MAX = 130.0;
    public double highVelocity = 1200;
    public double lowVelocity  = 800;
    double curTargetVelocity   = highVelocity;

    final double HOOD_A   = -0.00031621;
    final double HOOD_B   =  0.05565217;
    final double HOOD_C   = -2.00316206;
    final double HOOD_MIN =  0.2;
    final double HOOD_MAX =  0.4;

    final double F = 30.0;
    final double P = 50.0;
    final double FLYWHEEL_LOW_VEL   = 900.0;
    final double FLYWHEEL_HIGH_VEL  = 1300.0;
    final double VELOCITY_CROSSOVER = 88.0;

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private DcMotor   frontLeftDrive, inTake, transferMotor = null;
    private DcMotor   frontRightDrive = null;
    private DcMotor   backLeftDrive   = null;
    private DcMotor   backRightDrive  = null;

    private DcMotorEx flywheelMotor = null;
    private Servo     hoodServo, ballStopper = null;

    private static final boolean USE_WEBCAM     = true;
    private static final int     DESIRED_TAG_ID = 20;

    private VisionPortal      visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // -------------------------------------------------------------------------
    // State variables (previously local to runOpMode)
    // -------------------------------------------------------------------------
    private boolean targetFound = false;
    private double  drive       = 0;
    private double  strafe      = 0;
    private double  turn        = 0;

    // -------------------------------------------------------------------------
    // init() — replaces everything before waitForStart()
    // -------------------------------------------------------------------------
    @Override
    public void init()
    {
        initAprilTag();

        frontLeftDrive  = hardwareMap.get(DcMotor.class,   "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class,   "frontRight");
        backLeftDrive   = hardwareMap.get(DcMotor.class,   "backLeft");
        backRightDrive  = hardwareMap.get(DcMotor.class,   "backRight");
        flywheelMotor   = hardwareMap.get(DcMotorEx.class, "topRight");
        hoodServo       = hardwareMap.get(Servo.class,     "hoodServo");
        inTake          = hardwareMap.get(DcMotor.class,   "inTake");
        ballStopper     = hardwareMap.get(Servo.class,     "ballStopper");
        transferMotor   = hardwareMap.get(DcMotor.class,   "transferMotor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        if (USE_WEBCAM)
            setManualExposure(6, 250);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
    }

    // -------------------------------------------------------------------------
    // loop() — replaces the while (opModeIsActive()) body
    // -------------------------------------------------------------------------
    @Override
    public void loop()
    {
        // --- Flywheel ---
        if (gamepad2.right_stick_y != 0) {
            flywheelMotor.setVelocity(curTargetVelocity);
        } else {
            flywheelMotor.setPower(0);
        }

        // --- Intake ---
        if (gamepad1.left_trigger >= 0.1 || gamepad2.left_trigger >= 0.1) {
            inTake.setPower(1);
        } else if (gamepad1.right_trigger >= 0.1 || gamepad2.right_trigger >= 0.1) {
            inTake.setPower(-1);
        } else {
            inTake.setPower(0);
        }

        // --- Transfer / ball stopper ---
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            transferMotor.setPower(0.4);
            ballStopper.setPosition(1);
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            transferMotor.setPower(-0.4);
            ballStopper.setPosition(0);
        } else {
            transferMotor.setPower(0);
            ballStopper.setPosition(0.5);
        }

        // --- AprilTag scan ---
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

        telemetry.addData("Flywheel Speed:", flywheelMotor.getVelocity());
        telemetry.addData("Flywheel Target:", curTargetVelocity);

        // --- Driver HUD & hood/velocity presets ---
        if (targetFound) {
            telemetry.addData("\n>", "HOLD [A] to aim & spin up\n");
            telemetry.addData("Found",   "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",   "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw",     "%3.0f degrees", desiredTag.ftcPose.yaw);

            double range = desiredTag.ftcPose.range;
            if (range < 80) {
                hoodServo.setPosition(0.4);
                curTargetVelocity = lowVelocity;
            } else if (range > 85 && range < 120) {
                hoodServo.setPosition(0.2);
                curTargetVelocity = highVelocity;
            } else if (range > 120) {
                hoodServo.setPosition(0.3);
                curTargetVelocity = highVelocity;
            }
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }

        // --- Auto-aim ([A] held) vs manual drive ---
        if (gamepad1.a && targetFound) {
            double range = desiredTag.ftcPose.range;

            drive  = 0;
            strafe = 0;
            turn   = Range.clip(desiredTag.ftcPose.bearing * TURN_GAIN,
                    -MAX_AUTO_TURN, MAX_AUTO_TURN);

            double rawHood     = HOOD_A * range * range + HOOD_B * range + HOOD_C;
            double clampedHood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, rawHood));
            double hoodPos     = HOOD_MAX + HOOD_MIN - clampedHood;

            telemetry.addData("Auto-Aim",   "Turn %5.2f", turn);
            telemetry.addData("Range",      "%5.1f in", range);
            telemetry.addData("Hood Pos",   "%5.3f (raw: %.3f, flipped: %.3f)", clampedHood, rawHood, hoodPos);
            telemetry.addData("Target Vel", "%6.0f ticks/sec", curTargetVelocity);
            telemetry.addData("Actual Vel", "%6.0f ticks/sec", flywheelMotor.getVelocity());

        } else {
            drive  = -gamepad1.left_stick_y  / 1.0;
            strafe = -gamepad1.left_stick_x  / 1.0;
            turn   = -gamepad1.right_stick_x / 1.5;

            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f",
                    drive, strafe, turn);
        }

        telemetry.update();
        moveRobot(drive, strafe, turn);
    }

    // -------------------------------------------------------------------------
    // Helper methods (unchanged)
    // -------------------------------------------------------------------------
    private double interpolate(double value,
                               double inMin,  double inMax,
                               double outMin, double outMax) {
        double t = (value - inMin) / (inMax - inMin);
        t = Math.max(0.0, Math.min(1.0, t));
        return outMin + t * (outMax - outMin);
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

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
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