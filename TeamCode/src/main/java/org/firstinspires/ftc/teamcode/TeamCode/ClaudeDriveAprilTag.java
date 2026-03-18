package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Disabled
public class ClaudeDriveAprilTag extends LinearOpMode
{
    // -------------------------------------------------------------------------
    // Turning gain & clamp
    // -------------------------------------------------------------------------
    final double TURN_GAIN     = 0.04;
    final double MAX_AUTO_TURN = 0.2;

    // Distance bounds — used only to clamp inputs if needed
    final double RANGE_MIN = 24.0;
    final double RANGE_MAX = 130.0;

    // -------------------------------------------------------------------------
    // Hood servo — quadratic curve fitted to 3 confirmed scoring data points:
    //   76in  → 0.40  (low velocity)
    //   100in → 0.40  (high velocity)
    //   122in → 0.08  (high velocity)
    //
    // Formula: hoodPos = HOOD_A*range^2 + HOOD_B*range + HOOD_C
    // Naturally flattens in mid-range and drops toward far distances.
    // Result is clamped between HOOD_MIN and HOOD_MAX.
    // -------------------------------------------------------------------------
    final double HOOD_A   = -0.00031621;
    final double HOOD_B   =  0.05565217;
    final double HOOD_C   = -2.00316206;
    final double HOOD_MIN =  0.2;   // servo limit — highest physical angle
    final double HOOD_MAX =  0.4;   // servo limit — lowest physical angle

    // Flywheel PIDF — same values as working teleop
    final double FLYWHEEL_F = 28.0;
    final double FLYWHEEL_P = 30.0;
    //   < 88in  → 900  ticks/sec  (confirmed scoring at 76in)
    //   ≥ 88in  → 1300 ticks/sec  (confirmed scoring at 100in and 122in)
    // -------------------------------------------------------------------------
    final double FLYWHEEL_LOW_VEL    = 900.0;
    final double FLYWHEEL_HIGH_VEL   = 1300.0;
    final double VELOCITY_CROSSOVER  = 88.0;

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
    private static final int     DESIRED_TAG_ID = -1;  // -1 = any tag

    private VisionPortal      visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // -------------------------------------------------------------------------
    @Override public void runOpMode()
    {
        boolean targetFound = false;
        double  drive       = 0;
        double  strafe      = 0;
        double  turn        = 0;

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

        // SDK PIDF handles velocity — just needs encoder feedback
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FLYWHEEL_P, 0, 0, FLYWHEEL_F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        if (USE_WEBCAM)
            setManualExposure(6, 250);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            // --- Intake ---
            if (gamepad1.left_trigger >= 0.1) {
                inTake.setPower(1);
            } else if (gamepad1.right_trigger >= 0.1) {
                inTake.setPower(-1);
            } else {
                inTake.setPower(0);
            }

            // --- Transfer / ball stopper ---
            if (gamepad1.left_bumper) {
                transferMotor.setPower(0.4);
                ballStopper.setPosition(1);
            } else if (gamepad1.right_bumper) {
                transferMotor.setPower(-0.4);
                ballStopper.setPosition(0);
            } else {
                transferMotor.setPower(0);
                ballStopper.setPosition(0.5);
            }

            targetFound = false;
            desiredTag  = null;

            // Scan for desired tag
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

            // Driver HUD
            if (targetFound) {
                telemetry.addData("\n>", "HOLD [A] to aim & spin up\n");
                telemetry.addData("Found",   "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",   "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw",     "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }

            // ----------------------------------------------------------------
            // [A]  →  auto-aim mode
            //   • turns to center tag (bearing → 0), no drive/strafe
            //   • hood angle from quadratic curve fitted to real scoring data
            //   • flywheel switches between low/high velocity at 88in crossover
            // ----------------------------------------------------------------
            if (gamepad1.a && targetFound) {

                double range = desiredTag.ftcPose.range;

                // Turn only
                drive  = 0;
                strafe = 0;
                turn   = Range.clip(desiredTag.ftcPose.bearing * TURN_GAIN,
                        -MAX_AUTO_TURN, MAX_AUTO_TURN);

                // Hood — quadratic curve, then INVERTED (servo is physically reversed)
                double rawHood = HOOD_A * range * range + HOOD_B * range + HOOD_C;
                double clampedHood = Math.max(HOOD_MIN, Math.min(HOOD_MAX, rawHood));
                double hoodPos = HOOD_MAX + HOOD_MIN - clampedHood; // flip within [HOOD_MIN, HOOD_MAX]
                hoodServo.setPosition(hoodPos);

                // Flywheel — two-speed: switch at VELOCITY_CROSSOVER inches
                double targetVel = (range < VELOCITY_CROSSOVER)
                        ? FLYWHEEL_LOW_VEL
                        : FLYWHEEL_HIGH_VEL;
                flywheelMotor.setVelocity(targetVel);

                telemetry.addData("Auto-Aim",   "Turn %5.2f", turn);
                telemetry.addData("Range",      "%5.1f in", range);
                telemetry.addData("Hood Pos",   "%5.3f (raw: %.3f, flipped: %.3f)", clampedHood, rawHood, hoodPos);
                telemetry.addData("Target Vel", "%6.0f ticks/sec", targetVel);
                telemetry.addData("Actual Vel", "%6.0f ticks/sec", flywheelMotor.getVelocity());

            } else {
                // Manual drive
                drive  = -gamepad1.left_stick_y  / 1.0;
                strafe = -gamepad1.left_stick_x  / 1.0;
                turn   = -gamepad1.right_stick_x / 1.5;

                // Stop flywheel when not aiming
                flywheelMotor.setVelocity(0);

                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f",
                        drive, strafe, turn);
            }

            telemetry.update();
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    /**
     * Linearly maps value from [inMin, inMax] to [outMin, outMax], clamped.
     */
    private double interpolate(double value,
                               double inMin,  double inMax,
                               double outMin, double outMax) {
        double t = (value - inMin) / (inMax - inMin);
        t = Math.max(0.0, Math.min(1.0, t));
        return outMin + t * (outMax - outMin);
    }

    /**
     * Mecanum drive — Positive X = forward, Positive Y = strafe left, Positive Yaw = CCW.
     */
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

    /** Initialize the AprilTag processor. */
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

    /** Manually set camera gain and exposure (webcam only). */
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() &&
                    visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl =
                    visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}