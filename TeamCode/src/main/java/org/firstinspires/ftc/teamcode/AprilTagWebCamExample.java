package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class AprilTagWebCamExample extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private DcMotor backLeft, backRight, frontLeft, frontRight;
    private DcMotor shooterMotor;
    private Servo topLeftServo, topRightServo;

    // Encoder constants
    private static final double TICKS_PER_REV = 537.7; // GoBILDA 312 RPM
    private static final double WHEEL_DIAMETER_IN = 104.0 / 25.4; // 104 mm wheel
    private static final double WHEEL_CIRCUMFERENCE_IN = Math.PI * WHEEL_DIAMETER_IN;
    private static final double SPROCKET_RATIO = 14.0 / 10.0; // motor sprocket teeth / wheel sprocket teeth
    private static final double TICKS_PER_INCH = (TICKS_PER_REV / WHEEL_CIRCUMFERENCE_IN) / SPROCKET_RATIO;

    // Desired final range and bearing
    private static final double DESIRED_RANGE = 112.6; // far back zone distance
    private static final double DESIRED_BEARING = 0.8; // degrees

    // State machine
    private enum AutoState { LOOK_FOR_TAG, DRIVE_TO_ZONE, STRAFE, ALIGN, FIRE, DONE }
    private AutoState currentState = AutoState.LOOK_FOR_TAG;

    // Memory of starting tag range
    private boolean startRecorded = false;
    private double startRange = 0;
    private double distanceToDrive = 0;

    // Timeout tracking
    private long stateStartTime;
    private static final long TIMEOUT_MS = 5000; // 5 seconds per move

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight= hardwareMap.get(DcMotor.class, "frontRight");
        shooterMotor = hardwareMap.get(DcMotor.class, "topRight"); // your shooter motor
        topLeftServo = hardwareMap.get(Servo.class, "topLeftServo");
        topRightServo= hardwareMap.get(Servo.class, "topRightServo");

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();
    }

    private void resetEncoders() {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopAllMotors() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        resetEncoders();
    }

    private void shoot() {
        telemetry.addLine("Firing!");
        shooterMotor.setPower(0.9);
        topLeftServo.setPosition(0.56);
        topRightServo.setPosition(1.0);
    }

    private void driveForwardInches(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);

        backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetTicks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + targetTicks);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + targetTicks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + targetTicks);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }

    private void strafeInches(double inches, double power) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);

        backLeft.setTargetPosition(backLeft.getCurrentPosition() - targetTicks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + targetTicks);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + targetTicks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - targetTicks);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }

    private void drive(double y, double x, double rx) {
        double dominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / dominator;
        double backLeftPower = (y - (x - rx)) / dominator;
        double frontRightPower = (y - (x + rx)) / dominator;
        double backRightPower = (y + (x - rx)) / dominator;

        backLeft.setPower(frontLeftPower);
        backRight.setPower(frontRightPower);
        frontLeft.setPower(backLeftPower);
        frontRight.setPower(backRightPower);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

        switch (currentState) {
            case LOOK_FOR_TAG:
                if (id20 != null && !startRecorded) {
                    startRange = id20.ftcPose.range;
                    distanceToDrive = startRange - DESIRED_RANGE;
                    startRecorded = true;
                    telemetry.addData("Start Range", startRange);
                    telemetry.addData("Distance to Drive", distanceToDrive);
                    stateStartTime = System.currentTimeMillis();
                    currentState = AutoState.DRIVE_TO_ZONE;
                }
                break;

            case DRIVE_TO_ZONE:
                driveForwardInches(distanceToDrive, 0.5);
                if ((!backLeft.isBusy() && !backRight.isBusy()) ||
                        System.currentTimeMillis() - stateStartTime > TIMEOUT_MS) {
                    stopAllMotors();
                    stateStartTime = System.currentTimeMillis();
                    currentState = AutoState.STRAFE;
                }
                break;

            case STRAFE:
                strafeInches(12, 0); // adjust distance for your field
                if ((!backLeft.isBusy() && !backRight.isBusy()) ||
                        System.currentTimeMillis() - stateStartTime > TIMEOUT_MS) {
                    stopAllMotors();
                    stateStartTime = System.currentTimeMillis();
                    currentState = AutoState.ALIGN;
                }
                break;

            case ALIGN:
                if (id20 != null) {
                    double errorBearing = DESIRED_BEARING - id20.ftcPose.bearing;
                    double rotate = errorBearing * 0.01;
                    drive(0, 0, rotate);

                    if (Math.abs(errorBearing) < 1 ||
                            System.currentTimeMillis() - stateStartTime > TIMEOUT_MS) {
                        stopAllMotors();
                        currentState = AutoState.FIRE;
                    }
                }
                break;

            case FIRE:
                shoot();
                currentState = AutoState.DONE;
                break;

            case DONE:
                stopAllMotors();
                break;
        }
    }
}
