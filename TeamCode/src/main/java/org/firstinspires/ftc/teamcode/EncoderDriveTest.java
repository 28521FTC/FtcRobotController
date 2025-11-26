package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class EncoderDriveTest extends OpMode {
    private DcMotor backLeft, backRight, frontLeft, frontRight;

    // Encoder constants
    private static final double TICKS_PER_REV = 537.7; // GoBILDA 312 RPM
    private static final double WHEEL_DIAMETER_IN = 104.0 / 25.4; // 104 mm wheel
    private static final double WHEEL_CIRCUMFERENCE_IN = Math.PI * WHEEL_DIAMETER_IN;
    private static final double SPROCKET_RATIO = 14.0 / 10.0; // motor sprocket / wheel sprocket
    private static final double TICKS_PER_INCH = (TICKS_PER_REV / WHEEL_CIRCUMFERENCE_IN) / SPROCKET_RATIO;

    private enum AutoState { DRIVE_BACK, STRAFE, DONE }
    private AutoState currentState = AutoState.DRIVE_BACK;

    private long stateStartTime;
    private static final long TIMEOUT_MS = 4000; // 4s per move

    @Override
    public void init() {
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight= hardwareMap.get(DcMotor.class, "frontRight");

        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


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

    private boolean allMotorsDone() {
        return !backLeft.isBusy() && !backRight.isBusy()
                && !frontLeft.isBusy() && !frontRight.isBusy();
    }

    private boolean timedOut() {
        return System.currentTimeMillis() - stateStartTime > TIMEOUT_MS;
    }

    private void driveForwardInches(double inches, double power) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setPower(power + 0.15);
        backLeft.setPower(power);
        frontRight.setPower(power + 0.15);
        frontLeft.setPower(power);
    }

    private void strafeInches(double inches, double power) {
        int ticks = (int)(inches * TICKS_PER_INCH);

        // Adjust signs if it goes diagonally instead of sideways
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeft.setPower(-power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }

    @Override
    public void start() {
        stateStartTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        switch (currentState) {
            case DRIVE_BACK:
                driveForwardInches(12, 0.5); // drive back 24 inches
                if (allMotorsDone() || timedOut()) {
                    stopAllMotors();
                    stateStartTime = System.currentTimeMillis();
                    currentState = AutoState.STRAFE;
                }

                break;
            case STRAFE:
                strafeInches(12, 0);
                if (allMotorsDone() || timedOut()) {
                    stopAllMotors();
                    currentState = AutoState.DONE;
                }
                break;

            case DONE:
                stopAllMotors();
                telemetry.addLine("Test complete");
                break;
        }
    }
}
