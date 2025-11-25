package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
public class AprilTagWebCamExample extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor topRight;
    private Servo topRightServo;
    private Servo topLeftServo;



    // Desired target values
    private static final double TARGET_X = 9.9;
    private static final double TARGET_Y = 115.3; // midpoint of 115.2–115.4
    private static final double TARGET_Z = -11.2;
    private static final double DESIRED_RANGE = 112.6; // adjust to your far back zone
    private static final double DESIRED_BEARING = 0.8; // degrees



    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        topLeftServo= hardwareMap.get(Servo.class, "topLeftServo");
        topRightServo= hardwareMap.get(Servo.class, "topRightServo");

    }

    private void shoot() {
        // Example: spin flywheel and trigger servo
        topRight.setPower(0.9);
        topLeftServo.setPosition(0.56);
        topRightServo.setPosition(1);
    }


    private void drive(double y, double x, double rx) {
        double dominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / dominator;
        double backLeftPower = (y - (x - rx)) / dominator;
        double frontRightPower = (y - (x + rx)) / dominator;
        double backRightPower = (y + (x - rx)) / dominator;

        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

        if (id20 != null) {
            telemetry.addData("Tag 20", "Detected");
            aprilTagWebcam.displayDetectionTelemetry(id20);
            double errorBearing = DESIRED_BEARING - id20.ftcPose.bearing;
            double errorRange   = DESIRED_RANGE - id20.ftcPose.range;
            // Use Range to decide forward/back
            double forward = errorRange * 0.01;

            // Use Bearing to decide rotation
            double rotate = errorBearing * 0.01;

            // Optional: strafe to center
            double strafe = id20.ftcPose.x * 0.01;

            // Drive toward launch zone
            drive(forward, strafe, rotate);

            // If close enough, stop and shoot
            if (Math.abs(id20.ftcPose.range - DESIRED_RANGE) < 2 &&
                    Math.abs(id20.ftcPose.bearing) < 2) {
                drive(0, 0, 0);
                shoot();
            }
        } else {
            telemetry.addData("Tag 20", "Not detected");
            drive(0, 0, 0); // or search pattern
        }
    }
}