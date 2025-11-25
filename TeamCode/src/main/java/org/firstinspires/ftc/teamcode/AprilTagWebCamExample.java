package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
public class AprilTagWebCamExample extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight= hardwareMap.get(DcMotor.class, "frontRight");

    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);

        if (id20 != null) {
            telemetry.addData("id20 String", id20.toString());
            aprilTagWebcam.displayDetectionTelemetry(id20);
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        } else {
            telemetry.addData("id20 String", "Not detected");
            backLeft.setPower(-0.3);
            backRight.setPower(0.3);
            frontLeft.setPower(-0.3);
            frontRight.setPower(0.3);
        }
    }
}