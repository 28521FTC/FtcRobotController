package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class BasicPath extends OpMode {
    DcMotorEx flywheelMotor;
    Servo hoodServo;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // START POSTION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,

        SHOOT_PRELOAD
    }
    PathState pathState;
    private final Pose startPose = new Pose(60.78504672897196,2.6915887850467293, Math.toRadians(90));
    private final Pose shootPose = new Pose(59.813,9.757, Math.toRadians(115));
    private PathChain driveStartShootPos;
    public void buildPaths() {
        driveStartShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
    }
    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartShootPos, true);
                pathState = PathState.SHOOT_PRELOAD;
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    hoodServo.setPosition(0.34);
                    flywheelMotor.setPower(0.75);
                }
                break;
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "topRight");

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

    }
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}
