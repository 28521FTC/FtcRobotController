package org.firstinspires.ftc.teamcode.TeamCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class SmallBlue extends OpMode {
    DcMotorEx flywheelMotor;
    Servo hoodServo;
    DcMotor inTake;
    DcMotor transferMotor;

    private Follower follower;
    private Timer pathTimer, opModeTimer, nextPath;

    public enum PathState {
        // START POSTION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOTPOS,

        SHOOT_PRELOAD,
        SHOOT_1,
        SHOOT_3,

        RELOAD1,
        RELOAD2,
        RELOAD3,
        END
    }
    PathState pathState;
    private final Pose startPose = new Pose(83.21495327102804,2.6915887850467293, Math.toRadians(90));
    private final Pose endPose = new Pose(72,50, Math.toRadians(90));

    private final Pose shootPose = new Pose(84.187,9.757, Math.toRadians(70));
    private final Pose pickupStation1Start = new Pose(97.35514018691588,33,Math.toRadians(0));
    private final Pose pickupStation1End = new Pose(136,33,Math.toRadians(0));
    private final Pose pickupStation3Start = new Pose(97.35514018691588,84.5,Math.toRadians(0));
    private final Pose pickupStation3End = new Pose(130,84.5,Math.toRadians(0));


    private PathChain driveStartShootPos;
    private PathChain driveReloadPos1;
    private PathChain driveShootPos1;
    private PathChain driveReloadPos3;
    private PathChain driveShootPos3;
    private PathChain endPos;



    public void buildPaths() {
        driveStartShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        driveReloadPos1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupStation1Start,pickupStation1End))
                .setLinearHeadingInterpolation(pickupStation1Start.getHeading(),pickupStation1End.getHeading())
                .build();
        driveReloadPos3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupStation3Start,pickupStation3End))
                .setLinearHeadingInterpolation(pickupStation3Start.getHeading(),pickupStation3End.getHeading())
                .build();
        driveShootPos1 = follower.pathBuilder()
                .addPath(new BezierLine(pickupStation1End,endPose))
                .setLinearHeadingInterpolation(pickupStation1End.getHeading(),shootPose.getHeading())
                .build();
        driveShootPos3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupStation3End,shootPose))
                .setLinearHeadingInterpolation(pickupStation3End.getHeading(),shootPose.getHeading())
                .build();
        endPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }
    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartShootPos);
                pathState = PathState.SHOOT_PRELOAD;
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    hoodServo.setPosition(0.08);
                    flywheelMotor.setPower(1);
                    transferMotor.setPower(0.2);
                    if (pathTimer.getElapsedTimeSeconds() >= 6) {
                        flywheelMotor.setPower(0);
                        transferMotor.setPower(0);
                        pathState = PathState.RELOAD1;
                        setPathState(PathState.RELOAD1);
                    }
                }
                break;
            case SHOOT_1:
                follower.followPath(driveShootPos1,true);
                if (!follower.isBusy()) {
                    hoodServo.setPosition(0.08);
                    flywheelMotor.setPower(1);
                    transferMotor.setPower(0.3);
                    if (pathTimer.getElapsedTimeSeconds() >= 6) {
                        flywheelMotor.setPower(0);
                        transferMotor.setPower(0);
                        pathState = PathState.RELOAD3;
                        setPathState(PathState.RELOAD3);
                    }
                }
                break;
            case SHOOT_3:
                follower.followPath(driveShootPos3);
                if (!follower.isBusy()) {
                    hoodServo.setPosition(0.08);
                    flywheelMotor.setPower(1);
                    transferMotor.setPower(0.3);
                    if (pathTimer.getElapsedTimeSeconds() >= 6) {
                        flywheelMotor.setPower(0);
                        transferMotor.setPower(0);
                        pathState = PathState.RELOAD3;
                        setPathState(PathState.RELOAD3);
                    }
                }
                break;
            case RELOAD1:
                follower.followPath(driveReloadPos1);
                inTake.setPower(1);
                transferMotor.setPower(0.2);
                if (pathTimer.getElapsedTimeSeconds() >= 6) {
                    inTake.setPower(0);
                    transferMotor.setPower(0);
                    pathState = PathState.SHOOT_1;
                    setPathState(PathState.SHOOT_1);
                }
                break;
            case RELOAD3:
                follower.followPath(driveReloadPos3);
                inTake.setPower(1);
                transferMotor.setPower(0.2);
                if (pathTimer.getElapsedTimeSeconds() >= 6) {
                    inTake.setPower(0);
                    transferMotor.setPower(0);
                    pathState = PathState.SHOOT_1;
                    setPathState(PathState.SHOOT_1);
                }
                break;
            case END:
                follower.followPath(endPos,true);
                if (!follower.isBusy()) {
                    pathState = PathState.END;
                    setPathState(PathState.END);
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
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        inTake = hardwareMap.get(DcMotor.class, "inTake");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        inTake.setDirection(DcMotorSimple.Direction.REVERSE);


        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        nextPath = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

    }
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.DRIVE_STARTPOS_SHOOTPOS);
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
