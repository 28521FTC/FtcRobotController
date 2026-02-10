package org.firstinspires.ftc.teamcode.TeamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class GENSmallBlue extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    PathState pathState; // Current autonomous path state (state machine)
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
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain SHOOTPRE;
        public PathChain GOTORELOAD1;
        public PathChain RELOAD1;
        public PathChain SHOOT;
        public PathChain GOTORELOAD3;
        public PathChain RELOAD3;
        public PathChain SHOOT3;
        public PathChain Path8;

        public Paths(Follower follower) {
            SHOOTPRE = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.215, 8.000),

                                    new Pose(84.187, 9.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))

                    .build();

            GOTORELOAD1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.187, 9.700),

                                    new Pose(97.355, 35.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))

                    .build();

            RELOAD1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(97.355, 35.500),

                                    new Pose(135.800, 35.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            SHOOT = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.800, 35.500),

                                    new Pose(83.215, 8.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))

                    .build();

            GOTORELOAD3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.215, 8.000),

                                    new Pose(97.355, 84.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))

                    .build();

            RELOAD3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(97.355, 84.500),

                                    new Pose(130.000, 84.500)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            SHOOT3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.000, 84.500),

                                    new Pose(84.187, 9.757)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(70))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.187, 9.757),

                                    new Pose(72.000, 50.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(90))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    }
}