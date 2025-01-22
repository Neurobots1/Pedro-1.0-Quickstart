package OpMode.Autonomous;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;

import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;


import OpMode.Subsystems.ViperSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "AutonomousMEGATEST", group = "Autonomous")
public class AutonomousMEGATEST extends OpMode {

    private Follower follower;
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;


    // Viper Slide Variables
    /*public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;
    private ViperSlides viperSlides;
    private FtcDashboard dashboard;

    // REV Touch Sensor (Limit Switch)
    private TouchSensor limitSwitch;

    viperSlides = new ViperSlides(
            hardwareMap.get(DcMotorEx.class, "slidemotorleft"),
                hardwareMap.get(DcMotorEx.class, "slidemotorright"),
            hardwareMap.get(TouchSensor.class, "limitSwitch"),
    p, i, d
        );*/



    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(8, 90, Math.toRadians(270));
    private final Pose scorePose = new Pose(15, 125, Math.toRadians(315));
    private final Pose pickup1Pose = new Pose(35, 120, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(35, 132, Math.toRadians(0));
    private final Pose parkPose = new Pose(62, 96, Math.toRadians(90));
    //Control Points
    private final Pose parkControlPose = new Pose(64, 134);

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain pathChain,pathChain1, pathChain2, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, scorePickup4;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */


          pathChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose))) // First path
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose))) // Second path
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .setPathEndTimeoutConstraint(10.0) // Timeout for the entire chain
                .build();


         pathChain1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose))) // First path
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose))) // Second path
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .setPathEndTimeoutConstraint(10.0) // Timeout for the entire chain
                .build();

        pathChain2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose))) // First path
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addPath(new BezierCurve(new Point(scorePose),/*controle point*/ new Point(parkControlPose), new Point(parkPose))) // Second path
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setPathEndTimeoutConstraint(10.0) // Timeout for the entire chain
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathChain, true);
                setPathState(1);
                break;

            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(pathTimer.getElapsedTimeSeconds()>4) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pathChain1,true);
                    setPathState(2);
                }
                break;

            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(pathTimer.getElapsedTimeSeconds()>4) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pathChain2,true);
                    setPathState(-1);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    follower.followPath(scorePickup4,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        poseUpdater.update();
        dashboardPoseTracker.update();

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", poseUpdater.getPose().getX());
        telemetry.addData("y", poseUpdater.getPose().getY());
        telemetry.addData("heading", poseUpdater.getPose().getHeading());
        telemetry.addData("total heading", poseUpdater.getTotalHeading());
        telemetry.update();


        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        poseUpdater = new PoseUpdater(hardwareMap);

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}