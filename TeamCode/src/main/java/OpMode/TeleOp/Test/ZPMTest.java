package OpMode.TeleOp.Test;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

public class ZPMTest extends OpMode {
    private Follower follower;

    private int pathState;
    private final Pose startPose = new Pose(7, 104, Math.toRadians(270));
    private final Pose startPathPose = new Pose(19,100, Math.toRadians(-90));
    private final Pose testPose = new Pose(19,50, Math.toRadians(-90));

    private Path scorePreload, park;
    private PathChain startPath, testPath1, testPath2, testPath3, recoveryPath;

    public void autonomousPathUpdate() {
        startPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(startPathPose)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(),startPathPose.getHeading())
                .build();


        testPath1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPathPose),
                        new Point(testPose)
                ))
                .setZeroPowerAccelerationMultiplier(2)
                .setLinearHeadingInterpolation(startPathPose.getHeading(),testPose.getHeading())
                .build();

        testPath2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPathPose),
                        new Point(testPose)
                ))
                .setLinearHeadingInterpolation(startPathPose.getHeading(),testPose.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        testPath3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPathPose),
                        new Point(testPose)
                ))
                .setLinearHeadingInterpolation(startPathPose.getHeading(),testPose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        recoveryPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(follower.getPose()),
                        new Point(startPathPose)
                ))
                .setLinearHeadingInterpolation(follower.getTotalHeading(), startPathPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
    }



    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
        follower.followPath(startPath,0.5,true);
        autonomousPathUpdate();
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        if (gamepad1.a){
            follower.followPath(testPath1,1,false);
        }

        if (gamepad1.b){
            follower.followPath(testPath2,1,false);
        }
        if (gamepad1.y){
            follower.followPath(testPath3,1,false);
        }

        if (gamepad1.x){
            follower.followPath(recoveryPath,0.5,true);
        }


        Pose currentPose = follower.getPose();
        telemetry.addData("XOffset", follower.getPose().getX()- testPose.getX());
        telemetry.addData("YOffset", follower.getPose().getY()-testPose.getY());
        telemetry.addData("Heading (Degrees)", Math.toDegrees(currentPose.getHeading()));
        telemetry.update();
        autonomousPathUpdate();
    }
}
