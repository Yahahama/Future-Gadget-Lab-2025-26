package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE_BUCKET_AUTONOMOUS", group = "Autonomous")
public class BlueCloseAutonomous extends Autonomous{


    @Override
    public void runOpMode() {
        Autonomous.START startPos = START.BLUE_CLOSE;

        Pose2d initialPose = startPos.getPose();
        Autonomous.Robot robot = new Robot(
                new Intake(hardwareMap), new Launch(hardwareMap), new Load(hardwareMap),
                new MecanumDrive(hardwareMap, initialPose));

        // Trajectories to select from

        /*
        Naming scheme for TrajectoryActionBuilders that go from buckets to blocks:
        {color of team}BucketTo{distance of target block from wall}{color of target block}Block
        ex.
        blueBucketToMiddleNeutralBlock

        Naming scheme for TrajectoryActionBuilders that go from blocks to buckets:
        {color of team}{distance of current block from wall}{color of block}BlockToBucket
        ex.
        blueMiddleNeutralBlockToBucket
        */

        TrajectoryActionBuilder blueInitToBucket = robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(Autonomous.Positions.BUCKET_BLUE, Math.toRadians(315));
        TrajectoryActionBuilder blueBucketToFarNeutralBlock = robot.drive.actionBuilder(Autonomous.Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(Autonomous.Positions.SAMPLE_NEUTRAL_BLUE_FAR, Math.toRadians(-50));
        TrajectoryActionBuilder blueFarNeutralBlockToBucket = robot.drive.actionBuilder(Autonomous.Positions.SAMPLE_NEUTRAL_BLUE_FAR)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(Autonomous.Positions.BUCKET_BLUE, Math.toRadians(45));
        TrajectoryActionBuilder blueBucketToMiddleNeutralBlock = robot.drive.actionBuilder(Autonomous.Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(-225))
                .splineToLinearHeading(Autonomous.Positions.SAMPLE_NEUTRAL_BLUE_MIDDLE, Math.toRadians(-90));
        TrajectoryActionBuilder blueMiddleNeutralBlockToBucket = robot.drive.actionBuilder(Autonomous.Positions.SAMPLE_NEUTRAL_BLUE_MIDDLE)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(Autonomous.Positions.BUCKET_BLUE, Math.toRadians(45));
        TrajectoryActionBuilder blueBucketToCloseNeutralBlock = robot.drive.actionBuilder(Autonomous.Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(-225))
                .splineToLinearHeading(Autonomous.Positions.SAMPLE_NEUTRAL_BLUE_CLOSE, Math.toRadians(-60));
        TrajectoryActionBuilder blueCloseNeutralBlockToBucket = robot.drive.actionBuilder(Autonomous.Positions.SAMPLE_NEUTRAL_BLUE_CLOSE)
                .setTangent(Math.toRadians(-120))
                .splineToLinearHeading(Autonomous.Positions.BUCKET_BLUE, Math.toRadians(45));
        TrajectoryActionBuilder blueBucketToSubmersible = robot.drive.actionBuilder(Autonomous.Positions.BUCKET_BLUE)
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(26, -10, Math.toRadians(0)), Math.toRadians(200));



        // Initialization Actions
        Actions.runBlocking(robot.Init());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("X Position during Init", robot.drive.pose.position.x);
            telemetry.addData("Y Position during Init", robot.drive.pose.position.y);
            telemetry.addData("Heading during Init", robot.drive.pose.heading.real);

            telemetry.update();
        }

        Action actionToExecute = new SequentialAction(
                robot.poseToBucket(blueInitToBucket),
                robot.bucketToSample(blueBucketToFarNeutralBlock),
                robot.poseToBucket(blueFarNeutralBlockToBucket),
                robot.bucketToSample(blueBucketToMiddleNeutralBlock),
                robot.poseToBucket(blueMiddleNeutralBlockToBucket),
                robot.bucketToSample(blueBucketToCloseNeutralBlock),
                robot.poseToBucket(blueCloseNeutralBlockToBucket),
                robot.bucketToSubmersible(blueBucketToSubmersible)
        );

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Run Pathing
        Actions.runBlocking(
                new ParallelAction(
                        robot.launch.moveLaunch(),
                        robot.intake.moveIntake(),
                        actionToExecute
                )
        );
    }
}
