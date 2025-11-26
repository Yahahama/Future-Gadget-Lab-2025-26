package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RED_FAR_AUTONOMOUS", group = "Autonomous")
public class RedFarAutonomous extends Autonomous {

    @Override
    public void runOpMode() {

        Autonomous.Positions.START startPos = Autonomous.Positions.START.RED_FAR;
        char artifactLetter = 'C';

        Pose2d initialPose = startPos.getPose();

        Robot robot = new Robot(
                new Intake(hardwareMap),
                new Launch(hardwareMap),
                new Load(hardwareMap),
                new Bunt(hardwareMap),
                new MecanumDrive(hardwareMap, initialPose)
        );

        boolean isRed = true;
        boolean isClose = false;

        Positions.ARTIFACT detectedTag;
        if (artifactLetter == 'A') detectedTag = Positions.ARTIFACT.RED_A;
        else if (artifactLetter == 'B') detectedTag = Positions.ARTIFACT.RED_B;
        else detectedTag = Positions.ARTIFACT.RED_C;

        Pose2d target = detectedTag.getPose();

        Pose2d redApproachPoint = new Pose2d(target.position.x, target.position.y - 20, Math.toRadians(90));
        Pose2d redFarObeliskPose = new Pose2d(23, 12, Math.toRadians(187));

        Pose2d chosenObeliskPose = redFarObeliskPose;
        Pose2d approachPoint = redApproachPoint;

        // NEW FINAL PARK
        Pose2d finalPark = new Pose2d(60, 12, Math.toRadians(158+180));

        double approachHeading = Math.toRadians(90);
        double secondsToWait = 1;

        Action s1 = robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(197))
                .splineToLinearHeading(chosenObeliskPose, chosenObeliskPose.heading.real)
                .waitSeconds(secondsToWait)
                .build();

        Action s2 = robot.drive.actionBuilder(chosenObeliskPose)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(approachPoint, approachHeading)
                .build();

        Action s3 = robot.drive.actionBuilder(approachPoint)
                .setTangent(Math.toRadians(270))
                .lineToY(target.position.y, new TranslationalVelConstraint(10))
                .build();

        Action s4 = robot.drive.actionBuilder(
                        new Pose2d(approachPoint.position.x, target.position.y, approachHeading))
                .setTangent(Math.toRadians(90))
                .lineToY(approachPoint.position.y - 10)
                .build();

        Action s5 = robot.drive.actionBuilder(
                        new Pose2d(
                                approachPoint.position.x,
                                approachPoint.position.y - 10,
                                Math.toRadians(270)))
                .setTangent(Math.toRadians(-225))
                .splineToLinearHeading(finalPark, finalPark.heading.real)
                .build();

        Actions.runBlocking(robot.Init());
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        robot.intake.moveIntake(),
                        robot.launch.moveLaunch(),
                        new SequentialAction(
        //                        s1,
        //                        s2,
                                robot.intake.intakeLoad(),
        //                        s3,
                                robot.intake.intakeOff(),
        //                        s4,
        //                        s5,
                                robot.loadIntakeIntoHigh(0.5f),
                                robot.launch.launchFar(),
                                robot.shootHigh(1),
                                new SleepAction(0.75),
                                robot.loadIntakeIntoHigh(1.5f),
                                robot.shootHigh(1),
                                robot.launch.launchDrop(),
                                robot.loadIntakeIntoLow(0.25f),
                                robot.launch.launchFar(),
                                robot.shootLow(0.5f)
                        )
                )
        );
    }
}
