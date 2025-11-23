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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE_CLOSE_AUTONOMOUS", group = "Autonomous")
public class BlueCloseAutonomous extends Autonomous {

    @Override
    public void runOpMode() {

        Autonomous.Positions.START startPos = Autonomous.Positions.START.BLUE_CLOSE;
        char artifactLetter = 'C';

        Pose2d initialPose = startPos.getPose();

        Robot robot = new Robot(
                new Intake(hardwareMap),
                new Launch(hardwareMap),
                new Load(hardwareMap),
                new Bunt(hardwareMap),
                new MecanumDrive(hardwareMap, initialPose)
        );

        boolean isRed = false;
        boolean isClose = true;

        // ARTIFACT
        Positions.ARTIFACT detectedTag;
        if (artifactLetter == 'A') detectedTag = Positions.ARTIFACT.BLUE_A;
        else if (artifactLetter == 'B') detectedTag = Positions.ARTIFACT.BLUE_B;
        else detectedTag = Positions.ARTIFACT.BLUE_C;

        Pose2d target = detectedTag.getPose();

        // APPROACH POINTS
        Pose2d blueApproachPoint = new Pose2d(target.position.x, target.position.y + 20, Math.toRadians(270));

        // OBELISK POSE
        Pose2d blueCloseObeliskPose = new Pose2d(-32, -32, Math.toRadians(135));

        Pose2d chosenObeliskPose = blueCloseObeliskPose;
        Pose2d approachPoint = blueApproachPoint;

        // -------------------------------
        // NEW FINAL PARKING LOGIC (SWAPPED)
        // -------------------------------
        Pose2d finalPark = new Pose2d(-12, -12, Math.toRadians(225));

        double approachHeading = Math.toRadians(270);
        double secondsToWait = 1;

        // ------------------- MOVEMENT SEQUENCE S1–S5 -------------------

        Action s1 = robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(chosenObeliskPose, chosenObeliskPose.heading.real)
                .waitSeconds(secondsToWait)
                .build();

        Action s2 = robot.drive.actionBuilder(chosenObeliskPose)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(approachPoint, approachHeading)
                .build();

        Action s3 = robot.drive.actionBuilder(approachPoint)
                .setTangent(Math.toRadians(90))
                .lineToY(target.position.y, new TranslationalVelConstraint(10))
                .build();

        Action s4 = robot.drive.actionBuilder(
                        new Pose2d(approachPoint.position.x, target.position.y, approachHeading))
                .setTangent(Math.toRadians(270))
                .lineToY(approachPoint.position.y + 10)
                .build();

        // ✔ NEW FINAL SPLINE TO THE UPDATED PARK POSITION
        Action s5 = robot.drive.actionBuilder(
                        new Pose2d(
                                approachPoint.position.x,
                                approachPoint.position.y + 10,
                                Math.toRadians(90)))
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(finalPark, finalPark.heading.real)
                .build();

        // -------------------- INIT --------------------
        Actions.runBlocking(robot.Init());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Init X", robot.drive.pose.position.x);
            telemetry.addData("Init Y", robot.drive.pose.position.y);
            telemetry.addData("Init Heading", robot.drive.pose.heading.real);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // -------------------- RUN FULL AUTO --------------------
        Actions.runBlocking(
                new ParallelAction(
                        robot.intake.moveIntake(),
                        robot.launch.moveLaunch(),
                        new SequentialAction(
                                s1,
                                s2,
                                robot.intake.intakeLoad(),
                                s3,
                                robot.intake.intakeOff(),
                                s4,
                                s5,
                                robot.launch.launchClose(),
                                new SleepAction(1),
                                robot.bunt.buntLaunch(),
                                new SleepAction(1),
                                robot.bunt.buntReset(),
                                new SleepAction(1),
                                robot.intake.intakeLoad(),
                                new SleepAction(1.5),
                                robot.intake.intakeOff(),
                                new SleepAction(0.75),
                                robot.bunt.buntLaunch(),
                                new SleepAction(1),
                                robot.bunt.buntReset(),
                                robot.load.loadLoad(),
                                new SleepAction(1),
                                robot.bunt.buntLaunch(),
                                new SleepAction(1),
                                robot.bunt.buntReset()
                        )
                )
        );
    }
}
