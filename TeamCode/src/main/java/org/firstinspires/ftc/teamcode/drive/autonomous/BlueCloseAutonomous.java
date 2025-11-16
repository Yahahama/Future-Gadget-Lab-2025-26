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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE_BUCKET_AUTONOMOUS", group = "Autonomous")
public class BlueCloseAutonomous extends Autonomous {

    @Override
    public void runOpMode() {

        Autonomous.Positions.START startPos = Autonomous.Positions.START.BLUE_CLOSE; // fixed for this opmode
        char artifactLetter = 'C'; // todo: replace with detection result: 'A', 'B', or 'C'

        Pose2d initialPose = startPos.getPose();

        Robot robot = new Robot(
                new Intake(hardwareMap),
                new Launch(hardwareMap),
                new Load(hardwareMap),
                new Bunt(hardwareMap),
                new MecanumDrive(hardwareMap, initialPose)
        );

        boolean isRed = (startPos == Autonomous.Positions.START.RED_CLOSE || startPos == Autonomous.Positions.START.RED_FAR);
        boolean isClose = (startPos == Autonomous.Positions.START.RED_CLOSE || startPos == Autonomous.Positions.START.BLUE_CLOSE);

        // pick correct artifact automatically
        Autonomous.Positions.ARTIFACT detectedTag;
        if (isRed) {
            if (artifactLetter == 'A') {
                detectedTag = Autonomous.Positions.ARTIFACT.RED_A;
            } else if (artifactLetter == 'B') {
                detectedTag = Autonomous.Positions.ARTIFACT.RED_B;
            } else {
                detectedTag = Autonomous.Positions.ARTIFACT.RED_C;
            }
        } else {
            if (artifactLetter == 'A') {
                detectedTag = Autonomous.Positions.ARTIFACT.BLUE_A;
            } else if (artifactLetter == 'B') {
                detectedTag = Autonomous.Positions.ARTIFACT.BLUE_B;
            } else {
                detectedTag = Autonomous.Positions.ARTIFACT.BLUE_C;
            }
        }

        Pose2d target = detectedTag.getPose();

        // approach points (up for red, down for blue)
        Pose2d redApproachPoint = new Pose2d(target.position.x, target.position.y - 20, Math.toRadians(90));
        Pose2d blueApproachPoint = new Pose2d(target.position.x, target.position.y + 20, Math.toRadians(270));

        // obelisk poses
        Pose2d redCloseObeliskPose = new Pose2d(-32, 32, Math.toRadians(225));
        Pose2d redFarObeliskPose = new Pose2d(23, 12, Math.toRadians(187));
        Pose2d blueCloseObeliskPose = new Pose2d(-32, -32, Math.toRadians(135));
        Pose2d blueFarObeliskPose = new Pose2d(23, -12, Math.toRadians(173));

        // team goals
        Pose2d redGoal = Autonomous.Positions.GOAL.RED.getPose();
        Pose2d blueGoal = Autonomous.Positions.GOAL.BLUE.getPose();

        Pose2d approachPoint;
        Pose2d chosenObeliskPose;

        double secondsToWait = 1f;

        if (isRed && isClose) {
            approachPoint = redApproachPoint;
            chosenObeliskPose = redCloseObeliskPose;
        } else if (!isRed && isClose) {
            approachPoint = blueApproachPoint;
            chosenObeliskPose = blueCloseObeliskPose;
        } else if (isRed && !isClose) {
            approachPoint = redApproachPoint;
            chosenObeliskPose = redFarObeliskPose;
        } else {
            approachPoint = blueApproachPoint;
            chosenObeliskPose = blueFarObeliskPose;
        }

        // new goal selection using XOR
        boolean goToRedGoal = (isRed ^ isClose);
        Pose2d teamGoal = goToRedGoal ? redGoal : blueGoal;

        double approachHeading = isRed ? Math.toRadians(90) : Math.toRadians(270);

        // make a point in front of the goal (stop before entering)
        double stopDistance = 22; // how far to stop before goal
        Pose2d goalFront = new Pose2d(
                teamGoal.position.x + stopDistance + 21,
                teamGoal.position.y + (isRed ? -stopDistance : stopDistance),
                teamGoal.heading.real
        );

        // s1–s5 sequence (same as original, now per-opmode)
        Action s1 = robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(isRed ? (isClose ? -45 : 197) : (isClose ? 45 : 163)))
                .splineToLinearHeading(
                        chosenObeliskPose,
                        isClose ? chosenObeliskPose.heading.real :
                                (isRed ? Math.toRadians(225) : Math.toRadians(135))
                )
                .waitSeconds(secondsToWait)
                .build();

        Action s2 = robot.drive.actionBuilder(chosenObeliskPose)
                .setTangent(Math.toRadians(isRed ? 45 : -45))
                .splineToLinearHeading(approachPoint, approachHeading)
                .build();

        Action s3 = robot.drive.actionBuilder(approachPoint)
                .setTangent(Math.toRadians(90))
                .lineToY(target.position.y, new TranslationalVelConstraint(10))
                .build();

        Action s4 = robot.drive.actionBuilder(
                        new Pose2d(approachPoint.position.x, target.position.y, approachPoint.heading.real))
                .setTangent(Math.toRadians(270))
                .lineToY(approachPoint.position.y + (isRed ? -10 : 10))
                .build();

        Action s5 = robot.drive.actionBuilder(
                        new Pose2d(approachPoint.position.x,
                                approachPoint.position.y + (isRed ? -10 : 10),
                                Math.toRadians(90)))
                .setTangent(Math.toRadians(isRed ? -225 : 225))
                .splineTo(goalFront.position, Math.toRadians(isRed ? 135 : 225))
                .turnTo(Math.toRadians(240))
                .build();

        // initialization actions
        Actions.runBlocking(robot.Init());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("X Position during Init", robot.drive.pose.position.x);
            telemetry.addData("Y Position during Init", robot.drive.pose.position.y);
            telemetry.addData("Heading during Init", robot.drive.pose.heading.real);
            telemetry.update();
        }

        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // run pathing + scoring sequence (same as original)
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
