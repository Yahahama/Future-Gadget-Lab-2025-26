package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RED_FAR_AUTONOMOUS", group = "Autonomous")
public class RedFarAutonomous extends Autonomous {

    @Override
    public void runOpMode() {

        Positions.START startPos = Positions.START.RED_FAR;
        Positions.OBELISK obeliskPos = Positions.OBELISK.RED_FAR;
        Positions.LAUNCH launchPos = Positions.LAUNCH.RED_FAR;

        Pose2d initialPose = startPos.getPose();

        Robot robot = new Robot(
                new Intake(hardwareMap),
                new Launch(hardwareMap),
                new Load(hardwareMap),
                new Bunt(hardwareMap),
                new MecanumDrive(hardwareMap, initialPose),
                new Camera(hardwareMap)
        );

        boolean isFar = true;

//        Positions.ARTIFACT detectedTag;
//        if (artifactLetter == 'A') detectedTag = Positions.ARTIFACT.RED_A;
//        else if (artifactLetter == 'B') detectedTag = Positions.ARTIFACT.RED_B;
//        else detectedTag = Positions.ARTIFACT.RED_C;
//
//        Pose2d target = detectedTag.getPose();
//
//        Pose2d redApproachPoint = new Pose2d(target.position.x, target.position.y - 20, Math.toRadians(90));
//        Pose2d redFarObeliskPose = new Pose2d(23, 12, Math.toRadians(187));
//
//        Pose2d chosenObeliskPose = redFarObeliskPose;
//        Pose2d approachPoint = redApproachPoint;
//
//        // NEW FINAL PARK
//        Pose2d finalPark = new Pose2d(60, 12, Math.toRadians(158+180));
//
//        double approachHeading = Math.toRadians(90);
//        double secondsToWait = 1;
//
//        Action s1 = robot.drive.actionBuilder(initialPose)
//                .setTangent(Math.toRadians(197))
//                .splineToLinearHeading(chosenObeliskPose, chosenObeliskPose.heading.real)
//                .waitSeconds(secondsToWait)
//                .build();
//
//        Action s2 = robot.drive.actionBuilder(chosenObeliskPose)
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(approachPoint, approachHeading)
//                .build();
//
//        Action s3 = robot.drive.actionBuilder(approachPoint)
//                .setTangent(Math.toRadians(270))
//                .lineToY(target.position.y, new TranslationalVelConstraint(10))
//                .build();
//
//        Action s4 = robot.drive.actionBuilder(
//                        new Pose2d(approachPoint.position.x, target.position.y, approachHeading))
//                .setTangent(Math.toRadians(90))
//                .lineToY(approachPoint.position.y - 10)
//                .build();
//
//        Action s5 = robot.drive.actionBuilder(
//                        new Pose2d(
//                                approachPoint.position.x,
//                                approachPoint.position.y - 10,
//                                Math.toRadians(270)))
//                .setTangent(Math.toRadians(-225))
//                .splineToLinearHeading(finalPark, finalPark.heading.real)
//                .build();

        Actions.runBlocking(robot.Init());

        int scannedTagID = -1;
        while (!isStopRequested() && !opModeIsActive()) {
            int newScanID = robot.camera.scanTagInit();
            if (newScanID == -1) {
                continue;
            } else {
                scannedTagID = newScanID;
                telemetry.addData("ID", scannedTagID);
                telemetry.update();
            }
        }

        Positions.ARTIFACT firstArtifact;
        Positions.ARTIFACT firstArtifactCollect;
        Action preScanAction = new SleepAction(0);
        Action postScanAction;

        //a == ppg
        //b == pgp
        //c = gpp
        if (scannedTagID == 21) { //GPP
            firstArtifact = Positions.ARTIFACT.RED_C;
            firstArtifactCollect = Positions.ARTIFACT.RED_C_COLLECT;
        } else if (scannedTagID == 22) { //PGP
            firstArtifact = Positions.ARTIFACT.RED_C;
            firstArtifactCollect = Positions.ARTIFACT.RED_C_COLLECT;
        } else if (scannedTagID == 23) { //PPG
            firstArtifact = Positions.ARTIFACT.RED_C;
            firstArtifactCollect = Positions.ARTIFACT.RED_C_COLLECT;
        } else {
            preScanAction = new SequentialAction(
                    Positions.linearSplineTrajectory(robot, startPos, obeliskPos).build()
            );
            firstArtifact = Positions.ARTIFACT.RED_C;
            firstArtifactCollect = Positions.ARTIFACT.RED_C_COLLECT;
        }
        waitForStart();
        if (isStopRequested()) return;

        TrajectoryActionBuilder startToArtifact = Positions.linearSplineTrajectory(robot, startPos, firstArtifact);
        TrajectoryActionBuilder artifactToCollect = Positions.line(robot, firstArtifact, firstArtifactCollect);
        TrajectoryActionBuilder collectToLaunch = Positions.linearSplineTrajectory(robot, firstArtifactCollect, launchPos);

        postScanAction = new SequentialAction(
                startToArtifact.build(),
                robot.collectBalls(artifactToCollect),
                collectToLaunch.build(),
                robot.shootBalls(firstArtifact.letter(), scannedTagID, isFar)
        );

        Actions.runBlocking(
                new ParallelAction(
                        robot.intake.moveIntake(),
                        robot.launch.moveLaunch(),
                        preScanAction
                )
        );

        if (scannedTagID == -1) {
            int timeoutCycles = 500;
            while (timeoutCycles > 0) {
                int newScanID = robot.camera.scanTagInit();
                if (newScanID == -1) {
                    timeoutCycles--;
                } else {
                    scannedTagID = newScanID;
                    if (scannedTagID == 21) { //GPP
                        firstArtifact = Positions.ARTIFACT.RED_C;
                        firstArtifactCollect = Positions.ARTIFACT.RED_C_COLLECT;
                        break;
                    } else if (scannedTagID == 22) { //PGP
                        firstArtifact = Positions.ARTIFACT.RED_C;
                        firstArtifactCollect = Positions.ARTIFACT.RED_C_COLLECT;
                        break;
                    } else if (scannedTagID == 23) { //PPG
                        firstArtifact = Positions.ARTIFACT.RED_C;
                        firstArtifactCollect = Positions.ARTIFACT.RED_C_COLLECT;
                        break;
                    } else {
                        firstArtifact = Positions.ARTIFACT.RED_C;
                        firstArtifactCollect = Positions.ARTIFACT.RED_C_COLLECT;
                        break;
                    }
                }
            }
            if (timeoutCycles == 0) {
                firstArtifact = Positions.ARTIFACT.RED_C;
                firstArtifactCollect = Positions.ARTIFACT.RED_C_COLLECT;
            }

            postScanAction = new SequentialAction(
                    Positions.linearSplineTrajectory(robot, obeliskPos, firstArtifact).build(),
                    robot.collectBalls(Positions.line(robot, firstArtifact, firstArtifactCollect)),
                    Positions.linearSplineTrajectory(robot, firstArtifactCollect, launchPos).build(),
                    robot.shootBalls(firstArtifact.letter(), scannedTagID, isFar)
            );

        }

        Actions.runBlocking(
                postScanAction
        );
    }
}
