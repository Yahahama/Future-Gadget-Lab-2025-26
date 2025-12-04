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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE_FAR_AUTONOMOUS", group = "Autonomous")
public class BlueFarAutonomous extends Autonomous {

    @Override
    public void runOpMode() {

        Positions.START startPos = Positions.START.BLUE_FAR;
        Positions.OBELISK obeliskPos = Positions.OBELISK.BLUE_FAR;
        Positions.LAUNCH launchPos = Positions.LAUNCH.BLUE_FAR;

        boolean isFar = true;

        Pose2d initialPose = startPos.getPose();

        Robot robot = new Robot(
                new Intake(hardwareMap),
                new Launch(hardwareMap),
                new Load(hardwareMap),
                new Bunt(hardwareMap),
                new MecanumDrive(hardwareMap, initialPose),
                new Camera(hardwareMap)
        );

        Actions.runBlocking(robot.Init());

        telemetry.addLine("Robot Initialized. Scanning for AprilTags");
        telemetry.update();

        int scannedTagID = -1;
        while (!isStopRequested() && !opModeIsActive()) {
            int newScanID = robot.camera.scanTagInit();
            if (newScanID != -1) {
                scannedTagID = newScanID;
                telemetry.addData("Last Scanned AprilTag ID", scannedTagID);
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
            firstArtifact = Positions.ARTIFACT.BLUE_C;
            firstArtifactCollect = Positions.ARTIFACT.BLUE_C_COLLECT;
        } else if (scannedTagID == 22) { //PGP
            firstArtifact = Positions.ARTIFACT.BLUE_C;
            firstArtifactCollect = Positions.ARTIFACT.BLUE_C_COLLECT;
        } else if (scannedTagID == 23) { //PPG
            firstArtifact = Positions.ARTIFACT.BLUE_C;
            firstArtifactCollect = Positions.ARTIFACT.BLUE_C_COLLECT;
        } else {
            preScanAction = new SequentialAction(
                    Positions.linearSplineTrajectory(robot, startPos, obeliskPos).build()
            );
            firstArtifact = Positions.ARTIFACT.BLUE_C;
            firstArtifactCollect = Positions.ARTIFACT.BLUE_C_COLLECT;
        }
        waitForStart();
        if (isStopRequested()) return;

        postScanAction = new SequentialAction(
                Positions.linearSplineTrajectory(robot, startPos, launchPos).build(),
                robot.shootPreload(scannedTagID, isFar),
                Positions.linearSplineTrajectory(robot, launchPos, firstArtifact).build(),
                robot.collectBalls(Positions.line(robot, firstArtifact, firstArtifactCollect)),
                Positions.linearSplineTrajectory(robot, firstArtifactCollect, launchPos).build(),
                robot.shootBalls(firstArtifact.letter(), scannedTagID, isFar)
        );

        Actions.runBlocking(
                new ParallelAction(
                        robot.intake.moveIntake(),
                        robot.launch.moveLaunch(),
                        new SequentialAction(
                            preScanAction,
                                robot.launch.stopLaunch(),
                                robot.intake.stopIntake()
                        )
                )
        );

        int timeoutCycles = 500;
        while (scannedTagID == -1 && timeoutCycles >= 0) {
            if (timeoutCycles == 0) {
                firstArtifact = Positions.ARTIFACT.BLUE_C;
                firstArtifactCollect = Positions.ARTIFACT.BLUE_C_COLLECT;
                timeoutCycles = -1;
                break;
            }

            telemetry.addLine("Scanning for AprilTag");
            telemetry.addData("Cycles remaining before timeout", timeoutCycles);
            telemetry.update();

            int newScanID = robot.camera.scanTagInit();
            if (newScanID == -1) {
                timeoutCycles--;
                continue;
            }

            scannedTagID = newScanID;
            telemetry.addData("New AprilTag identified. ID", scannedTagID);
            telemetry.update();

            if (scannedTagID == 21) { //GPP
                firstArtifact = Positions.ARTIFACT.BLUE_C;
                firstArtifactCollect = Positions.ARTIFACT.BLUE_C_COLLECT;
                timeoutCycles = -1;
                break;
            } else if (scannedTagID == 22) { //PGP
                firstArtifact = Positions.ARTIFACT.BLUE_C;
                firstArtifactCollect = Positions.ARTIFACT.BLUE_C_COLLECT;
                timeoutCycles = -1;
                break;
            } else if (scannedTagID == 23) { //PPG
                firstArtifact = Positions.ARTIFACT.BLUE_C;
                firstArtifactCollect = Positions.ARTIFACT.BLUE_C_COLLECT;
                timeoutCycles = -1;
                break;
            } else {
                scannedTagID = -1;
                timeoutCycles--;
            }
        }

        if (timeoutCycles == -1) {
            postScanAction = new SequentialAction(
                    Positions.linearSplineTrajectory(robot, obeliskPos, launchPos).build(),
                    robot.shootPreload(scannedTagID, isFar),
                    Positions.linearSplineTrajectory(robot, launchPos, firstArtifact).build(),
                    robot.collectBalls(Positions.line(robot, firstArtifact, firstArtifactCollect)),
                    Positions.linearSplineTrajectory(robot, firstArtifactCollect, launchPos).build(),
                    robot.shootBalls(firstArtifact.letter(), scannedTagID, isFar)
            );
        }

        Actions.runBlocking(
                new ParallelAction(
                        robot.intake.moveIntake(),
                        robot.launch.moveLaunch(),
                        new SequentialAction(
                                postScanAction
                        )
                )
        );
    }
}
