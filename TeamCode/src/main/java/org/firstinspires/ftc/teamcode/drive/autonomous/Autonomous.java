package org.firstinspires.ftc.teamcode.drive.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTONOMOUS", group = "Autonomous")
public class Autonomous extends LinearOpMode {
    public static class Positions {
        enum GOAL {
            RED(new Pose2d(-53, 48, Math.toRadians(135))),
            BLUE(new Pose2d(-53, -48, Math.toRadians(225)));

            private final Pose2d pose2d;

            GOAL(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }

        enum START {
            RED_CLOSE(new Pose2d(-49, 48, Math.toRadians(-45)), Math.toRadians(-45)),
            RED_FAR(new Pose2d(62, 23, Math.toRadians(0)), Math.toRadians(180)),
            BLUE_CLOSE(new Pose2d(-49, -48, Math.toRadians(45)), Math.toRadians(45)),
            BLUE_FAR(new Pose2d(62, -23, Math.toRadians(0)), Math.toRadians(180));

            private final Pose2d pose2d;
            private final double outHeading;

            START(Pose2d _pose2d, double _outHeading) {
                this.pose2d = _pose2d;
                this.outHeading = _outHeading;
            }

            public Pose2d getPose() {
                return pose2d;
            }

            public double out() {
                return outHeading;
            }
        }

        enum ARTIFACT {
            RED_A(new Pose2d(-11.5f, 26.5f, Math.toRadians(90)), Math.toRadians(30), Math.toRadians(-90), 'A'),
            RED_B(new Pose2d(12.3f, 26.5f, Math.toRadians(90)), Math.toRadians(90), Math.toRadians(-90), 'B'),
            RED_C(new Pose2d(35.75, 26.5f, Math.toRadians(90)), Math.toRadians(90), Math.toRadians(-90), 'C'),
            BLUE_A(new Pose2d(-11.5f, -26.5f, Math.toRadians(-90)), Math.toRadians(-30), Math.toRadians(90), 'A'),
            BLUE_B(new Pose2d(12.3f, -26.5f, Math.toRadians(-90)), Math.toRadians(-90), Math.toRadians(90), 'B'),
            BLUE_C(new Pose2d(35.75, -26.5f, Math.toRadians(-90)), Math.toRadians(-90), Math.toRadians(90), 'C'),
            RED_A_COLLECT(new Pose2d(-11.5f, 46.5f, Math.toRadians(90)), Math.toRadians(90), Math.toRadians(-90), 'A'),
            RED_B_COLLECT(new Pose2d(12.3f, 46.5f, Math.toRadians(90)), Math.toRadians(90), Math.toRadians(-90), 'B'),
            RED_C_COLLECT(new Pose2d(35.75, 46.5f, Math.toRadians(90)), Math.toRadians(90), Math.toRadians(-90), 'C'),
            BLUE_A_COLLECT(new Pose2d(-11.5f, -46.5f, Math.toRadians(-90)), Math.toRadians(-90), Math.toRadians(90), 'A'),
            BLUE_B_COLLECT(new Pose2d(12.3f, -46.5f, Math.toRadians(-90)), Math.toRadians(-90), Math.toRadians(90), 'B'),
            BLUE_C_COLLECT(new Pose2d(35.75, -46.5f, Math.toRadians(-90)), Math.toRadians(-90), Math.toRadians(90), 'C');

            private final Pose2d pose2d;
            private final double inHeading;
            private final double outHeading;
            private final char set;

            ARTIFACT(Pose2d _pose2d, double _inHeading, double _outHeading, char _set) {
                this.pose2d = _pose2d;
                this.inHeading = _inHeading;
                this.outHeading = _outHeading;
                this.set = _set;
            }

            public Pose2d getPose() {
                return pose2d;
            }

            public double in() {
                return inHeading;
            }

            public double out() {
                return outHeading;
            }

            public char letter() {
                return set;
            }
        }

        enum OBELISK {
            RED_CLOSE(new Pose2d(-32, 32, Math.toRadians(45)), Math.toRadians(-45), Math.toRadians(-30)),
            RED_FAR(new Pose2d(23, 12, Math.toRadians(7)), Math.toRadians(-173), Math.toRadians(90)),
            BLUE_CLOSE(new Pose2d(-32, -32, Math.toRadians(-45)), Math.toRadians(45), Math.toRadians(30)),
            BLUE_FAR(new Pose2d(23, -12, Math.toRadians(-7)), Math.toRadians(173), Math.toRadians(-90));

            private final Pose2d pose2d;
            private final double inHeading;
            private final double outHeading;

            OBELISK(Pose2d _pose2d, double _inHeading, double _outHeading) {
                this.pose2d = _pose2d;
                this.inHeading = _inHeading;
                this.outHeading = _outHeading;
            }

            public Pose2d getPose() {
                return pose2d;
            }

            public double in() {
                return inHeading;
            }

            public double out() {
                return outHeading;
            }
        }

        enum LAUNCH {
            RED_CLOSE(new Pose2d(-12, 12, Math.toRadians(-45)), Math.toRadians(-135), Math.toRadians(15)),
            RED_FAR(new Pose2d(58, 14, Math.toRadians(-22)), Math.toRadians(-10), Math.toRadians(-170)),
            BLUE_CLOSE(new Pose2d(-12, -12, Math.toRadians(45)), Math.toRadians(135), Math.toRadians(-15)),
            BLUE_FAR(new Pose2d(58, -14, Math.toRadians(22)), Math.toRadians(10), Math.toRadians(170));

            private final Pose2d pose2d;
            private final double inHeading;
            private final double outHeading;

            LAUNCH(Pose2d _pose2d, double _inHeading, double _outHeading) {
                this.pose2d = _pose2d;
                this.inHeading = _inHeading;
                this.outHeading = _outHeading;
            }

            public Pose2d getPose() {
                return pose2d;
            }

            public double in() {
                return inHeading;
            }
             public double out() {
                return outHeading;
             }
        }

        enum PARKING {
            PARK_RED(new Pose2d(38.25f, 32, Math.toRadians(0))),
            PARK_BLUE(new Pose2d(38.25f, -32, Math.toRadians(0)));

            private final Pose2d pose2d;

            PARKING(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }

        static TrajectoryActionBuilder linearSplineTrajectory(Robot robot, START start, ARTIFACT artifact) {
            return robot.drive.actionBuilder(start.getPose())
                    .setTangent(start.out())
                    .splineToLinearHeading(artifact.getPose(), artifact.in());
        }

        static TrajectoryActionBuilder linearSplineTrajectory(Robot robot, START start, OBELISK obelisk) {
            return robot.drive.actionBuilder(start.getPose())
                    .setTangent(start.out())
                    .splineToLinearHeading(obelisk.getPose(), obelisk.in());
        }

        static TrajectoryActionBuilder linearSplineTrajectory(Robot robot, OBELISK obelisk, ARTIFACT artifact) {
            return robot.drive.actionBuilder(obelisk.getPose())
                    .setTangent(obelisk.out())
                    .splineToLinearHeading(artifact.getPose(), artifact.in());
        }

        static TrajectoryActionBuilder linearSplineTrajectory(Robot robot, ARTIFACT artifact, LAUNCH launch) {
            return robot.drive.actionBuilder(artifact.getPose())
                    .setTangent(artifact.out())
                    .splineToLinearHeading(launch.getPose(), launch.in());
        }

        static TrajectoryActionBuilder line(Robot robot, ARTIFACT artifact, ARTIFACT artifactCollect) {
            return robot.drive.actionBuilder(artifact.getPose())
                    .setTangent(artifact.out())
                    .lineToY(artifactCollect.getPose().position.y, new TranslationalVelConstraint(9));
        }

        static TrajectoryActionBuilder linearSplineTrajectory(Robot robot, LAUNCH launch, ARTIFACT artifact) {
            return robot.drive.actionBuilder(launch.getPose())
                    .setTangent(launch.out())
                    .splineToLinearHeading(artifact.getPose(), artifact.in());
        }

        static TrajectoryActionBuilder linearSplineTrajectory(Robot robot, START start, LAUNCH launch) {
            double diffX = launch.getPose().position.x - start.getPose().position.x;
            double diffY = launch.getPose().position.y - start.getPose().position.y;
            double tangent = Math.toRadians(Math.atan2(diffX, diffY));
            if (diffX < 0) {
                tangent += Math.toRadians(180);
            }
            return robot.drive.actionBuilder(start.getPose())
                    .setTangent(tangent)
                    .lineToXLinearHeading(launch.getPose().position.x, launch.getPose().heading);
        }

        static TrajectoryActionBuilder linearSplineTrajectory(Robot robot, OBELISK obelisk, LAUNCH launch) {
            return robot.drive.actionBuilder(obelisk.getPose())
                    .setTangent(obelisk.out())
                    .splineToLinearHeading(launch.getPose(), launch.in());
        }
    }

    MecanumDrive.Params parameters = new MecanumDrive.Params();

    public class Robot {
        final Intake intake;
        final Launch launch;
        final Load load;
        final Bunt bunt;
        final MecanumDrive drive;
        final Camera camera;
        int id;

        public Robot(Intake intake, Launch launch, Load load, Bunt bunt, MecanumDrive drive, Camera camera) {
            this.intake = intake;
            this.launch = launch;
            this.load = load;
            this.bunt = bunt;
            this.drive = drive;
            this.camera = camera;
        }

        public Action Init() {
            return new SequentialAction(
                    intake.intakeInit(),
                    launch.launchInit(),
                    load.loadInit(),
                    bunt.buntInit()
            );
        }

        public Action hitBall() {
            return new SequentialAction(
                    bunt.buntLaunch(),
                    new SleepAction(0.75f),
                    bunt.buntReset()
            );
        }

        public Action shootLow(float timeToWait) {
            return new SequentialAction(
                    load.loadLoad(),
                    new SleepAction(timeToWait),
                    hitBall()
            );
        }

        public Action shootHigh(float timeToWait) {
            return new SequentialAction(
                    load.loadReset(),
                    new SleepAction(timeToWait),
                    hitBall()
            );
        }

        public Action shootHighDouble(float timeToWait) {
            return new SequentialAction(
                    load.loadReset(),
                    new SleepAction(timeToWait),
                    intake.intakeOut(),
                    hitBall(),
                    intake.intakeOff()
            );
        }

        public Action loadIntakeIntoHigh(float intakeTime) {
            return new SequentialAction(
                    load.loadReset(),
                    intake.intakeLoad(),
                    new SleepAction(intakeTime),
                    nudgeIntake(),
                    intake.intakeOff()
            );
        }

        public Action loadIntakeIntoLow(float intakeTime) {
            return new SequentialAction(
                    load.loadLoad(),
                    intake.intakeLoad(),
                    new SleepAction(intakeTime),
                    nudgeIntake(),
                    intake.intakeOff()
            );
        }

        public Action lowerIntake(float intakeTime) {
            return new SequentialAction(
                    intake.intakeOut(),
                    new SleepAction(intakeTime),
                    intake.intakeOff()
            );
        }

        public Action raiseIntake(float intakeTime) {
            return new SequentialAction(
                    intake.intakeIn(),
                    new SleepAction(intakeTime),
                    intake.intakeOff()
            );
        }

        public Action loadIntake(float intakeTime) {
            return new SequentialAction(
                    intake.intakeLoad(),
                    new SleepAction(intakeTime),
                    intake.intakeOff()
            );
        }

        public Action nudgeIntake() {
            return new SequentialAction(
                    bunt.buntLoad(),
                    new SleepAction(0.3f),
                    bunt.buntReset()
            );
        }

        public Action shuffleIntakeIntoHigh(float intakeTime) {
            return new SequentialAction(
                    load.loadReset(),
                    intake.intakeOut(),
                    new SleepAction(0.25),
                    intake.intakeLoad(),
                    new SleepAction(intakeTime),
                    intake.intakeOff()
            );
        }

        public Action collectBalls(TrajectoryActionBuilder forwardTrajectory) {
            return new SequentialAction(
                    intake.intakeLoad(),
                    forwardTrajectory.build(),
                    intake.intakeOff()
            );
        }

        public Action shootLLLFar() { // FINISHED
            return new SequentialAction(
                    load.loadReload(),
                    lowerIntake(0.45f),
                    loadIntakeIntoLow(0f),
                    launch.launchFarLow(),
                    shootLow(1.5f),
                    new SleepAction(1),
                    loadIntakeIntoLow(0.25f),
                    shootLow(1),
                    new SleepAction(1),
                    loadIntakeIntoLow(1f),
                    shootLow(1),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootPLLLFar() {
            return new SequentialAction(
                    launch.launchFarLow(),
                    shootLow(1.5f),
                    new SleepAction(0.5f),
                    loadIntakeIntoLow(0.5f),
                    shootLow(0),
                    new SleepAction(0.5f),
                    loadIntakeIntoLow(0.5f),
                    shootLow(0),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootLLLClose() {
            return new SequentialAction(
                    lowerIntake(0.75f),
                    launch.launchCloseLow(),
                    shootLow(2),
                    loadIntakeIntoLow(0.25f),
                    shootLow(1),
                    loadIntakeIntoLow(1f),
                    shootLow(1),
                    launch.launchOff()
            );
        }

        public Action shootPLLLClose() {
            return new SequentialAction(
                    launch.launchCloseLow(),
                    shootLow(1.5f),
                    loadIntakeIntoLow(0.25f),
                    shootLow(1),
                    loadIntakeIntoLow(1f),
                    shootLow(1),
                    launch.launchOff()
            );
        }

        public Action shootHLLFar() {
            return new SequentialAction(
                    lowerIntake(0.1f),
                    launch.launchFarHigh(),
                    shootHigh(1.5f),
                    launch.launchFarLow(),
                    shootLow(1),
                    loadIntakeIntoLow(1),
                    shootLow(1),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootPHLLFar() {
            return new SequentialAction(
                    launch.launchFarHigh(),
                    shootHigh(1.5f),
                    launch.launchFarLow(),
                    shootLow(1),
                    loadIntakeIntoLow(1),
                    shootLow(1),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootHLLClose() {
            return new SequentialAction(
                    launch.launchCloseHigh(),
                    shootHigh(1f),
                    launch.launchCloseLow(),
                    new SleepAction(1),
                    shootLow(1),
                    loadIntakeIntoLow(1),
                    shootLow(1),
                    new SleepAction(0.4),
                    launch.launchOff()
            );
        }

        public Action shootPHLLClose() {
            return new SequentialAction(
                    launch.launchCloseHigh(),
                    shootHigh(1f),
                    launch.launchCloseLow(),
                    new SleepAction(1),
                    shootLow(1),
                    loadIntakeIntoLow(1),
                    shootLow(1),
                    new SleepAction(0.4),
                    launch.launchOff()
            );
        }

        public Action shootLHLFar() {
            return new SequentialAction(
                    launch.launchFarLow(),
                    shootLow(0),
                    launch.launchFarHigh(),
                    loadIntakeIntoHigh(2),
                    shootHigh(1),
                    launch.launchFarLow(),
                    shootLow(1),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootPLHLFar() {
            return new SequentialAction(
                    launch.launchFarLow(),
                    shootLow(0),
                    launch.launchFarHigh(),
                    loadIntakeIntoHigh(2),
                    shootHigh(1),
                    launch.launchFarLow(),
                    shootLow(1),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootLHLClose() {
            return new SequentialAction(
                    launch.launchCloseLow(),
                    shootLow(0),
                    launch.launchCloseHigh(),
                    loadIntakeIntoHigh(2),
                    shootHigh(1),
                    launch.launchCloseLow(),
                    shootLow(1),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootPLHLClose() {
            return new SequentialAction(
                    launch.launchCloseLow(),
                    shootLow(0),
                    launch.launchCloseHigh(),
                    loadIntakeIntoHigh(2),
                    shootHigh(1),
                    launch.launchCloseLow(),
                    shootLow(1),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootHHLFar() {
            return new SequentialAction(
                    launch.launchFarHigh(),
                    shootHigh(0),
                    loadIntakeIntoHigh(1),
                    shootHigh(1),
                    launch.launchFarLow(),
                    loadIntakeIntoLow(1),
                    shootLow(1),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootPHHLFar() {
            return new SequentialAction(
                    launch.launchFarHigh(),
                    shootHigh(0),
                    loadIntakeIntoHigh(1),
                    shootHigh(1),
                    launch.launchFarLow(),
                    loadIntakeIntoLow(1),
                    shootLow(1),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootHHLClose() {
            return new SequentialAction(
                    launch.launchCloseHigh(),
                    shootHigh(0),
                    loadIntakeIntoHigh(1),
                    shootHigh(1),
                    launch.launchCloseLow(),
                    loadIntakeIntoLow(1),
                    shootLow(1),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootPHHLClose() {
            return new SequentialAction(
                    launch.launchCloseHigh(),
                    shootHigh(0),
                    loadIntakeIntoHigh(1),
                    shootHigh(1),
                    launch.launchCloseLow(),
                    loadIntakeIntoLow(1),
                    shootLow(1),
                    new SleepAction(0.4f),
                    launch.launchOff()
            );
        }

        public Action shootBalls(char artifactSet, int tagID, boolean isFar) {
            if (isFar) {
                if (artifactSet == 'A') { //PPG
                    if (tagID == 21) { //GPP
                        return new SequentialAction(
                                shootLLLFar()
                        );
                    } else if (tagID == 22) { //PGP
                        return new SequentialAction(
                                shootLHLFar()
                        );
                    } else if (tagID == 23) { //PPG
                        return new SequentialAction(
                                shootLLLFar()
                        );
                    }
                } else if (artifactSet == 'B') { //PGP
                    if (tagID == 21) { //GPP
                        return new SequentialAction(
                                shootHLLFar()
                        );
                    } else if (tagID == 22) { //PGP
                        return new SequentialAction(
                                shootLLLFar()
                        );
                    } else if (tagID == 23) { //PPG
                       return new SequentialAction(
                                shootLHLFar()
                       );
                    }
                } else if (artifactSet == 'C') { //GPP
                    if (tagID == 21) { //GPP
                        return new SequentialAction(
                                shootLLLFar()
                        );
                    } else if (tagID == 22) { //PGP
                        return new SequentialAction(
                                shootHLLFar()
                        );
                    } else if (tagID == 23) { //PPG
                        return new SequentialAction(
                                shootHHLFar()
                        );
                    }
                }
                return new SequentialAction(
                        shootLLLFar()
                );
            } else {
                if (artifactSet == 'A') { //PPG
                    if (tagID == 21) { //GPP
                        return new SequentialAction(
                                shootLLLClose()
                        );
                    } else if (tagID == 22) { //PGP
                        return new SequentialAction(
                                shootLHLClose()
                        );
                    } else if (tagID == 23) { //PPG
                        return new SequentialAction(
                                shootLLLClose()
                        );
                    }
                } else if (artifactSet == 'B') { //PGP
                    if (tagID == 21) { //GPP
                        return new SequentialAction(
                                shootHLLClose()
                        );
                    } else if (tagID == 22) { //PGP
                        return new SequentialAction(
                                shootLLLClose()
                        );
                    } else if (tagID == 23) { //PPG
                        return new SequentialAction(
                                shootLHLClose()
                        );
                    }
                } else if (artifactSet == 'C') { //GPP
                    if (tagID == 21) { //GPP
                        return new SequentialAction(
                                shootLLLClose()
                        );
                    } else if (tagID == 22) { //PGP
                        return new SequentialAction(
                                shootHLLClose()
                        );
                    } else if (tagID == 23) { //PPG
                        return new SequentialAction(
                                shootHHLClose()
                        );
                    }
                }
                return new SequentialAction(
                        shootHHLClose()
                );
            }
        }

        public Action shootPreload(int tagID, boolean isFar) { // PGP
            if (isFar) {
                if (tagID == 21) { //GPP
                    return new SequentialAction(
                            shootPHLLFar()
                    );
                } else if (tagID == 22) { //PGP
                    return new SequentialAction(
                            shootPLLLFar()
                    );
                } else if (tagID == 23) { //PPG
                    return new SequentialAction(
                            shootPLHLFar()
                    );
                }
                return new SequentialAction(
                        shootPLLLFar()
                );
            } else {
                if (tagID == 21) { //GPP
                    return new SequentialAction(
                            shootPHLLClose()
                    );
                } else if (tagID == 22) { //PGP
                    return new SequentialAction(
                            shootPLLLClose()
                    );
                } else if (tagID == 23) { //PPG
                    return new SequentialAction(
                            shootPLHLClose()
                    );
                }
                return new SequentialAction(
                        shootPHHLClose()
                );
            }
        }
    }

    public class Intake {
        private final DcMotorEx intake;
        private boolean runMove = true;

        int intakeDirection = parameters.INTAKE_DIRECTION_START;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class IntakeMove implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0);
                    initialized = true;
                    runMove = true;
                }

                if (intakeDirection == 1) {
                    intake.setPower(parameters.INTAKE_SPEED_IN);
                } else if (intakeDirection == -1) {
                    intake.setPower(parameters.INTAKE_SPEED_OUT);
                } else if (intakeDirection == 2) {
                    intake.setPower(parameters.INTAKE_SPEED_LOAD);
                } else {
                    intake.setPower(0f);
                }

                return runMove;
            }
        }

        public Action moveIntake() {
            return new IntakeMove();
        }

        public class IntakeStop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                runMove = false;
                return false;
            }
        }

        public Action stopIntake() {
            return new IntakeStop();
        }

        public class IntakeIn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeDirection = 1;
                return false;
            }
        }

        public Action intakeIn() {
            return new IntakeIn();
        }

        public class IntakeOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeDirection = -1;
                return false;
            }
        }

        public Action intakeOut() {
            return new IntakeOut();
        }

        public class IntakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeDirection = 0;
                return false;
            }
        }

        public Action intakeOff() {
            return new IntakeOff();
        }

        public class IntakeLoad implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeDirection = 2;
                return false;
            }
        }

        public Action intakeLoad() {return new IntakeLoad();}

        public class IntakeInit implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public Action intakeInit() {
            return new IntakeInit();
        }
    }

    public class Load {
        private final Servo load;

        public Load(HardwareMap hardwareMap) {
            load = hardwareMap.get(Servo.class, "load");
        }

        public class LoadLoad implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                load.setPosition(parameters.LOAD_LOAD);
                return false;
            }
        }

        public Action loadLoad() {
            return new LoadLoad();
        }

        public class LoadReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                load.setPosition(parameters.LOAD_RESET);
                return false;
            }
        }

        public Action loadReset() {
            return new LoadReset();
        }

        public class LoadInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                load.setPosition(parameters.LOAD_INIT);
                return false;
            }
        }

        public Action loadInit() {
            return new LoadInit();
        }

        public class LoadReload implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                load.setPosition(parameters.LOAD_RELOAD);
                return false;
            }
        }

        public Action loadReload() {
            return new LoadReload();
        }
    }

    public class Bunt {
        private final Servo bunt;

        public Bunt(HardwareMap hardwareMap) {
            bunt = hardwareMap.get(Servo.class, "bunt");
        }

        public class BuntLaunch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bunt.setPosition(parameters.BUNT_LAUNCH);
                return false;
            }
        }

        public Action buntLaunch() {
            return new Bunt.BuntLaunch();
        }

        public class BuntReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bunt.setPosition(parameters.BUNT_RESET);
                return false;
            }
        }

        public Action buntReset() {
            return new Bunt.BuntReset();
        }

        public class BuntInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bunt.setPosition(parameters.BUNT_RESET);
                return false;
            }
        }

        public Action buntInit() {
            return new Bunt.BuntInit();
        }

        public class BuntLoad implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                bunt.setPosition(parameters.BUNT_LOAD);
                return false;
            }
        }

        public Action buntLoad() {
            return new Bunt.BuntLoad();
        }
    }

    public class Launch {
        private final DcMotorEx launch1;
        private final DcMotorEx launch2;
        private boolean runMove = true;

        int launchState = 0;

        public Launch(HardwareMap hardwareMap) {
            launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
            launch1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            launch1.setDirection(DcMotorEx.Direction.FORWARD);
            launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
            launch2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            launch2.setDirection(DcMotorEx.Direction.REVERSE);
        }

        public class LaunchMove implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    runMove = true;
                }

                if (launchState == 0 ) {
                    launch1.setVelocity(0, AngleUnit.RADIANS);
                    launch2.setVelocity(0, AngleUnit.RADIANS);
                } else if (launchState == 1) {
                    launch1.setVelocity(parameters.LAUNCH_SPEED_CLOSE_HIGH, AngleUnit.RADIANS);
                    launch2.setVelocity(parameters.LAUNCH_SPEED_CLOSE_HIGH, AngleUnit.RADIANS);
                } else if (launchState == 2) {
                    launch1.setVelocity(parameters.LAUNCH_SPEED_FAR_HIGH, AngleUnit.RADIANS);
                    launch2.setVelocity(parameters.LAUNCH_SPEED_FAR_HIGH, AngleUnit.RADIANS);
                } else if (launchState == 3) {
                    launch1.setVelocity(parameters.LAUNCH_SPEED_DROP, AngleUnit.RADIANS);
                    launch2.setVelocity(parameters.LAUNCH_SPEED_DROP, AngleUnit.RADIANS);
                } else if (launchState == 4) {
                    launch1.setVelocity(parameters.LAUNCH_SPEED_FAR_LOW, AngleUnit.RADIANS);
                    launch2.setVelocity(parameters.LAUNCH_SPEED_FAR_LOW, AngleUnit.RADIANS);
                } else if (launchState == 5) {
                    launch1.setVelocity(parameters.LAUNCH_SPEED_CLOSE_LOW, AngleUnit.RADIANS);
                    launch2.setVelocity(parameters.LAUNCH_SPEED_CLOSE_LOW, AngleUnit.RADIANS);
                }

                return runMove;
            }
        }

        public Action moveLaunch() {
            return new LaunchMove();
        }

        public class LaunchStop implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                runMove = false;
                return false;
            }
        }

        public Action stopLaunch() {
            return new LaunchStop();
        }

        public class LaunchCloseHigh implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchState = 1;
                return false;
            }
        }

        public Action launchCloseHigh() {
            return new LaunchCloseHigh();
        }

        public class LaunchCloseLow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchState = 5;
                return false;
            }
        }

        public Action launchCloseLow() {
            return new LaunchCloseLow();
        }

        public class LaunchFarHigh implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchState = 2;
                return false;
            }
        }

        public Action launchFarHigh() {
            return new LaunchFarHigh();
        }

        public class LaunchFarLow implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchState = 4;
                return false;
            }
        }

        public Action launchFarLow() {
            return new LaunchFarLow();
        }

        public class LaunchOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchState = 0;
                return false;
            }
        }

        public Action launchOff() {
            return new LaunchOff();
        }

        public class LaunchDrop implements Action {
            @Override
            public boolean run (@NonNull TelemetryPacket packet) {
                launchState = 3;
                return false;
            }
        }

        public Action launchDrop() {
            return new LaunchDrop();
        }

        public class LaunchInit implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return false;
            }
        }

        public Action launchInit() {
            return new LaunchInit();
        }
    }

    public class Camera {
        private final OpenCvCamera camera;
        public final AprilTagDetectionPipeline aprilTagDetectionPipeline;

        public Camera (HardwareMap hardwareMap) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FGLs Webcam 2025!"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(parameters.TAGSIZE_METERS, parameters.WEBCAM_FOCAL_X, parameters.WEBCAM_FOCAL_Y, parameters.WEBCAM_PRINCIPAL_POINT_X, parameters.WEBCAM_PRINCIPAL_POINT_Y);
            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
        }

        public int scanTagInit() {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            if (detections == null) {
                return -1;
            }

            if (!detections.isEmpty()) {
                for (AprilTagDetection detection : detections) {
                    if (detection.id == 21) {
                        return 21;
                    } else if (detection.id == 22) {
                        return 22;
                    } else if (detection.id == 23) {
                        return 23;
                    }
                }
            }
            return -1;
        }
    }

    @Override
    public void runOpMode() {
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

        Positions.START startPos = Positions.START.BLUE_FAR; // choose start
        char artifactLetter = 'C'; // choose which artifact: 'A', 'B', or 'C'

        Pose2d initialPose = startPos.getPose();

        Robot robot = new Robot(
                new Intake(hardwareMap), new Launch(hardwareMap), new Load(hardwareMap), new Bunt(hardwareMap),
                new MecanumDrive(hardwareMap, initialPose), new Camera(hardwareMap));

        boolean isRed = (startPos == Positions.START.RED_CLOSE || startPos == Positions.START.RED_FAR);
        boolean isClose = (startPos == Positions.START.RED_CLOSE || startPos == Positions.START.BLUE_CLOSE);

        // pick correct artifact automatically
        Positions.ARTIFACT detectedTag;
        if (isRed) {
            if (artifactLetter == 'A') detectedTag = Positions.ARTIFACT.RED_A;
            else if (artifactLetter == 'B') detectedTag = Positions.ARTIFACT.RED_B;
            else detectedTag = Positions.ARTIFACT.RED_C;
        } else {
            if (artifactLetter == 'A') detectedTag = Positions.ARTIFACT.BLUE_A;
            else if (artifactLetter == 'B') detectedTag = Positions.ARTIFACT.BLUE_B;
            else detectedTag = Positions.ARTIFACT.BLUE_C;
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
        Pose2d redGoal = Positions.GOAL.RED.getPose();
        Pose2d blueGoal = Positions.GOAL.BLUE.getPose();

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

        double firstTangent = isRed ? Math.toRadians(225) : Math.toRadians(135);
        double approachHeading = isRed ? Math.toRadians(90) : Math.toRadians(270);

        // make a point in front of the goal (stop before entering)
        double stopDistance = 22; // how far to stop before goal
        Pose2d goalFront = new Pose2d(
                teamGoal.position.x + stopDistance + 21,
                teamGoal.position.y + (isRed? -stopDistance : +stopDistance),
                teamGoal.heading.real
        );

        // build the sequence (view obelisk -> artifact -> back out -> goal front -> stop)
        Action fullSequence = robot.drive.actionBuilder(initialPose)
                // 1. go to obelisk
                .setTangent(Math.toRadians(isRed ? (isClose ? -45 : 197) : (isClose ?  45 : 163)))
                .splineToLinearHeading(chosenObeliskPose, isClose ? chosenObeliskPose.heading.real : (isRed ? Math.toRadians(225) : Math.toRadians(135)))
                .waitSeconds(secondsToWait)
                // 2. go to artifact
                .setTangent(Math.toRadians(isRed ? 45 : -45))
                .splineToLinearHeading(approachPoint, approachHeading)
                .lineToY(target.position.y)
                // 3. back out (along Y axis)
                .lineToY(approachPoint.position.y + (isRed ? -10 : 10))
                // 4. move in front of goal (stop before entering)
                .setTangent(Math.toRadians(isRed ? -90 : 90))
                .splineTo(goalFront.position, Math.toRadians(isRed ? 135 : 225))
                .build();

        Action s1 = robot.drive.actionBuilder(initialPose)
                // 1. go to obelisk
                .setTangent(Math.toRadians(isRed ? (isClose ? -45 : 197) : (isClose ?  45 : 163)))
                .splineToLinearHeading(chosenObeliskPose, isClose ? chosenObeliskPose.heading.real : (isRed ? Math.toRadians(225) : Math.toRadians(135))).waitSeconds(secondsToWait).build();

        Action s2 = robot.drive.actionBuilder(chosenObeliskPose).setTangent(Math.toRadians(isRed ? 45 : -45))
                .splineToLinearHeading(approachPoint, approachHeading).build();

        Action s3 = robot.drive.actionBuilder(approachPoint).setTangent(Math.toRadians(90)).lineToY(target.position.y, new TranslationalVelConstraint(10)).build();
        TrajectoryActionBuilder t3 = robot.drive.actionBuilder(approachPoint).setTangent(Math.toRadians(90)).lineToY(target.position.y, new TranslationalVelConstraint(10));


        Action s4 = robot.drive.actionBuilder(new Pose2d(approachPoint.position.x, target.position.y, approachPoint.heading.real)).setTangent(Math.toRadians(270)).lineToY(approachPoint.position.y + (isRed ? -10 : 10)).build();

        Action s5 = robot.drive.actionBuilder(new Pose2d(approachPoint.position.x, approachPoint.position.y + (isRed ? -10 : 10), Math.toRadians(90))).setTangent(Math.toRadians(isRed ? -225 : 225))
                .splineTo(goalFront.position, Math.toRadians(isRed ? 135 : 225)).turnTo(Math.toRadians(240)).build();


        // Initialization Actions
        Actions.runBlocking(robot.Init());

        int scannedTagID = -1;
        while (!isStopRequested() && !opModeIsActive()) {
            int newScanID = robot.camera.scanTagInit();
            if (newScanID == -1) {
                telemetry.addLine("Scanned some other tag");
                telemetry.update();
            } else {
                scannedTagID = newScanID;
                telemetry.addData("ID", scannedTagID);
                telemetry.update();
            }
        }

        Action preScanTrajectory;
        Action postScanTrajectory;

        //a == ppg
        if (scannedTagID == 21) { //GPP
            preScanTrajectory = new SleepAction(1);
            postScanTrajectory = robot.drive.actionBuilder(Positions.START.RED_FAR.getPose())
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(Positions.ARTIFACT.RED_C.getPose(), Math.toRadians(90)).build();
        } else if (scannedTagID == 22) { //PGP
            preScanTrajectory = new SleepAction(1);
        } else if (scannedTagID == 23) { //PPG
            preScanTrajectory = new SleepAction(1);
        } else {
            preScanTrajectory = new SequentialAction(
                    robot.bunt.buntLaunch()
            );
        }

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Run Pathing
        Actions.runBlocking(
                new ParallelAction(
                        robot.intake.moveIntake(),
                        robot.launch.moveLaunch(),
//                        fullSequence
                        new SequentialAction(
//                                s1,
//                                s2,
//                                robot.collectBalls(t3),
//                                s4,
//                                s5,
//                                robot.launch.launchClose(),
//                                new SleepAction(1),
//                                robot.shootHigh(0),
//                                new SleepAction(1),
//                                robot.loadIntakeIntoHigh(1),
//                                new SleepAction(0.75),
//                                robot.shootHigh(0),
//                                robot.load.loadLoad(),
//                                robot.shootLow(1)
//                                robot.camera.scanOrder(10000000),
//                                robot.setOrder(),
                                robot.shootHLLFar()
                        )
                )
        );

        int maxCycles = 50;
        if (scannedTagID == -1) {
            while (maxCycles > 0) {
                maxCycles--;
                if (maxCycles == 0) {
                    telemetry.addData("How interesting", scannedTagID);
                    break;
                }
            }
            //scan tag;
            //if not found
                // --> Try again, cycles--;
            //if found success

        }

//        Actions.runBlocking(
//                robot.load.loadLoad()
//        );

    }
}
