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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

import java.util.Arrays;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE_AUTONOMOUS", group = "Autonomous")
public class Autonomous extends LinearOpMode {

    MecanumDrive.Params parameters = new MecanumDrive.Params();
    public static class Positions {

        //Note: These are APPROXIMATE POSITIONS

        enum GOAL {
            RED(new Pose2d(-64, 57, Math.toRadians(135))),
            BLUE(new Pose2d(-64, -57, Math.toRadians(225)));

            private Pose2d pose2d;

            GOAL(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }

        enum START {

            //RED_CLOSE means the starting position CLOSE TO THE GOAL

            RED_CLOSE(new Pose2d(-49.3f, 50.3f, Math.toRadians(315))),
            RED_FAR(new Pose2d(62, 16, Math.toRadians(180))), //y pos + 12???
            BLUE_CLOSE(new Pose2d(-49.3f, -50.3f, Math.toRadians(45))),
            BLUE_FAR(new Pose2d(62, -16, Math.toRadians(180))); //y pos - 12???

            private Pose2d pose2d;

            START(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }
        enum ARTIFACT {

            //+-5 for the other artifacts near the middle one

            RED_A(new Pose2d(-11.5f, 46.5f, Math.toRadians(0))),
            RED_B(new Pose2d(12.3f, 46.5f, Math.toRadians(0))),
            RED_C(new Pose2d(36, 46.5f, Math.toRadians(0))),
            BLUE_A(new Pose2d(-11.5f, -46.5f, Math.toRadians(0))),
            BLUE_B(new Pose2d(12.3f, -46.5f, Math.toRadians(0))),
            BLUE_C(new Pose2d(36, -46.5f, Math.toRadians(0)));

            private Pose2d pose2d;

            ARTIFACT(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }

        enum PARKING {

            //+-5 for the other artifacts near the middle one

            PARK_RED(new Pose2d(38.25f, 32, Math.toRadians(0))),
            PARK_BLUE(new Pose2d(38.25f, -32, Math.toRadians(0)));

            private Pose2d pose2d;

            PARKING(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }
    }

    public class Robot {
        Intake intake;
        Launch launch;
        Load load;
        MecanumDrive drive;

        public Robot(Intake intake, Launch launch, Load load, MecanumDrive drive) {
            this.intake = intake;
            this.launch = launch;
            this.load = load;
            this.drive = drive;
        }

        public Action Init() {
            return new SequentialAction(
                    intake.intakeInit(),
                    launch.launchInit(),
                    launch.launchInit(),
                    load.loadInit()
            );
        }

        public Action poseToScoreClose(TrajectoryActionBuilder poseToLaunchPose) {
            return new SequentialAction(
                    new ParallelAction(
                            poseToLaunchPose.build(),
                            launch.launchClose()
                    ),
                    load.loadLoad(),
                    load.loadReset(),
                    launch.launchOff()
            );
        }

        public Action poseToScoreFar(TrajectoryActionBuilder poseToLaunchPose) {
            return new SequentialAction(
                    new ParallelAction(
                            poseToLaunchPose.build(),
                            launch.launchFar()
                    ),
                    load.loadLoad(),
                    load.loadReset(),
                    launch.launchOff()
            );
        }

        public Action poseToBucket(TrajectoryActionBuilder poseToBucket) {
            return new SequentialAction(
                    new ParallelAction(
                            poseToBucket.build() // Movement + manipulators
                    ),
                    new SleepAction(0.5), // Sequential movements
                    new SleepAction(0.1)
            );
        }

        public Action bucketToSample(TrajectoryActionBuilder bucketToSample) {
            return new SequentialAction(
                    new SleepAction(0.1),
                    new ParallelAction(
                            bucketToSample.build()
                    ),
                    GetSample()
            );
        }

        public Action GetSample() {
            return new SequentialAction(
                    new SleepAction(0.5),
                    new SleepAction(0.5),
                    new SleepAction(0.5),
                    new ParallelAction(
                    )
            );
        }

        public Action GetSampleLow() {
            return new SequentialAction(
                    new SleepAction(0.5), // Sequential movements
                    new SleepAction(0.5),
                    new SleepAction(0.5)
            );
        }

        public Action bucketToSubmersible(TrajectoryActionBuilder bucketToSubmersible) {
            return new ParallelAction(
                    bucketToSubmersible.build()
            );
        }

        public Action poseToClip(TrajectoryActionBuilder poseToClip) {
            return new SequentialAction(
                    new ParallelAction(
                            poseToClip.build()
                    ),
                    new SleepAction(0.5)
            );
        }

        public Action clipToSample(TrajectoryActionBuilder clipToSample) {
            return new SequentialAction(
                    new ParallelAction(
                            clipToSample.build()
                    )
            );
        }

        public Action poseToPose(TrajectoryActionBuilder poseToPose) {
            return new SequentialAction(
                    poseToPose.build()
            );
        }
    }

    public class Intake {

        private final DcMotor intake;

        int intakeDirection = parameters.INTAKE_DIRECTION_START;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeDirection = parameters.INTAKE_DIRECTION_START;
        }

        public class IntakeMove implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(0);
                    initialized = true;
                }

                if (intakeDirection == 1) {
                    intake.setPower(parameters.INTAKE_SPEED_IN);
                } else if (intakeDirection == -1) {
                    intake.setPower(parameters.INTAKE_SPEED_OUT);
                } else {
                    intake.setPower(0f);
                }

                return true;
            }
        }

        public Action moveIntake() {
            return new IntakeMove();
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
        private Servo load;

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
    }

    public class Launch {
        private final DcMotorEx launch1;
        private final DcMotorEx launch2;

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
                }

                if (launchState == 0 ) {
                    launch1.setVelocity(0, AngleUnit.RADIANS);
                    launch2.setVelocity(0, AngleUnit.RADIANS);
                } else if (launchState == 1) {
                    launch1.setVelocity(parameters.LAUNCH_SPEED_CLOSE, AngleUnit.RADIANS);
                    launch2.setVelocity(parameters.LAUNCH_SPEED_CLOSE, AngleUnit.RADIANS);
                } else if (launchState == 2) {
                    launch1.setVelocity(parameters.LAUNCH_SPEED_FAR, AngleUnit.RADIANS);
                    launch2.setVelocity(parameters.LAUNCH_SPEED_FAR, AngleUnit.RADIANS);
                }

                return true;
            }
        }

        public Action moveLaunch() {
            return new LaunchMove();
        }

        public class LaunchClose implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchState = 1;
                return false;
            }
        }

        public Action launchClose() {
            return new LaunchClose();
        }

        public class LaunchFar implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchState = 2;
                return false;
            }
        }

        public Action launchFar() {
            return new LaunchFar();
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

    Positions.START startPos = Positions.START.RED_CLOSE;  // startPos will be different in every extension

    @Override
    public void runOpMode() {
        Pose2d initialPose = startPos.getPose();
        Robot robot = new Robot(
                new Intake(hardwareMap), new Launch(hardwareMap), new Load(hardwareMap),
                new MecanumDrive(hardwareMap, initialPose));

        // Trajectories to select from

        TrajectoryActionBuilder redCloseToViewObelisk = robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-32, 32, Math.toRadians(225)), Math.toRadians(0));

        TrajectoryActionBuilder blueCloseToViewObelisk = robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-32, -32, Math.toRadians(135)), Math.toRadians(0));
        // Initialization Actions
        Actions.runBlocking(robot.Init());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("X Position during Init", robot.drive.pose.position.x);
            telemetry.addData("Y Position during Init", robot.drive.pose.position.y);
            telemetry.addData("Heading during Init", robot.drive.pose.heading.real);

            telemetry.update();
        }

        Action actionToExecute;

        switch(startPos) {
            case RED_CLOSE:
                actionToExecute = new SequentialAction(
                    robot.poseToPose(redCloseToViewObelisk)
                );
                break;
            case RED_FAR:
                actionToExecute = new SequentialAction(
                        // robot.SOMEKINDAACTION
                );
                break;
            case BLUE_CLOSE:
                actionToExecute = new SequentialAction(
                        robot.poseToPose(blueCloseToViewObelisk)
                );
                break;
            case BLUE_FAR:
                actionToExecute = new SequentialAction(
                        // robot.SOMEKINDAACTION
                );
                break;
            default:
                actionToExecute = robot.drive.actionBuilder(new Pose2d(0, 0, 0)).build();
                break;
        }

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Run Pathing
        Actions.runBlocking(
                new ParallelAction(
                        robot.intake.moveIntake(),
                        robot.launch.moveLaunch(),
                        actionToExecute
                )
        );
    }
}
