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

import java.util.Arrays;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTONOMOUS", group = "Autonomous")
public class Autonomous extends LinearOpMode {
    public static class Positions {
        //TODO: CREATE NEW POSITIONAL CONSTANTS
        enum GOAL {
            RED(new Pose2d(-53, 48, Math.toRadians(135))),
            BLUE(new Pose2d(-53, -48, Math.toRadians(225)));

            private Pose2d pose2d;

            GOAL(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }

        enum START {
            RED_CLOSE(new Pose2d(-48, 47, Math.toRadians(315))),
            RED_FAR(new Pose2d(62, 23, Math.toRadians(180))),
            BLUE_CLOSE(new Pose2d(-48, -47, Math.toRadians(45))),
            BLUE_FAR(new Pose2d(62, -23, Math.toRadians(180)));

            private Pose2d pose2d;

            START(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }

        enum ARTIFACT {
            RED_A(new Pose2d(-11.5f, 46.5f, Math.toRadians(0))),
            RED_B(new Pose2d(12.3f, 46.5f, Math.toRadians(0))),
            RED_C(new Pose2d(35.75, 46.5f, Math.toRadians(0))),
            BLUE_A(new Pose2d(-11.5f, -46.5f, Math.toRadians(0))),
            BLUE_B(new Pose2d(12.3f, -46.5f, Math.toRadians(0))),
            BLUE_C(new Pose2d(35.75, -46.5f, Math.toRadians(0)));

            private Pose2d pose2d;

            ARTIFACT(Pose2d _pose2d) {
                this.pose2d = _pose2d;
            }

            public Pose2d getPose() {
                return pose2d;
            }
        }

        enum PARKING {
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

    MecanumDrive.Params parameters = new MecanumDrive.Params();

    enum StartingPosition {
        BLUE_BUCKET(new Pose2d(35, -62, 0)),
        BLUE_DIVE(new Pose2d(-12, -62, Math.toRadians(180))),
        RED_BUCKET(new Pose2d(-35, 62, Math.toRadians(180))),
        RED_DIVE(new Pose2d(35, 62, 0));

        final Pose2d startPos;

        public Pose2d getStartPos() {
            return startPos;
        }

        StartingPosition(Pose2d startPos) {
            this.startPos = startPos;
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

    public class Robot {
        Intake intake;
        Launch launch;
        Load load;
        Bunt bunt;
        MecanumDrive drive;

        public Robot(Intake intake, Launch launch, Load load, Bunt bunt, MecanumDrive drive) {
            this.intake = intake;
            this.launch = launch;
            this.load = load;
            this.bunt = bunt;
            this.drive = drive;
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
                    new SleepAction(1),
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

        public Action loadIntakeIntoHigh(float intakeTime) {
            return new SequentialAction(
                    load.loadReset(),
                    intake.intakeLoad(),
                    new SleepAction(intakeTime),
                    intake.intakeOff()
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

        public Action collectBalls(TrajectoryActionBuilder ballCollectionTrajectory) {
            return new SequentialAction(
                    intake.intakeLoad(),
                    ballCollectionTrajectory.build(),
                    intake.intakeOff()
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

        public Action clipInchFoward(TrajectoryActionBuilder clipInchFoward){
            return new SequentialAction(
                    new ParallelAction(
                            clipInchFoward.build()
                    )
            );
        }

        public Action sampleToClip(TrajectoryActionBuilder sampleToClip) {
            return new SequentialAction(
                    new ParallelAction(
                            sampleToClip.build()
                    )
            );
        }

        public Action clipToHang(TrajectoryActionBuilder clipToHang) {
            return new SequentialAction(
                    new ParallelAction(
                            clipToHang.build()
                    ),
                    new SleepAction(0.75)
            );
        }

        public Action hangToSample(TrajectoryActionBuilder hangToSample) {
            return new SequentialAction(
                    new ParallelAction(
                            hangToSample.build()
                    ),
                    GetSample()
            );
        }
    }

    public class Intake {
        private final DcMotorEx intake;

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

    public class Bunt {
        private Servo bunt;

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
                new MecanumDrive(hardwareMap, initialPose));

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

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("X Position during Init", robot.drive.pose.position.x);
            telemetry.addData("Y Position during Init", robot.drive.pose.position.y);
            telemetry.addData("Heading during Init", robot.drive.pose.heading.real);

            telemetry.update();
        }

        Action actionToExecute;

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
                                s1,
                                s2,
                                robot.collectBalls(t3),
                                s4,
                                s5,
                                robot.launch.launchClose(),
                                new SleepAction(1),
                                robot.shootHigh(0),
                                new SleepAction(1),
                                robot.loadIntakeIntoHigh(1),
                                new SleepAction(0.75),
                                robot.shootHigh(0),
                                robot.load.loadLoad(),
                                robot.shootLow(1)
                        )
                )
        );
    }
}
