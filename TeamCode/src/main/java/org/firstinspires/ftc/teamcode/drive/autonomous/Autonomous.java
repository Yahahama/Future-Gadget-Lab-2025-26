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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AUTONOMOUS", group = "Autonomous")
public class Autonomous extends LinearOpMode {

    MecanumDrive.Params parameters = new MecanumDrive.Params();

    // dashboard-editable pattern
    public static String START_PATTERN = "GPP";

    public static class Positions {

        enum GOAL {
            RED(new Pose2d(-64, 57, Math.toRadians(135))),
            BLUE(new Pose2d(-64, -57, Math.toRadians(225)));

            private Pose2d pose2d;
            GOAL(Pose2d _pose2d) { this.pose2d = _pose2d; }
            public Pose2d getPose() { return pose2d; }
        }

        enum START {
            RED_CLOSE(new Pose2d(-49.3f, 50.3f, Math.toRadians(315))),
            RED_FAR(new Pose2d(62, 23, Math.toRadians(180))),
            BLUE_CLOSE(new Pose2d(-49.3f, -50.3f, Math.toRadians(45))),
            BLUE_FAR(new Pose2d(62, -23, Math.toRadians(180)));

            private Pose2d pose2d;
            START(Pose2d _pose2d) { this.pose2d = _pose2d; }
            public Pose2d getPose() { return pose2d; }
        }

        enum ARTIFACT {
            RED_A(new Pose2d(-11.5f, 46.5f, Math.toRadians(0))),
            RED_B(new Pose2d(12.3f, 46.5f, Math.toRadians(0))),
            RED_C(new Pose2d(36, 46.5f, Math.toRadians(0))),
            BLUE_A(new Pose2d(-11.5f, -46.5f, Math.toRadians(0))),
            BLUE_B(new Pose2d(12.3f, -46.5f, Math.toRadians(0))),
            BLUE_C(new Pose2d(36, -46.5f, Math.toRadians(0)));

            private Pose2d pose2d;
            ARTIFACT(Pose2d _pose2d) { this.pose2d = _pose2d; }
            public Pose2d getPose() { return pose2d; }
        }

        enum PARKING {
            PARK_RED(new Pose2d(38.25f, 32, Math.toRadians(0))),
            PARK_BLUE(new Pose2d(38.25f, -32, Math.toRadians(0)));

            private Pose2d pose2d;
            PARKING(Pose2d _pose2d) { this.pose2d = _pose2d; }
            public Pose2d getPose() { return pose2d; }
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
                    load.loadInit()
            );
        }

        public Action poseToPose(TrajectoryActionBuilder poseToPose) {
            return new SequentialAction(poseToPose.build());
        }
    }

    public class Intake {
        private final DcMotor intake;
        int intakeDirection = parameters.INTAKE_DIRECTION_START;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotor.Direction.FORWARD);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class IntakeMove implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (intakeDirection == 1)
                    intake.setPower(parameters.INTAKE_SPEED_IN);
                else if (intakeDirection == -1)
                    intake.setPower(parameters.INTAKE_SPEED_OUT);
                else
                    intake.setPower(0);
                return false; // changed to false
            }
        }

        public Action moveIntake() { return new IntakeMove(); }

        public class IntakeIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeDirection = 1;
                return false;
            }
        }

        public Action intakeIn() { return new IntakeIn(); }

        public class IntakeOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeDirection = -1;
                return false;
            }
        }

        public Action intakeOut() { return new IntakeOut(); }

        public class IntakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeDirection = 0;
                return false;
            }
        }

        public Action intakeOff() { return new IntakeOff(); }

        public class IntakeInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) { return false; }
        }

        public Action intakeInit() { return new IntakeInit(); }
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

        public Action loadLoad() { return new LoadLoad(); }

        public class LoadReset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                load.setPosition(parameters.LOAD_RESET);
                return false;
            }
        }

        public Action loadReset() { return new LoadReset(); }

        public class LoadInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                load.setPosition(parameters.LOAD_INIT);
                return false;
            }
        }

        public Action loadInit() { return new LoadInit(); }
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
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (launchState == 0) {
                    launch1.setVelocity(0, AngleUnit.RADIANS);
                    launch2.setVelocity(0, AngleUnit.RADIANS);
                } else if (launchState == 1) {
                    launch1.setVelocity(parameters.LAUNCH_SPEED_CLOSE, AngleUnit.RADIANS);
                    launch2.setVelocity(parameters.LAUNCH_SPEED_CLOSE, AngleUnit.RADIANS);
                } else if (launchState == 2) {
                    launch1.setVelocity(parameters.LAUNCH_SPEED_FAR, AngleUnit.RADIANS);
                    launch2.setVelocity(parameters.LAUNCH_SPEED_FAR, AngleUnit.RADIANS);
                }
                return false; // changed to false
            }
        }

        public Action moveLaunch() { return new LaunchMove(); }

        public class LaunchClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchState = 1;
                return false;
            }
        }

        public Action launchClose() { return new LaunchClose(); }

        public class LaunchFar implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchState = 2;
                return false;
            }
        }

        public Action launchFar() { return new LaunchFar(); }

        public class LaunchOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launchState = 0;
                return false;
            }
        }

        public Action launchOff() { return new LaunchOff(); }

        public class LaunchInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) { return false; }
        }

        public Action launchInit() { return new LaunchInit(); }
    }

    Positions.START startPos = Positions.START.RED_FAR;

    String getPattern() { return START_PATTERN; }

    @Override
    public void runOpMode() {
        Pose2d initialPose = startPos.getPose();
        boolean isRed = (startPos == Positions.START.RED_CLOSE || startPos == Positions.START.RED_FAR);
        boolean isClose = (startPos == Positions.START.RED_CLOSE || startPos == Positions.START.BLUE_CLOSE);

        Robot robot = new Robot(
                new Intake(hardwareMap),
                new Launch(hardwareMap),
                new Load(hardwareMap),
                new MecanumDrive(hardwareMap, initialPose)
        );

        Actions.runBlocking(robot.Init());

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Init X", robot.drive.pose.position.x);
            telemetry.addData("Init Y", robot.drive.pose.position.y);
            telemetry.addData("Init Heading", robot.drive.pose.heading.real);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        Positions.GOAL teamGoal = isRed ? Positions.GOAL.RED : Positions.GOAL.BLUE;
        Positions.ARTIFACT target = (isRed)
                ? (isClose ? Positions.ARTIFACT.RED_B : Positions.ARTIFACT.RED_C)
                : (isClose ? Positions.ARTIFACT.BLUE_B : Positions.ARTIFACT.BLUE_C);

        Pose2d targetPose = target.getPose();
        Pose2d approachPoint = new Pose2d(
                targetPose.position.x,
                targetPose.position.y + (isRed ? -20 : 20),
                Math.toRadians(isRed ? 90 : 270)
        );
        Pose2d chosenObeliskPose = (isRed)
                ? (isClose ? new Pose2d(-32, 32, Math.toRadians(225)) : new Pose2d(23, 12, Math.toRadians(187)))
                : (isClose ? new Pose2d(-32, -32, Math.toRadians(135)) : new Pose2d(23, -12, Math.toRadians(173)));

        double stopDistance = 10;
        Pose2d goalFront = new Pose2d(
                teamGoal.getPose().position.x + stopDistance,
                teamGoal.getPose().position.y,
                teamGoal.getPose().heading.real
        );

        double firstTangent = isRed ? Math.toRadians(225) : Math.toRadians(135);
        double approachHeading = isRed ? Math.toRadians(90) : Math.toRadians(270);

        TrajectoryActionBuilder fullSequence = robot.drive.actionBuilder(initialPose)
                .setTangent(firstTangent)
                .splineToLinearHeading(chosenObeliskPose, chosenObeliskPose.heading.real)
                .waitSeconds(1.0)
                .setTangent(Math.toRadians(isRed ? 45 : -45))
                .splineToLinearHeading(approachPoint, approachHeading)
                .lineToY(targetPose.position.y)
                .lineToY(approachPoint.position.y + (isRed ? -10 : 10))
                .setTangent(Math.toRadians(isRed ? -90 : 90))
                .splineTo(goalFront.position, Math.toRadians(isRed ? 135 : 225));

        // FIXED: only run trajectory, not infinite parallel
        Actions.runBlocking(robot.poseToPose(fullSequence));

        PoseStorage.currentPose = robot.drive.pose;

        String pattern = getPattern();
        Positions.ARTIFACT targetArtifactTriplet;
        switch (pattern) {
            case "GPP":
                targetArtifactTriplet = isRed ? Positions.ARTIFACT.RED_C : Positions.ARTIFACT.BLUE_C;
                break;
            case "PGP":
                targetArtifactTriplet = isRed ? Positions.ARTIFACT.RED_B : Positions.ARTIFACT.BLUE_B;
                break;
            case "PPG":
                targetArtifactTriplet = isRed ? Positions.ARTIFACT.RED_A : Positions.ARTIFACT.BLUE_A;
                break;
            default:
                throw new IllegalStateException("Unexpected pattern: " + pattern);
        }

        TrajectoryActionBuilder toArtifactTriplet = robot.drive.actionBuilder(robot.drive.pose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(targetArtifactTriplet.getPose(), Math.toRadians(0));

        Actions.runBlocking(robot.poseToPose(toArtifactTriplet));
    }
}
