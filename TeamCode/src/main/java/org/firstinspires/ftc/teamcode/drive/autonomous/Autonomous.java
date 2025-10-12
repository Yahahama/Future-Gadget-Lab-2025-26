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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

import java.util.Arrays;

@Config
@Disabled
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
        MecanumDrive drive;

        public Robot(Intake intake, Launch launch, MecanumDrive drive) {
            this.intake = intake;
            this.launch = launch;
            this.drive = drive;
        }

        public Action Init() {
            return new SequentialAction(
                    intake.intakeInit(),
                    launch.launchInit()
            );
        }
    }

    public class Intake {

        private final DcMotorEx intake;
        int intakeDirection;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
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

    public class Launch {
        private final DcMotorEx launch1;
        private final DcMotorEx launch2;

        boolean isLaunchActive = false;

        public Launch(HardwareMap hardwareMap) {
            launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
            launch1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            launch1.setDirection(DcMotorSimple.Direction.FORWARD);
            launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
            launch2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            launch2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LaunchMove implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                }

                if (isLaunchActive) {
                    launch1.setPower(parameters.LAUNCH_POWER);
                }

                return true;
            }
        }

        public Action moveLaunch() {
            return new LaunchMove();
        }

        public class LaunchOn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                isLaunchActive = true;
               return false;
            }
        }

        public Action launchOn() {
            return new LaunchOn();
        }

        public class LaunchOff implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                isLaunchActive = false;
                return false;
            }
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
        Positions.START startPos = Positions.START.RED_CLOSE;

        Pose2d initialPose = startPos.getPose();
        Robot robot = new Robot(
                new Intake(hardwareMap), new Launch(hardwareMap),
                new MecanumDrive(hardwareMap, initialPose));

        // Trajectories to select from, CURRENTLY EMPTY AS OF 10/12/25

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
                        // robot.SOMEKINDAACTION
                );
                break;
            case RED_FAR:
                actionToExecute = new SequentialAction(
                        // robot.SOMEKINDAACTION
                );
                break;
            case BLUE_CLOSE:
                actionToExecute = new SequentialAction(
                        // robot.SOMEKINDAACTION
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
