package org.firstinspires.ftc.teamcode.drive.autonomous;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

// *** THIS IS THE IMPORTANT ONE ***
import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.SleepAction;



public abstract class Autonomous extends LinearOpMode {

    // ------------------------- SHARED ENUMS -------------------------
    // ------------------------- POSITIONS (RESTORED) -------------------------
    public static class Positions {

        public enum GOAL {
            RED(new Pose2d(-53, 48, Math.toRadians(135))),
            BLUE(new Pose2d(-53, -48, Math.toRadians(225)));

            public final Pose2d pose;
            GOAL(Pose2d pose) { this.pose = pose; }
            public Pose2d getPose(){ return pose; }
        }

        public enum START {
            RED_CLOSE(new Pose2d(-49.3, 50.3, Math.toRadians(315))),
            RED_FAR(new Pose2d(62, 16, Math.toRadians(180))),
            BLUE_CLOSE(new Pose2d(-49.3, -50.3, Math.toRadians(45))),
            BLUE_FAR(new Pose2d(62, -16, Math.toRadians(180)));

            public final Pose2d pose;
            START(Pose2d pose){ this.pose = pose; }
            public Pose2d getPose(){ return pose; }
        }

        public enum ARTIFACT {
            RED_A(new Pose2d(-11.5, 46.5, 0)),
            RED_B(new Pose2d(12.3, 46.5, 0)),
            RED_C(new Pose2d(36, 46.5, 0)),
            BLUE_A(new Pose2d(-11.5, -46.5, 0)),
            BLUE_B(new Pose2d(12.3, -46.5, 0)),
            BLUE_C(new Pose2d(36, -46.5, 0));

            public final Pose2d pose;
            ARTIFACT(Pose2d pose){ this.pose = pose; }
            public Pose2d getPose(){ return pose; }
        }
    }

    public enum START {
        RED_CLOSE(new Pose2d(-49.3, 50.3, Math.toRadians(315))),
        RED_FAR(new Pose2d(62, 16, Math.toRadians(180))),
        BLUE_CLOSE(new Pose2d(-49.3, -50.3, Math.toRadians(45))),
        BLUE_FAR(new Pose2d(62, -16, Math.toRadians(180)));

        public final Pose2d pose;
        START(Pose2d pose) { this.pose = pose; }
        public Pose2d getPose() {
            return pose;
        }

    }

    public static class Robot {
        public Intake intake;
        public Launch launch;
        public Load load;
        public Bunt bunt;
        public MecanumDrive drive;


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
        public Action shootHigh(double waitTime) {
            return new SequentialAction(
                    shootHigh(1),
                    new SleepAction(waitTime)
            );
        }

        public Action shootLow(double waitTime) {
            return new SequentialAction(
                    shootLow(1),
                    new SleepAction(waitTime)
            );
        }

    }

    // ------------------------- INTAKE -------------------------
    public class Intake {
        private final DcMotorEx intake;
        int intakeDirection = 0;

        public Intake(HardwareMap hw) {
            intake = hw.get(DcMotorEx.class, "intake");
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class IntakeMove implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if (intakeDirection == 1) intake.setPower(1);
                else if (intakeDirection == -1) intake.setPower(-1);
                else if (intakeDirection == 2) intake.setPower(0.25);
                else intake.setPower(0);
                return true;
            }
        }

        public Action moveIntake() { return new IntakeMove(); }

        public Action intakeIn() {
            intakeDirection = 1;
            return p -> false;
        }

        public Action intakeOut() {
            intakeDirection = -1;
            return p -> false;
        }

        public Action intakeOff() {
            intakeDirection = 0;
            return p -> false;
        }

        public Action intakeLoad() {
            intakeDirection = 2;
            return p -> false;
        }

        public Action intakeInit() {
            return p -> false;
        }

    }

    // ------------------------- LOAD -------------------------
    public class Load {
        private final Servo load;

        public Load(HardwareMap hw) {
            load = hw.get(Servo.class, "load");
        }

        public Action loadLoad() { return p->{ load.setPosition(0.5); return false; }; }
        public Action loadReset() {
            return p -> { load.setPosition(0.1); return false; };
        }

        public Action loadInit() { return p->{ load.setPosition(0.1); return false; }; }
    }

    // ------------------------- BUNT -------------------------
    public class Bunt {
        private final Servo bunt;

        public Bunt(HardwareMap hw) {
            bunt = hw.get(Servo.class, "bunt");
        }

        public Action buntLaunch(){ return p->{ bunt.setPosition(1); return false; }; }
        public Action buntReset() {
            return p -> { bunt.setPosition(0); return false; };
        }

        public Action buntInit()  { return p->{ bunt.setPosition(0); return false; }; }
    }

    // ------------------------- LAUNCH -------------------------
    public class Launch {
        private final DcMotorEx a, b;
        int state = 0;

        public Launch(HardwareMap hw) {
            a = hw.get(DcMotorEx.class, "launch1");
            b = hw.get(DcMotorEx.class, "launch2");
            a.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class Move implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                if (state == 0) { a.setPower(0); b.setPower(0); }
                else if (state == 1) { a.setPower(0.8); b.setPower(0.8); }
                else if (state == 2) { a.setPower(1.0); b.setPower(1.0); }
                return true;
            }
        }

        public Action moveLaunch() { return new Move(); }
        public Action launchClose() {
            state = 1;
            return p -> false;
        }

        public Action launchFar() {
            state = 2;
            return p -> false;
        }

        public Action launchOff() {
            state = 0;
            return p -> false;
        }

        public Action launchInit() {
            return p -> false;
        }


    }
}
