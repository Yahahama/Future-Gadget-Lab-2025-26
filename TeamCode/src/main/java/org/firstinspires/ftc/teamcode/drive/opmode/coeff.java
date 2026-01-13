package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

@Disabled
@TeleOp(name="coeff", group="Linear OpMode")
public class coeff extends LinearOpMode {
    /*
     * Driving (1):
     *    FLIP                - Right Stick
     * Intake (BOTH):
     *    COLLECT             - A
     *    OUT                 - B
     *    FAST                - X
     *    STOP                - Y
     * Launcher (2):
     *    FAST                - Left Trigger
     *    SLOW                - Right Trigger
     *    BACK                - Left Stick
     * Load (2):
     *    RAISE               - Dpad Up
     *    LOWER               - Dpad Down
     *    RECYCLE             - Dpad Left
     * Bunt (2):
     *    RESET               - Left Trigger
     *    LAUNCH              - Right Trigger
     * Macros (1):
     *    NONE
     * Macros (2):
     *    NONE
     */

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Initialize and configure drive motor variables
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "lfMtr"); // CH 2
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "lbMtr"); // CH 3
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rfMtr"); // CH 0
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rbMtr"); // CH 1
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize and configure intake motor
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize and configure launch motor
        DcMotorEx launch1 = hardwareMap.get(DcMotorEx.class, "launch1");
        DcMotorEx launch2 = hardwareMap.get(DcMotorEx.class, "launch2");
        launch1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launch2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launch1.setDirection(DcMotorEx.Direction.REVERSE);
        launch2.setDirection(DcMotorEx.Direction.FORWARD);
        launch1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        launch2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients c1 = launch1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients c2 = launch2.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        float kP = 12;
        float kI = 2;
        float kD = 10;
        float kF = 12;

        boolean previousL = false;
        boolean previousU = false;
        boolean previousR = false;
        boolean previousD = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousB = false;
        boolean previousA = false;

        launch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        launch2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

        DcMotor kickstand = hardwareMap.get(DcMotor.class, "pl");
        kickstand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kickstand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize servos
        Servo load = hardwareMap.get(Servo.class, "load");
        Servo bunt = hardwareMap.get(Servo.class, "bunt");

        // Initialize localizer and robot position variables. Get position constants
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        MecanumDrive.Params parameters = new MecanumDrive.Params();

        // Initialize control parameters
        boolean isIntakeCentric = true;
        boolean previousDown = false;

        float[] change = {0.1f, 1f, 10f};
        int index = 0;

        // Robot is ready to start! Display message to screen
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.setMsTransmissionInterval(50);

        //Initialization Actions
        load.setPosition(parameters.LOAD_INIT);
        bunt.setPosition(parameters.BUNT_RESET);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get Pose
            drive.updatePoseEstimate();
            Pose2d myPose = drive.pose;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial_target = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral_target = gamepad1.left_stick_x * 1.1;
            double yaw = -gamepad1.right_stick_x;

            double theta = isIntakeCentric ? Math.toRadians(180) : Math.toRadians(0); // Change this to use robot-centric soon
            double cosine = Math.cos(theta);
            double sine = Math.sin(theta);

            double lateral_real = lateral_target * cosine - axial_target * sine;
            double axial_real = lateral_target * sine + axial_target * cosine;

            double right_trigger = 1 + gamepad1.right_trigger;
            double left_trigger = 1 - gamepad1.left_trigger;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = ((axial_real + lateral_real + yaw) / 2) * right_trigger * left_trigger;
            double rightFrontPower = ((axial_real - lateral_real - yaw) / 2) * right_trigger * left_trigger;
            double leftBackPower = ((axial_real - lateral_real + yaw) / 2) * right_trigger * left_trigger;
            double rightBackPower = ((axial_real + lateral_real - yaw) / 2) * right_trigger * left_trigger;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            launch1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
            launch2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));

            if (gamepad1.dpad_left && !previousL) {
                kP += change[index];
            }
            if (gamepad1.dpad_up && !previousU) {
                kI += change[index];
            }
            if (gamepad1.dpad_right && !previousR) {
                kD += change[index];
            }
            if (gamepad1.dpad_down && !previousD) {
                kF += change[index];
            }
            if (gamepad1.x && !previousX) {
                kP -= change[index];
            }
            if (gamepad1.y && !previousY) {
                kI -= change[index];
            }
            if (gamepad1.b && !previousB) {
                kD -= change[index];
            }
            if (gamepad1.a && !previousA) {
                kF -= change[index];
            }
            double speed;
            if (gamepad1.right_trigger > 0.5f) {
                speed = parameters.LAUNCH_SPEED_CLOSE;
            } else if (gamepad1.left_trigger > 0.5) {
                speed = parameters.LAUNCH_SPEED_FAR;
            } else {
                speed = 0;
            }
            if (gamepad1.right_bumper) {
                index += 1;
                index %= 3;
            }

            launch1.setVelocity(speed);
            launch2.setVelocity(speed);

            previousL = gamepad1.dpad_left;
            previousU = gamepad1.dpad_up;
            previousR = gamepad1.dpad_right;
            previousD = gamepad1.dpad_down;
            previousX = gamepad1.x;
            previousY = gamepad1.y;
            previousB = gamepad1.b;
            previousA = gamepad1.a;

            // Power Wheels
                leftFrontDrive.setPower(-leftFrontPower);
                rightFrontDrive.setPower(-rightFrontPower);
                leftBackDrive.setPower(-leftBackPower);
                rightBackDrive.setPower(-rightBackPower);

            if (gamepad1.dpad_down && !previousDown) {
                isIntakeCentric = !isIntakeCentric;
            }
            previousDown = gamepad1.dpad_down;

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target", speed);
            packet.put("Motor 1 Speed", launch1.getVelocity());
            packet.put("Motor 2 Speed", launch2.getVelocity());
            dashboard.sendTelemetryPacket(packet);
            float target = 200f / 2f / 3.14159f * 28f;
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Target Speed", target);
            telemetry.addData("Error", ((launch1.getVelocity() - target) + launch2.getVelocity() - target) / 2);
            telemetry.addData("Launch1 Speed", launch1.getVelocity());
            telemetry.addData("Launch2 Speed", launch2.getVelocity());
            telemetry.addData("Real PIDF1", "%4.2f, %4.2f, %4.2f, %4.2f", kP, kI, kD, kF);
            telemetry.addData("Real PIDF2", "%4.2f, %4.2f, %4.2f, %4.2f", kP, kI, kD, kF);
            telemetry.addData("PIDF1", "%4.2f, %4.2f, %4.2f, %4.2f", c1.p, c1.i, c1.d, c1.f);
            telemetry.addData("PIDF2", "%4.2f, %4.2f, %4.2f, %4.2f", c2.p, c2.i, c2.d, c2.f);
            telemetry.addData("Change", change[index]);
            telemetry.update();
        }
    }
}