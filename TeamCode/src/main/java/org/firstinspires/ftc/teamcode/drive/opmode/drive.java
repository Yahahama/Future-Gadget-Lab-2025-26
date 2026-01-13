package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.AimAssist;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

@TeleOp(name="Basic Drive", group="Linear OpMode")
public class drive extends LinearOpMode {
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

    private final MecanumDrive.Params parameters = new MecanumDrive.Params();
    private final PIDFCoefficients launchPIDFCoefficients = new PIDFCoefficients(parameters.LAUNCH_kP, parameters.LAUNCH_kI, parameters.LAUNCH_kD, parameters.LAUNCH_kF);

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        // Initialize and configure drive motor variables
        DcMotor leftFrontDrive = initializeMotor("lfMtr", DcMotor.Direction.FORWARD);
        DcMotor leftBackDrive = initializeMotor("lbMtr", DcMotor.Direction.FORWARD);
        DcMotor rightFrontDrive = initializeMotor("rfMtr", DcMotor.Direction.REVERSE);
        DcMotor rightBackDrive = initializeMotor("rbMtr", DcMotor.Direction.REVERSE);

        // Initialize and configure intake motor
        DcMotor intake = initializeMotor("intake", DcMotor.Direction.FORWARD);

        // Initialize and configure launch motor
        DcMotorEx launch1 = initializeLaunchMotor("launch1", DcMotorEx.Direction.REVERSE);
        DcMotorEx launch2 = initializeLaunchMotor("launch2", DcMotorEx.Direction.FORWARD);

        DcMotor kickstand = initializeMotor("pl", DcMotor.Direction.FORWARD);

        // Initialize servos
        Servo load = hardwareMap.get(Servo.class, "load");
        Servo bunt = hardwareMap.get(Servo.class, "bunt");

        // Initialize localizer and robot position variables. Get position constants
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        AimAssist aimAssist = new AimAssist(hardwareMap);

        // Initialize control parameters
        int intakeDirection = parameters.INTAKE_DIRECTION_START;
        double intakeSpeed = 0f;
        boolean isIntakeCentric = true;
        boolean isLoadUp = false;
        boolean previousDown = false;

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
            double heading = Math.toDegrees(myPose.heading.toDouble());

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial_target = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral_target = gamepad1.left_stick_x * 1.1;
            double yaw = -gamepad1.right_stick_x * 1.5;

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

            // Intake
            if (gamepad1.y || gamepad2.y) {
                intakeDirection = 0;
            } else if (gamepad1.a || gamepad2.a) {
                intakeDirection = 1;
                intakeSpeed = parameters.INTAKE_SPEED_IN;
            } else if (gamepad1.b || gamepad2.b) {
                intakeDirection = -1;
                intakeSpeed = parameters.INTAKE_SPEED_OUT;
            } else if (gamepad1.x || gamepad2.x) {
                intakeDirection = 1;
                intakeSpeed = parameters.INTAKE_SPEED_LOAD;
            }

            // Power Intake
            if (intakeDirection == 1) {
                intake.setPower(intakeSpeed);
            } else if (intakeDirection == -1) {
                intake.setPower(intakeSpeed);
            } else {
                intake.setPower(0f);
            }

            // Power Launch
            if (gamepad2.right_trigger > 0.5f) {
                double power = isLoadUp ? parameters.LAUNCH_SPEED_CLOSE : parameters.LAUNCH_SPEED_CLOSE + 0;
                launch1.setVelocity(power);
                launch2.setVelocity(power);
            } else if (gamepad2.left_trigger > 0.5f) {
                launch1.setVelocity(parameters.LAUNCH_SPEED_FAR);
                launch2.setVelocity(parameters.LAUNCH_SPEED_FAR);
            } else if (gamepad2.left_stick_button) {
                launch1.setVelocity(parameters.LAUNCH_SPEED_DROP, AngleUnit.RADIANS);
                launch2.setVelocity(parameters.LAUNCH_SPEED_DROP, AngleUnit.RADIANS);
            } else {
                launch1.setVelocity(0, AngleUnit.RADIANS);
                launch2.setVelocity(0, AngleUnit.RADIANS);
            }

            // Loader Servo
            if (gamepad2.dpad_up) {
                load.setPosition(parameters.LOAD_LOAD);
                isLoadUp = true;
            } else if (gamepad2.dpad_down) {
                load.setPosition(parameters.LOAD_RESET);
                isLoadUp = false;
            } else if (gamepad2.dpad_right) {
                load.setPosition(parameters.LOAD_RELOAD);
                isLoadUp = false;
            } else if (gamepad2.dpad_left) {
                load.setPosition(parameters.LOAD_FALL);
                isLoadUp = false;
            }

            if (gamepad2.left_bumper) {
                bunt.setPosition(parameters.BUNT_RESET);
            } else if (gamepad2.right_bumper) {
                bunt.setPosition(parameters.BUNT_LAUNCH);
            } else if (gamepad2.left_stick_y > 0.5f) {
                bunt.setPosition(parameters.BUNT_LOAD);
            } else if (gamepad2.left_stick_y < -0.5f) {
                bunt.setPosition(parameters.BUNT_RESET);
            }

            if (gamepad1.dpad_left) {
                kickstand.setPower(1.0f);
            } else if (gamepad1.dpad_right) {
                kickstand.setPower(-1.0f);
            } else {
                kickstand.setPower(0f);
            }

            // Scan AprilTags
            aimAssist.detect(heading);

            // Power Wheels
            if (!gamepad1.right_bumper) {
                leftFrontDrive.setPower(-leftFrontPower);
                rightFrontDrive.setPower(-rightFrontPower);
                leftBackDrive.setPower(-leftBackPower);
                rightBackDrive.setPower(-rightBackPower);
            } else {
                double power = aimAssist.calculate(heading);
                leftFrontDrive.setPower(-power);
                rightFrontDrive.setPower(power);
                leftBackDrive.setPower(-power);
                rightBackDrive.setPower(power);
                telemetry.addData("power", power);
            }

            // Change Robot facing-direction
            if (gamepad1.dpad_down && !previousDown) {
                isIntakeCentric = !isIntakeCentric;
            }
            previousDown = gamepad1.dpad_down;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Position", "x: " + myPose.position.x + "y: " + myPose.position.y);
            telemetry.addData("Heading", "Angle: " + heading);
            telemetry.addData("Intake", "Direction: " + intakeDirection);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Launch1 Speed", launch1.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("Launch2 Speed", launch2.getVelocity(AngleUnit.RADIANS));
            telemetry.update();
        }
    }

    private DcMotor initializeMotor(String name, DcMotor.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private DcMotorEx initializeLaunchMotor(String name, DcMotorEx.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launchPIDFCoefficients);
        return motor;
    }
}