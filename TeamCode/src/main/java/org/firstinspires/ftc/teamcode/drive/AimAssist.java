package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.autonomous.Autonomous;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagPose;
//24 is red
public class AimAssist {
    private final static float RED_BEARING = -4.76f;
    private final static float BLUE_BEARING = 1f;

    private final MecanumDrive DRIVE;

    public AimAssist(MecanumDrive _DRIVE) {
        this.DRIVE = _DRIVE;
    }

    public Action aim(AprilTagDetection detection) {
        boolean isRed = detection.id == 24;
        boolean isBlue = detection.id == 20;

        double bearing = Math.toDegrees(Math.atan2(detection.pose.x, detection.pose.z));

        double difference = 0;
        if (isRed) {
            difference = RED_BEARING - bearing;
        } else if (isBlue) {
            difference = BLUE_BEARING - bearing;
        }

//        return DRIVE.actionBuilder(DRIVE.pose).splineToLinearHeading(new Pose2d(0, 0, difference), 0).build();
        return DRIVE.actionBuilder(DRIVE.pose).setTangent(0).turn(Math.toRadians(difference)).build();
    }
}
