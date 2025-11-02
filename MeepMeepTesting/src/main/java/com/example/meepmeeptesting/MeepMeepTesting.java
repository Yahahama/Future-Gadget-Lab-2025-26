package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

public class MeepMeepTesting {

    public static class Positions {

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

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double secondsToWait = 1;

        RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        // change these to test
        Positions.START startPos = Positions.START.BLUE_FAR; // choose start
        char artifactLetter = 'C'; // choose which artifact: 'A', 'B', or 'C'

        Pose2d initialPose = startPos.getPose();

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
        double stopDistance = 10; // how far to stop before goal
        Pose2d goalFront = new Pose2d(
                teamGoal.position.x + (isRed ? stopDistance : stopDistance),
                teamGoal.position.y,
                teamGoal.heading.real
        );

        // build the sequence (view obelisk -> artifact -> back out -> goal front -> stop)
        Action fullSequence = robot.getDrive().actionBuilder(initialPose)
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

        robot.runAction(fullSequence);

        Image img = null;
        try {
            img = ImageIO.read(new File("/Users/bma30/Documents/FTC/DecodeMeepMeepBackground.png"));
        } catch (IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(robot)
                .start();
    }
}