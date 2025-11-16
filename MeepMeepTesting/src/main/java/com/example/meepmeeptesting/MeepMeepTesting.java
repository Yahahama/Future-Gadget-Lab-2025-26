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

        Positions.START startPos = Positions.START.RED_FAR;
        char artifactLetter = 'C';

        Pose2d initialPose = startPos.getPose();

        boolean isRed = (startPos == Positions.START.RED_CLOSE || startPos == Positions.START.RED_FAR);
        boolean isClose = (startPos == Positions.START.RED_CLOSE || startPos == Positions.START.BLUE_CLOSE);

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

        Pose2d redApproachPoint = new Pose2d(target.position.x, target.position.y - 20, Math.toRadians(90));
        Pose2d blueApproachPoint = new Pose2d(target.position.x, target.position.y + 20, Math.toRadians(270));

        Pose2d redCloseObeliskPose = new Pose2d(-32, 32, Math.toRadians(225));
        Pose2d redFarObeliskPose = new Pose2d(23, 12, Math.toRadians(187));
        Pose2d blueCloseObeliskPose = new Pose2d(-32, -32, Math.toRadians(135));
        Pose2d blueFarObeliskPose = new Pose2d(23, -12, Math.toRadians(173));

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

        // -------------------------
        // FINAL PARKING SELECTION
        // -------------------------
        Pose2d finalPark;

        if(!isRed && isClose) {
            finalPark = new Pose2d(-12, -12, Math.toRadians(225));    // BLUE CLOSE
        }else if(!isRed && !isClose) {
            finalPark = new Pose2d(60, -12, Math.toRadians(202));     // BLUE FAR
        }else if(isRed && isClose) {
            finalPark = new Pose2d(-12, 12, Math.toRadians(135));      // RED CLOSE
        }else{
            finalPark = new Pose2d(60, 12, Math.toRadians(158));     // RED FAR
        }


        // -------------------------
        // FULL TRAJECTORY
        // -------------------------
        Action fullSequence = robot.getDrive().actionBuilder(initialPose)
                .setTangent(Math.toRadians(isRed ? (isClose ? -45 : 197) : (isClose ? 45 : 163)))
                .splineToLinearHeading(chosenObeliskPose,
                        isClose ? chosenObeliskPose.heading.real :
                                (isRed ? Math.toRadians(225) : Math.toRadians(135)))
                .waitSeconds(secondsToWait)

                .setTangent(Math.toRadians(isRed ? 45 : -45))
                .splineToLinearHeading(approachPoint, (isRed ? Math.toRadians(90) : Math.toRadians(270)))
                .lineToY(target.position.y)

                .lineToY(approachPoint.position.y + (isRed ? -10 : 10))

                .splineToLinearHeading(finalPark, finalPark.heading.real)

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
