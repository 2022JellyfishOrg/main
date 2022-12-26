
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static double startPoseX = 34;
    public static double startPoseY = -61;
    public static double depositX = 33;
    public static double depositY = -14;
    public static double startAngle = Math.toRadians(0);
    public static double depositAngle = Math.toRadians(-45);
    public static double loadAngle = Math.toRadians(0);
    public static double parkAngle = Math.toRadians(0);
    public static double loadX = 57;
    public static double loadY = -11.5;
    public static double parkX = 33;
    public static double parkY = -14;

    public static void main(String[] args) throws InterruptedException {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(58.3191798, 50, 4.6055, 4.6055, 12)
                .setDimensions(14.5,18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.5, -61, 0))
                                .lineToLinearHeading(new Pose2d(36, -20, 0))
                                .lineToLinearHeading(new Pose2d(30.5, -10, Math.toRadians(-45)))
                                .lineToLinearHeading(new Pose2d (56, -11, 0))
                                .build()





                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}