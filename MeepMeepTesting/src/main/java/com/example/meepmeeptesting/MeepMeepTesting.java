
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) throws InterruptedException {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(14,17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(34.5, -61, 0))
                                .lineTo(new Vector2d(34, -12))
                                .waitSeconds(3)
                                .forward(20)
                                .waitSeconds(2)
                                .back(20)
                                .waitSeconds(3)
                                .forward(20)
                                .waitSeconds(2)
                                .back(20)
                                .waitSeconds(2)
                                .forward(20)
                                .waitSeconds(2)
                                .back(20)
                                .strafeRight(24)
                                .back(24)
                                .build()


                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}