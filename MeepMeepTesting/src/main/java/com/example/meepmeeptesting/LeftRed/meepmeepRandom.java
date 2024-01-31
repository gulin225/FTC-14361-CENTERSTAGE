package com.example.meepmeeptesting.LeftRed;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


    public class meepmeepRandom {
        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(700);

            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.5)

                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(0, -55, Math.toRadians(0)))
                                 //   .lineToLinearHeading(new Pose2d(-43,-33.5, Math.toRadians(180)))
                                    .splineToConstantHeading(new Vector2d(52, -29.5),Math.toRadians(180))
//
//                                    })
                                    .build()
                    );

            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }

