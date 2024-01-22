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
                            drive.trajectorySequenceBuilder(new Pose2d(-38, -61, Math.toRadians(270)))
                                    .lineToConstantHeading(new Vector2d(-36,-55))

                                    .addDisplacementMarker(() -> {

                                    })
                                    .waitSeconds(2)
                                    .lineToConstantHeading(new Vector2d(-29, -55))
                                    .addDisplacementMarker( () -> {

                                    })
                                    .lineToConstantHeading(new Vector2d(-29, -55.01))
                                    .waitSeconds(.25)
                                    .addDisplacementMarker(() -> {


                                    })
                                    .lineToConstantHeading(new Vector2d(-31,-31))
                                    .lineToLinearHeading(new Pose2d(-31,-31.01,Math.toRadians(180)))
                                    .waitSeconds(.5)
                                    .lineToConstantHeading(new Vector2d(-23,-29))
                                    .lineToConstantHeading(new Vector2d(-24,-29))
                                    .waitSeconds(1)
                                    .addDisplacementMarker(() -> {

                                    })

                                    .lineToConstantHeading(new Vector2d(-48,-29))
                                    .lineToLinearHeading(new Pose2d(-47,-25,Math.toRadians(180)))
                                    .lineToConstantHeading(new Vector2d(-59.5,-25))
                                    .addDisplacementMarker(() -> {

                                    })
                                    .lineToConstantHeading(new Vector2d(-55,-25))
                                    .addDisplacementMarker(() -> {

                                    })
                                    .lineToConstantHeading(new Vector2d(-47,-15))

                                    .lineToConstantHeading(new Vector2d(39, -12))
                                    .addDisplacementMarker(() -> {

                                    })
                                    .waitSeconds(.5)
                                    .lineToConstantHeading(new Vector2d(39,-50))
                                    .lineToConstantHeading(new Vector2d(51,-50))

                                    .addDisplacementMarker(() -> {

                                    })
                                    .waitSeconds(.5)
                                    .lineToConstantHeading(new Vector2d(48,-33))
                                    .addDisplacementMarker(() -> {

                                    })
                                    .lineToConstantHeading(new Vector2d(45, -30))
                                    .lineToLinearHeading(new Pose2d(46 ,-15, Math.toRadians(270)))
                                    .lineToConstantHeading(new Vector2d(45, -15))
                                    .lineToConstantHeading(new Vector2d(52,-16))

                                    .build()
                    );

            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }

