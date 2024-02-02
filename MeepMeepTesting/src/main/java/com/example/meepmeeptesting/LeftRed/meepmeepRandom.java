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
                    .setConstraints(45, 30, Math.toRadians(180), Math.toRadians(180), 10.5)

                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(-40.5, -61, Math.toRadians(270)))
                                    .lineToConstantHeading(new Vector2d(-43,-33.5))

                                    .lineToConstantHeading(new Vector2d(-52,-37))
                                    .splineTo(new Vector2d(-39,-9), Math.toRadians(300))
                                    //.lineToLinearHeading(new Pose2d(-39, -9, Math.toRadians(180)))
                                    // .lineToLinearHeading(new Pose2d(-39,-56.5, Math.toRadians(180)))

                                    //  .lineToConstantHeading(new Vector2d(-43,-42))
                                    //  .lineToLinearHeading(new Pose2d(-43,-55, Math.toRadians(180)))
                                    // .waitSeconds(.05)
                                    // .splineToConstantHeading(new Vector2d(42,-55), Math.toRadians(180))

                                    .lineToConstantHeading(new Vector2d(35,-9))

                                    .lineToConstantHeading(new Vector2d(52, -29.4))




                                    .lineToConstantHeading(new Vector2d(30,-9))

                                    .lineToConstantHeading(new Vector2d(-45,-9))
                                    .lineToLinearHeading(new Pose2d(-56,-9, Math.toRadians(230)))
                                    .addDisplacementMarker(20,() -> {

                                    })
                                    .forward(1)

                                    // .splineToConstantHeading(new Vector2d(-59,-43), Math.toRadians(180))
                                    //  .splineToLinearHeading(new Pose2d(-55,-30,Math.toRadians(135)), Math.toRadians(135))
                                    // .waitSeconds(.1)
                                    .turn(Math.toRadians(-100))

                                    //  .turn(Math.toRadians(-35))

                                    .forward(7)
                                    .waitSeconds(.1)
                                    // .forward(bot.distanceSensor.getFrontDistance()-.5)
                                    // .strafeLeft(bot.distanceSensor.getLeftDistance()-24)
                                    .lineToLinearHeading(new Pose2d(-35,-9, Math.toRadians(180)))

                                    .addDisplacementMarker(60, () -> {

                                        // bot.setLidPosition(lidState.close);
                                    })
                                    .addDisplacementMarker(75, () -> {


                                    })
                                    .addDisplacementMarker(80, () -> {



                                    })
//                .addDisplacementMarker(80, () -> {
//                    bot.setLidPosition(lidState.close);
//                })



                                    .lineToConstantHeading(new Vector2d(30,-9))
                                    .lineToConstantHeading(new Vector2d(52.35,-34))
                                    //     .splineToConstantHeading(new Vector2d(51.9,-34), Math.toRadians(180))
                                    .addDisplacementMarker(120,() ->{


                                    })

                                    .addDisplacementMarker( () -> {


                                    })
                                    .lineToConstantHeading(new Vector2d(50,-34))
                                    .addDisplacementMarker( () -> {


                                    })
                                    .lineToLinearHeading(new Pose2d(46,-11, Math.toRadians(90)))



                                    .build()
//                                    })

                    );

            meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(1F)
                    .addEntity(myBot)
                    .start();
        }
    }

