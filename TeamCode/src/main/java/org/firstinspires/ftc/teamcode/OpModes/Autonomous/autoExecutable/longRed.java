package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;

@Autonomous(name = "longRedObjectDetect", group = "Auto")
public class longRed extends LinearOpMode {
    OpenCvCamera camera;
    HSVRedDetection redDetection;
    String webcamName;
    Robot bot;

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-31, -61, Math.toRadians(270));
        //idk if this will cause issues!!!
        double angleOffset = 2;
        //i dont know if these two lines below need to be here ESP initCAM
        //initCam(); try adding this in if doesnt work
        //line 32 may have actually been the issue pls
        drive.setPoseEstimate(startPose);
        //left ------------------------------------------------------------------
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)


                .lineToConstantHeading(new Vector2d(-37,-52))

                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.active);
                    bot.setActiveIntakeState(activeIntakeState.active);
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })
                .waitSeconds(3)
                .lineToConstantHeading(new Vector2d(-37, -51))
                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setActiveIntakeState(activeIntakeState.inactive);
                })
                .lineToConstantHeading(new Vector2d(-37, -50))
                .waitSeconds(.25)
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);

                })

                .lineToConstantHeading(new Vector2d(-37, -44))

                .addDisplacementMarker(() -> {
                    bot.setClawPosition(clawState.leftClose);
                    bot.setClawState(clawState.leftClose);
                })
                .waitSeconds(1)

                .lineToConstantHeading(new Vector2d(-34,-50))


                .lineToLinearHeading(new Pose2d(-34,-55,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(45,-55))
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(575);
                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(44, -23))
                .lineToConstantHeading(new Vector2d(61, -23))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    bot.setClawPosition(clawState.rightClose);
                    bot.setClawState(clawState.rightClose);
                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(50,-22))

                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })

                .lineToLinearHeading(new Pose2d(50 ,-15, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(55,-15))






//                .lineToConstantHeading(new Vector2d(-34,-47))
//                .lineToConstantHeading(new Vector2d(-26,-47))
//                .lineToConstantHeading(new Vector2d(-26,-15))
//                .lineToLinearHeading(new Pose2d(-26,-10,Math.toRadians(180)))
//                .lineToConstantHeading(new Vector2d(44, -10))
//
//                .addDisplacementMarker( () -> {
//                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
//                    bot.setArmState(armState.outtaking);
//
//                    bot.outtakeSlide.setPosition(575);
//                })
//
//                .lineToConstantHeading(new Vector2d(44, -23))
//                .lineToConstantHeading(new Vector2d(61, -23))
//                .waitSeconds(.5)
//                .addDisplacementMarker(() -> {
//                    bot.setClawPosition(clawState.rightClose);
//                    bot.setClawState(clawState.rightClose);
//                })
//                .lineToConstantHeading(new Vector2d(50,-22))
//
//                .addDisplacementMarker(() -> {
//                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
//                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
//                    bot.setWristPosition(wristState.intaking);
//                })
//
//                // j backup and park
//                .lineToLinearHeading(new Pose2d(55,-19,Math.toRadians(90)))
//
//                //if we wanna park close to wall
//                //    .lineToLinearHeading(new Pose2d(54 ,-54, Math.toRadians(450)))

                .build();


        //center ------------------------------------------------------------------

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)

                .lineToConstantHeading(new Vector2d(-26,-52))

                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.active);
                    bot.setActiveIntakeState(activeIntakeState.active);
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })
                .waitSeconds(3)
                .lineToConstantHeading(new Vector2d(-26, -54))
                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setActiveIntakeState(activeIntakeState.inactive);
                })
                .lineToConstantHeading(new Vector2d(-26,-55.01))
                .waitSeconds(.5)
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);

                })


                .lineToConstantHeading(new Vector2d(-26, -36))
                .addDisplacementMarker(() -> {
                    bot.setClawPosition(clawState.leftClose);
                    bot.setClawState(clawState.leftClose);

                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-26  ,-45))

                .lineToLinearHeading(new Pose2d(-34,-57,Math.toRadians(180)))

                .lineToConstantHeading(new Vector2d(45,-57))
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(575);
                })
                .waitSeconds(.25)

                .lineToConstantHeading(new Vector2d(44, -29))
                .lineToConstantHeading(new Vector2d(61,-29))
                .addDisplacementMarker( () -> {
                    bot.setClawPosition(clawState.rightClose);
                    bot.setClawState(clawState.rightClose);
                })

                .lineToConstantHeading(new Vector2d(50,-31))

                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToLinearHeading(new Pose2d(50 ,-15, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(55,-15))



/*
                .lineToConstantHeading(new Vector2d(-24,-40))
                .lineToConstantHeading(new Vector2d(-39,-40))
                .lineToConstantHeading(new Vector2d(-39,-15))
                .lineToLinearHeading(new Pose2d(-34,-10,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(44, -10))

                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(550);
                })

                .waitSeconds(.25)

                .lineToConstantHeading(new Vector2d(44, -29))
                .lineToConstantHeading(new Vector2d(61,-29))
                .addDisplacementMarker( () -> {
                    bot.setClawPosition(clawState.rightClose);
                    bot.setClawState(clawState.rightClose);
                })

                .lineToConstantHeading(new Vector2d(50,-31))

                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToLinearHeading(new Pose2d(54,-20,Math.toRadians(90)))

 */
                .build();


        //right ------------------------------------------------------------------
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)

                .lineToConstantHeading(new Vector2d(-36,-55))

                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.active);
                    bot.setActiveIntakeState(activeIntakeState.active);
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })
                .waitSeconds(3)
                .lineToConstantHeading(new Vector2d(-29, -55))
                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setActiveIntakeState(activeIntakeState.inactive);
                })
                .lineToConstantHeading(new Vector2d(-29, -55.01))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);

                })
                .lineToConstantHeading(new Vector2d(-31,-31))
                .lineToLinearHeading(new Pose2d(-31,-31.01,Math.toRadians(180)))
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(-23,-29))
                .lineToConstantHeading(new Vector2d(-24,-29))
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    bot.setClawPosition(clawState.leftClose);
                    bot.setClawState(clawState.leftClose);
                })

                .lineToConstantHeading(new Vector2d(-34,-57))



                .lineToConstantHeading(new Vector2d(45,-57))
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(575);
                })


                .lineToConstantHeading(new Vector2d(62, -35.5))
                .addDisplacementMarker(() -> {
                    bot.setClawPosition(clawState.rightClose);
                    bot.setClawState(clawState.rightClose);
                })
                .lineToConstantHeading(new Vector2d(50,-38))

                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(55, -33))
                .lineToLinearHeading(new Pose2d(50 ,-15, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(55,-15))

                .build();



        //camera initialization -----------------------------------------------------
        initCam();
        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;

        switch (redDetection.getLocation())
        {
            case LEFT:
                drive.setPoseEstimate(startPose);
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setOuttakeSlideState(outtakeSlidesState.STATION);
                bot.setArmState(armState.intaking);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);
                bot.setClawPosition(clawState.open);
                bot.setClawState(clawState.open);
                drive.followTrajectorySequence(left);

                break;
            case RIGHT:
                drive.setPoseEstimate(startPose);
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setOuttakeSlideState(outtakeSlidesState.STATION);
                bot.setArmState(armState.intaking);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);
                bot.setClawPosition(clawState.open);
                bot.setClawState(clawState.open);
                drive.followTrajectorySequence(right);

                break;
            case MIDDLE:
                drive.setPoseEstimate(startPose);
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setOuttakeSlideState(outtakeSlidesState.STATION);
                bot.setArmState(armState.intaking);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);
                bot.setClawPosition(clawState.open);
                bot.setClawState(clawState.open);
                drive.followTrajectorySequence(center);

                break;
        }
    }

    private void initCam() {

        //This line retrieves the resource identifier for the camera monitor view. The camera monitor view is typically used to display the camera feed
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = "Webcam 1";

        // This line creates a webcam instance using the OpenCvCameraFactor with the webcam name (webcamName) and the camera monitor view ID.
        // The camera instance is stored in the camera variable that we can use later
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        // initializing our Detection class (details on how it works at the top)
        redDetection = new HSVRedDetection(telemetry);

        // yeah what this does is it gets the thing which uses the thing so we can get the thing
        /*
        (fr tho idk what pipeline does, but from what I gathered,
         we basically passthrough our detection into the camera
         and we feed the streaming camera frames into our Detection algorithm)
         */
        camera.setPipeline(redDetection);

        /*
        this starts the camera streaming, with 2 possible combinations
        it starts streaming at a chosen res, or if something goes wrong it throws an error
         */
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.showFpsMeterOnViewport(true);
                camera.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Unspecified Error Occurred; Camera Opening");
            }
        });
    }
}

