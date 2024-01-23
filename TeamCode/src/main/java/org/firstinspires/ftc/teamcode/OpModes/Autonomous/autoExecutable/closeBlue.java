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

@Autonomous(name = "closeBlueObjectDetect", group = "Auto")
public class closeBlue extends LinearOpMode {
    OpenCvCamera camera;
    HSVBlueDetection blueDetection;
    String webcamName;
    Robot bot;
    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15, 61, Math.toRadians(90));
        //bot.setInBrake();
        //i dont know if these two lines below need to be here ESP initCAM
        initCam();
        //line 32 may have actually been the issue pls
        drive.setPoseEstimate(startPose);
        //left ------------------------------------------------------------------
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)

                .lineToConstantHeading(new Vector2d(22.25,55))
                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidPosition(lidState.close);
                })

//
//                .lineToConstantHeading(new Vector2d(20, 48))
//                .waitSeconds(.25)
//                .addDisplacementMarker( () -> {
//                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
//                    bot.setArmState(armState.autoDrop);
//
//                })
                .lineToConstantHeading(new Vector2d(22.25,43.75))

                .lineToConstantHeading(new Vector2d(22.25,50))
                .addDisplacementMarker( () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .lineToLinearHeading(new Pose2d(48 ,42.5, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(52,42.5))
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })
                .lineToConstantHeading(new Vector2d(51.8,42.5))
                .addDisplacementMarker( () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })

                .lineToConstantHeading(new Vector2d(43, 42.5))
                .addDisplacementMarker( () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(43, 47))
                .lineToLinearHeading(new Pose2d(49 ,58, Math.toRadians(270)))

                .build();
        //waitForStart();
        //if(isStopRequested()) return;
        //     bot.intakeSlide.setPosition(50);
        //drive.followTrajectorySequence(left);
        //}
        //center ------------------------------------------------------------------
        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)


                .lineToConstantHeading(new Vector2d(10,55))
                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidPosition(lidState.close);
                })

                .lineToConstantHeading(new Vector2d(10, 45))

                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(10, 33.5))

                .lineToConstantHeading(new Vector2d(10, 38))
                .lineToLinearHeading(new Pose2d(40 ,38, Math.toRadians(180)))
                .addDisplacementMarker( () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                  bot.setWristPosition(wristState.outtaking);
                })

                .lineToConstantHeading(new Vector2d(52,36))
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                })
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(51.8, 36))
                .addDisplacementMarker( () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(43, 36))
                .addDisplacementMarker( () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(43, 47))
                .lineToLinearHeading(new Pose2d(49 ,58, Math.toRadians(270)))

                .build();

        //right ------------------------------------------------------------------
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(15, 54))

                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidPosition(lidState.close);
                })
                .lineToLinearHeading(new Pose2d(15,32, Math.toRadians(0)))

//                .lineToConstantHeading(new Vector2d(4, 32))
//                .lineToConstantHeading(new Vector2d(8, 32))
//                .addDisplacementMarker( () -> {
//
//                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
//                    bot.setArmState(armState.autoDrop);
//
//                })
                .lineToConstantHeading(new Vector2d(10,32))

//                .lineToLinearHeading(new Pose2d(20,30, Math.toRadians(90)))
                //    .lineToConstantHeading(new Vector2d(30,36))
                .lineToConstantHeading(new Vector2d(15,32))
                .lineToLinearHeading(new Pose2d(30,30,Math.toRadians(180)))
                .addDisplacementMarker( () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                //    .lineToConstantHeading(new Vector2d(21, 32))
                .lineToConstantHeading(new Vector2d(53,31))
                .addDisplacementMarker(() -> {
                   bot.setLidPosition(lidState.open);
                })
                .lineToConstantHeading(new Vector2d(52.8,31))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })

                .lineToConstantHeading(new Vector2d(43,31))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(47, 32))
                .lineToLinearHeading(new Pose2d(47 ,58, Math.toRadians(270)))
                .build();
        // waitForStart();
        //if(isStopRequested()) return;
        //     bot.intakeSlide.setPosition(50);

        //drive.followTrajectorySequence(right);

        //camera initialization -----------------------------------------------------
        // initCam();
        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;

        switch (blueDetection.getLocation())
        {
            case LEFT:
                drive.setPoseEstimate(startPose);
                bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                bot.setOuttakeSlideState(outtakeSlidesState.STATION);
                bot.setArmState(armState.intaking);
                bot.setArmPosition(armState.intaking, armExtensionState.extending);
                bot.setWristPosition(wristState.intaking);
                bot.setWristState(wristState.intaking);
                
                
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
        blueDetection = new HSVBlueDetection(telemetry);

        // yeah what this does is it gets the thing which uses the thing so we can get the thing
        /*
        (fr tho idk what pipeline does, but from what I gathered,
         we basically passthrough our detection into the camera
         and we feed the streaming camera frames into our Detection algorithm)
         */
        camera.setPipeline(blueDetection);

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