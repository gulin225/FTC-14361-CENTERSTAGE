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

@Autonomous(name = "longBlueObjectDetect", group = "Auto")
public class longBlue extends LinearOpMode {
    OpenCvCamera camera;
    HSVBlueDetection blueDetection;
    String webcamName;
    Robot bot;

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-31, 61, Math.toRadians(90));
        //idk if this will cause issues!!!
        double angleOffset = 2;
        //i dont know if these two lines below need to be here ESP initCAM
        //initCam(); try adding this in if doesnt work
        //line 32 may have actually been the issue pls
        drive.setPoseEstimate(startPose);
        //left ------------------------------------------------------------------
        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.active);
                    bot.setActiveIntakeState(activeIntakeState.active);
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-34, 55))
                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setActiveIntakeState(activeIntakeState.inactive);
                })
                .lineToConstantHeading(new Vector2d(-34, 55.01))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);
                })
                .lineToLinearHeading(new Pose2d(-35,37,Math.toRadians(145)))
                .addDisplacementMarker(() -> {
                    ;
                    
                })


                .lineToLinearHeading(new Pose2d(-45,36,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-45,15,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(39, 16))
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(575);
                })
                .lineToConstantHeading(new Vector2d(40, 46.5))
                .lineToConstantHeading(new Vector2d(49, 48.5))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                    
                    
                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(40,43))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(40,39))
                .lineToLinearHeading(new Pose2d(44 ,15, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(45, 16))
                .lineToConstantHeading(new Vector2d(52,16))
                .build();
        //waitForStart();
        // if (isStopRequested()) return;
        //drive.followTrajectorySequence(left);

        //center ------------------------------------------------------------------
        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)

                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.active);
                    bot.setActiveIntakeState(activeIntakeState.active);
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-34, 55))
                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setActiveIntakeState(activeIntakeState.inactive);
                })
                .lineToConstantHeading(new Vector2d(-34, 55.01))
                .waitSeconds(.25)
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);
                })
                .lineToConstantHeading(new Vector2d(-34, 31))
                .lineToConstantHeading(new Vector2d(-34, 34))
                .addDisplacementMarker(() -> {

                    
                })
                .lineToConstantHeading(new Vector2d(-34,39))
                //.lineToConstantHeading(new Vector2d(-38,39))
                .lineToConstantHeading(new Vector2d(-38,42))
                .lineToLinearHeading(new Pose2d(-47,42,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-47,15,Math.toRadians(180)))

                .lineToConstantHeading(new Vector2d(39, 12))
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(550);
                })
                .lineToConstantHeading(new Vector2d(40, 42))
                .lineToConstantHeading(new Vector2d(50, 42))
                .waitSeconds(.25)
                .addDisplacementMarker( () -> {
                    
                    
                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(40, 42))
                .waitSeconds(.5)
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(44,39))
                .lineToLinearHeading(new Pose2d(44 ,15, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(45, 16))
                .lineToConstantHeading(new Vector2d(52,16))
                .build();
        // waitForStart();
        //if (isStopRequested()) return;
        //drive.followTrajectorySequence(center);

        //right ------------------------------------------------------------------
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.active);
                    bot.setActiveIntakeState(activeIntakeState.active);
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(-48, 55))
                .waitSeconds(.25)

                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setActiveIntakeState(activeIntakeState.inactive);
                })
                .lineToConstantHeading(new Vector2d(-48, 55.01))
                .waitSeconds(.25)
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);
                })
                .lineToConstantHeading(new Vector2d(-48, 44))
                .addDisplacementMarker(() -> {
                    ;
                    
                })
                .lineToConstantHeading(new Vector2d(-48,47))
                .lineToConstantHeading(new Vector2d(-34,47))
                //     .lineToConstantHeading(new Vector2d(-34,13))
                .lineToLinearHeading(new Pose2d(-35,13,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(39, 12))
                .addDisplacementMarker(() -> {
                    bot.outtakeSlide.setPosition(575);
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);
                })
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(39,33.5))
                .lineToConstantHeading(new Vector2d(51,33.5))

                .addDisplacementMarker(() -> {
                    
                    
                })
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(48,33))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(45, 30))
                .lineToLinearHeading(new Pose2d(46 ,15, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(45, 15))
                .lineToConstantHeading(new Vector2d(52,16))
                .build();
        //waitForStart();
        //if (isStopRequested())
        //drive.followTrajectorySequence(right);

        //camera initialization -----------------------------------------------------
        initCam();
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


