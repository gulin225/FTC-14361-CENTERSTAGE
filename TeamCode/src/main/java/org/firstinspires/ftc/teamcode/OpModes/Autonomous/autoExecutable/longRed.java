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

                // this aligns to tape
                .lineToConstantHeading(new Vector2d(-37,-52))

                .addDisplacementMarker(() -> {
                    // init all transfer stuff
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.init);
                    bot.setLidPosition(lidState.close);

                })

                // pushes to tape
                .lineToConstantHeading(new Vector2d(-38, -44))

                // going back to not mess pixel up
                .lineToConstantHeading(new Vector2d(-38, -50))
                // back up and move towards center
                .lineToConstantHeading(new Vector2d(-32,-50))

                // go to gate level & face outtaking
                .lineToLinearHeading(new Pose2d(-32,-12, Math.toRadians(180)))

                // stopping after going under gate
                .lineToConstantHeading(new Vector2d(45,-12))

                // go under gate
                .lineToConstantHeading(new Vector2d(45,-55))

                // set all transfer to outtake states
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                   bot.setWristPosition(wristState.outtaking);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                })

                // go to backboard
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(61, -23))

                // score!
                .addDisplacementMarker(() -> {
                bot.setLidState(lidState.open);
                })
                .waitSeconds(.25)

                // back up from board
                .lineToConstantHeading(new Vector2d(50,-23))

                // set states for tele op
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.init);
                })

                // park
                .lineToLinearHeading(new Pose2d(50 ,-15, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(55,-15))

                .build();


        //center ------------------------------------------------------------------
        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)

                // going forward & little left
                .lineToConstantHeading(new Vector2d(-26,-52))

                .addDisplacementMarker(() -> {
                // init all transfer stuff
                    bot.setWristState(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidState(lidState.close);
                })

                // to center tape to push pixel
                .lineToConstantHeading(new Vector2d(-26, -36))

                // Backing up and aligning with backboard
                .lineToConstantHeading(new Vector2d(-26  ,-42))

                // turning to face backboard & moving towards backboard
                .lineToLinearHeading(new Pose2d(-34,-42, Math.toRadians(180)))

                // going to level with gate
                .lineToConstantHeading(new Vector2d(-34 , -12))

                // stopping after going under gate
                .lineToConstantHeading(new Vector2d(45,-12))

                // all transfer set to outtake
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                // wait a second so it finishes outtake setup
                .waitSeconds(.25)

                // going to backboard
                .lineToConstantHeading(new Vector2d(61,-29))

                // open up the box
                .addDisplacementMarker( () -> {
                 bot.setLidState(lidState.open);
                })

                // back up from backboard
                .lineToConstantHeading(new Vector2d(50,-31))

                // prepare states for tele op
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.init);
                })

                // parking
                .lineToLinearHeading(new Pose2d(50 ,-15, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(55,-15))

                .build();


        //right ------------------------------------------------------------------
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)

                // move left a lil
                .lineToConstantHeading(new Vector2d(-36,-55))

                // set all transfer states to init
                .addDisplacementMarker(() -> {
                    bot.setWristState(wristState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setLidState(lidState.close);
                })

                // move right towards tape
                .lineToConstantHeading(new Vector2d(-32, -55))

                // turn robot & go up next to tape
                .lineToLinearHeading(new Pose2d(-32,-29, Math.toRadians(180)))

                // score pixel on tape (untested)
                .lineToConstantHeading(new Vector2d(-24,-29))

                //move away from tape to not mess it up

                .lineToConstantHeading(new Vector2d(-34,-29))
                // move to truss

                .lineToConstantHeading(new Vector2d(-34,-12))


                //move to close side
                .lineToConstantHeading(new Vector2d(45,-12))

                // set all transfer to outtaking
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.LOWOUT, extensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })

                // go to backboard
                .lineToConstantHeading(new Vector2d(62, -35.5))

                // wait & then open up lid and score
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {
                bot.setLidPosition(lidState.open);

                })

                // back up to start parking
                .lineToConstantHeading(new Vector2d(50,-38))

                // set transfer to teleop ready
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.init);
                })

                // head to park zone
                .lineToConstantHeading(new Vector2d(55, -33))

                // turn & park 
                .lineToLinearHeading(new Pose2d(50 ,-15, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(55,-15))

                .build();


        //waitForStart();
        //if (isStopRequested())
        //drive.followTrajectorySequence(right);

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

