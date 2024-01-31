package org.firstinspires.ftc.teamcode.OpModes.Autonomous.autoExecutable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Commands.activeIntakeState;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.clawState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.lidState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name = "goobTestLongRed", group = "goobTest")
public class LongCenterRedStack extends LinearOpMode {
    OpenCvCamera camera;
    HSVRedDetection redDetection;
    String webcamName;
    Robot bot;


    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-40.5, -61, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        //left ------------------------------------------------------------------
//        TrajectorySequence toTapeDropOff = drive.trajectorySequenceBuilder(startPose)
//
//                //Push to tape
//                //Move away from tape

//
//                .build();
        TrajectorySequence centerUnderTruss = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-43,-33.5))
                .lineToLinearHeading(new Pose2d(-39,-55, Math.toRadians(180)))

              //  .lineToConstantHeading(new Vector2d(-43,-42))
              //  .lineToLinearHeading(new Pose2d(-43,-55, Math.toRadians(180)))
               // .waitSeconds(.05)
                .lineToConstantHeading(new Vector2d(42,-55))
                .addDisplacementMarker(55,() ->{
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })

                .build();
        TrajectorySequence rightUnderTruss = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-36,-33,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-33, -33))
                .lineToConstantHeading(new Vector2d(-39, -55))

                .lineToConstantHeading(new Vector2d(40,-55))
                .addDisplacementMarker(55,() ->{
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .build();
        TrajectorySequence leftUnderTruss = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-48.25, -44))
                .lineToLinearHeading(new Pose2d(-39,-55, Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(40,-55))
                .addDisplacementMarker(55,() ->{
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                })
                .build();
        TrajectorySequence toCenterBackboard = drive.trajectorySequenceBuilder(centerUnderTruss.end())
//                .lineToConstantHeading(new Vector2d(-43,-42))
//                .lineToLinearHeading(new Pose2d(-43,-55, Math.toRadians(180)))


                .lineToConstantHeading(new Vector2d(52, -29.5))
                .addDisplacementMarker(3, () -> {

                    bot.outtakeSlide.setPosition(800);
                })
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);

                })
                .addDisplacementMarker( 35,() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(30,-55))
                .lineToConstantHeading(new Vector2d(-45,-53))


                .build();
        TrajectorySequence toRightBackboard = drive.trajectorySequenceBuilder(rightUnderTruss.end())
                .lineToConstantHeading(new Vector2d(52,-34))
                .addDisplacementMarker(3,() ->{
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.outtakeSlide.setPosition(800);

                })

                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);

                })
              //  .lineToConstantHeading(new Vector2d(50,-34))
                .lineToConstantHeading(new Vector2d(30,-55))
                .lineToConstantHeading(new Vector2d(-45,-53))
                .build();
        TrajectorySequence toLeftBackboard = drive.trajectorySequenceBuilder(leftUnderTruss.end())
                .lineToConstantHeading(new Vector2d(51.3, -22))
                .addDisplacementMarker(3,() ->{
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.outtakeSlide.setPosition(800);

                })
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })

               // .lineToConstantHeading(new Vector2d(47,-22))
                .splineToConstantHeading(new Vector2d(30,-55), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-45,-53))
                .build();

//        TrajectorySequence leaveBackBoard = drive.trajectorySequenceBuilder(toBackboard.end())
////                .lineToConstantHeading(new Vector2d(-43,-42))
////                .lineToLinearHeading(new Pose2d(-43,-55, Math.toRadians(180)))
//
//
//                //.lineToConstantHeading(new Vector2d(30,-55))
//
//                //.splineToConstantHeading(new Vector2d(30, -55),Math.toRadians(270))
//
//
//               // .waitSeconds(.1)
//
//               // .splineToConstantHeading(new Vector2d(-40,-53), Math.toRadians(0))
//
//
//
//
//                .build();
        TrajectorySequence toStack = drive.trajectorySequenceBuilder(toCenterBackboard.end())
              //  .turn(Math.toRadians(-35))


               // .splineToLinearHeading(new Pose2d(-54.5,-33, Math.toRadians(145)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-54.5,-33, Math.toRadians(145)))
                .addDisplacementMarker(20,() -> {
                    bot.linkage.setLinkageCustomPosition(.96);
                    bot.setActiveIntakePosition(activeIntakeState.active);
                })

               // .splineToConstantHeading(new Vector2d(-59,-43), Math.toRadians(180))
                //  .splineToLinearHeading(new Pose2d(-55,-30,Math.toRadians(135)), Math.toRadians(135))
               // .waitSeconds(.1)
                .turn(Math.toRadians(70))

              //  .turn(Math.toRadians(-35))

                //.forward(3)
               // .forward(bot.distanceSensor.getFrontDistance()-.5)
               // .strafeLeft(bot.distanceSensor.getLeftDistance()-24)
                .lineToLinearHeading(new Pose2d(-25,-56, Math.toRadians(180)))

                .addDisplacementMarker(50, () -> {
                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                   // bot.setLidPosition(lidState.close);
                })
                .addDisplacementMarker(75, () -> {
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.init);

                })
                .addDisplacementMarker(80, () -> {
                    bot.setLidPosition(lidState.close);
                })



                .lineToConstantHeading(new Vector2d(40,-55))
                .lineToConstantHeading(new Vector2d(52,-34))
                .addDisplacementMarker(120,() ->{
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.outtakeSlide.setPosition(800);

                })

                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);

                })
                .lineToConstantHeading(new Vector2d(50,-34))

                .build();
//        TrajectorySequence toBackboardFromTruss = drive.trajectorySequenceBuilder(toStack.end())
//
//
//
//
//
//                .build();



        initCam();
        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;

        switch (redDetection.getLocation())
        {
            case LEFT:
                drive.setPoseEstimate(startPose);


                drive.followTrajectorySequence(leftUnderTruss);
                drive.followTrajectorySequence(toLeftBackboard);
                drive.followTrajectorySequence(toStack);
                //drive.followTrajectorySequence(toBackboardFromTruss);
                break;
            case RIGHT:
                drive.setPoseEstimate(startPose);
                drive.followTrajectorySequence(rightUnderTruss);
                drive.followTrajectorySequence(toRightBackboard);
                drive.followTrajectorySequence(toStack);
               // drive.followTrajectorySequence(toBackboardFromTruss);


                break;
            case MIDDLE:
                drive.setPoseEstimate(startPose);


                drive.followTrajectorySequence(centerUnderTruss);
                drive.followTrajectorySequence(toCenterBackboard);
                drive.followTrajectorySequence(toStack);
               // drive.followTrajectorySequence(toBackboardFromTruss);
                break;
        }

   //     drive.followTrajectorySequence(center);
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