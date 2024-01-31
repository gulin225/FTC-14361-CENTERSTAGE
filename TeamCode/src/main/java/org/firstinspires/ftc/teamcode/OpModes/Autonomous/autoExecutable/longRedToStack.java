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
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.lidState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.distanceSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name = "longRedUnderTrussToStack", group = "Auto")
public class longRedToStack extends LinearOpMode {
    OpenCvCamera camera;
    HSVRedDetection redDetection;
    String webcamName;
    Robot bot;

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-40.5, -61, Math.toRadians(270));
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
                    ;

                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-45,-15,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-59.5,-25))
                .addDisplacementMarker(() -> {

                })
                .lineToConstantHeading(new Vector2d(-55,-25))
                .addDisplacementMarker(() -> {

                })
                .lineToConstantHeading(new Vector2d(-47,-15))

                .lineToConstantHeading(new Vector2d(39, -12))
                .addDisplacementMarker( () -> {

                })


                .lineToConstantHeading(new Vector2d(39, -16))
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(575);
                })
                .lineToConstantHeading(new Vector2d(40, -24.5))
                .lineToConstantHeading(new Vector2d(49, -24.5))
                .waitSeconds(.25)
                .addDisplacementMarker(() -> {


                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(40,-43))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(40,-39))
                .lineToLinearHeading(new Pose2d(44 ,-15, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(45, -16))
                .lineToConstantHeading(new Vector2d(52,-16))
                .build();
        //waitForStart();
        // if (isStopRequested()) return;
        //drive.followTrajectorySequence(left);

        //center ------------------------------------------------------------------
        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)


                .lineToConstantHeading(new Vector2d(-43,-33.5))
                .lineToLinearHeading(new Pose2d(-39,-55, Math.toRadians(180)))
                .addDisplacementMarker(55,() ->{
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);                })
                //  .lineToConstantHeading(new Vector2d(-43,-42))
                //  .lineToLinearHeading(new Pose2d(-43,-55, Math.toRadians(180)))
                // .waitSeconds(.05)
                .lineToConstantHeading(new Vector2d(42,-55))
                .addDisplacementMarker( 100,() -> {

                    bot.outtakeSlide.setPosition(800);
                })
                .lineToConstantHeading(new Vector2d(52, -29.5))
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);

                })

                .lineToConstantHeading(new Vector2d(30,-55))
                .addDisplacementMarker(125, () -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(-40,-53))
                .splineToLinearHeading(new Pose2d(-54.5,-33, Math.toRadians(145)), Math.toRadians(180))
                .addDisplacementMarker(200,() -> {
                    bot.linkage.setLinkageCustomPosition(.96);
                    bot.setActiveIntakePosition(activeIntakeState.active);
                })

                // .splineToConstantHeading(new Vector2d(-59,-43), Math.toRadians(180))
                //  .splineToLinearHeading(new Pose2d(-55,-30,Math.toRadians(135)), Math.toRadians(135))
                // .waitSeconds(.1)
                .turn(Math.toRadians(70))

                //  .turn(Math.toRadians(-35))

                .forward(3)

                .lineToLinearHeading(new Pose2d(-15,-55, Math.toRadians(180)))

                .addDisplacementMarker(230, () -> {
                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                    bot.setWristPosition(wristState.init);
                })


                .lineToConstantHeading(new Vector2d(40,-55))
                .addDisplacementMarker( 260,() -> {
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                    bot.setLidPosition(lidState.close);
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.outtaking);
                    bot.outtakeSlide.setPosition(800);
                })
                .lineToConstantHeading(new Vector2d(52,-37))
                .addDisplacementMarker( () -> {
                    bot.setLidPosition(lidState.open);
                    bot.setOuttakeSlidePosition(outtakeSlidesState.MEDIUMOUT, extensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(50,-37))
                .build();



        // waitForStart();
        //if (isStopRequested()) return;
        //drive.followTrajectorySequence(center);

        //right ------------------------------------------------------------------
        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
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
                    bot.outtakeSlide.setPosition(575);
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);
                })
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(39,-50))
                .lineToConstantHeading(new Vector2d(51,-50))

                .addDisplacementMarker(() -> {
//
//
                })
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(48,-33))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToConstantHeading(new Vector2d(45, -30))
                .lineToLinearHeading(new Pose2d(46 ,-15, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(45, -15))
                .lineToConstantHeading(new Vector2d(52,-16))
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