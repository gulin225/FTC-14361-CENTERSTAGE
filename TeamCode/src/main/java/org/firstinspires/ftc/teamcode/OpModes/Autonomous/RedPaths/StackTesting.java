package org.firstinspires.ftc.teamcode.OpModes.Autonomous.RedPaths;

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

@Autonomous(name = "stackTesting", group = "Auto")
public class StackTesting extends LinearOpMode {
    OpenCvCamera camera;

    HSVRedDetection redDetection;
    String webcamName;
    Robot bot;

    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        initCam();

        // ---------------------------- Left ---------------------------- //

        TrajectorySequence strafeIntoStack = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(1.5,0))
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                  bot.setArmPosition(armState.intaking, armExtensionState.extending);
                  bot.setWristPosition(wristState.intaking);
                  bot.linkage.setLinkageCustomPosition(.88);
                  bot.activeIntake.setActiveIntakePower(.4);
                })

                .lineToLinearHeading(new Pose2d(6.5, -3 ,Math.toRadians(45)))

                .lineToLinearHeading(new Pose2d(6.5, 0 ,Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                    bot.linkage.setLinkageCustomPosition(.96);
                    bot.setActiveIntakePosition(activeIntakeState.active);
                })
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(6.6,0))
                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(4,0))
                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.activeReverse);
                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(3.99,0))
                .addDisplacementMarker(() -> {
                    bot.setActiveIntakePosition(activeIntakeState.inactive);
                })
                .build();


        TrajectorySequence dropdownLinkage = drive.trajectorySequenceBuilder(startPose)

                .lineToConstantHeading(new Vector2d(1,0))
                .addDisplacementMarker(() -> {
                    bot.setLidPosition(lidState.open);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                    bot.linkage.setLinkageCustomPosition(82);
                })
                .lineToConstantHeading(new Vector2d(6.5,0))
                .lineToConstantHeading(new Vector2d(6.6,0))
                .addDisplacementMarker(() -> {
                    bot.linkage.setLinkageCustomPosition(92);
                })
                .lineToConstantHeading(new Vector2d(5,0))
                .addDisplacementMarker(() -> {
                  bot.setActiveIntakePosition(activeIntakeState.active);
                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(3,0))



                .build();



        // ---------------------------- Camera Initialization ---------------------------- //

        waitForStart();
        camera.stopStreaming();
        if (isStopRequested()) return;
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequence(strafeIntoStack);


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
