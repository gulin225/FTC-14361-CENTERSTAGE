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

@Autonomous(name = "distanceTest", group = "Auto")
public class distanceSenseTest extends LinearOpMode {
    Robot bot;
    distanceSensor distanceSense;
    double xOffset, yOffset;
    public void runOpMode() {
        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(9, -61, Math.toRadians(270));

        xOffset = 0;
        yOffset = 0;

        drive.setPoseEstimate(startPose);


        TrajectorySequence Test = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(13, -55))
                .lineToConstantHeading(new Vector2d(10, -10))
                .lineToLinearHeading(new Pose2d(10,-13,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(48, -11))
                .lineToConstantHeading(new Vector2d(48,-40))

                .addDisplacementMarker(() -> {
                    distanceSense.setRedAlliance();
                })

                .strafeLeft(distanceSense.getLeftStackDistance())
                .build();

        waitForStart();
        if (isStopRequested()) return;


        drive.followTrajectorySequence(Test);
    }

}