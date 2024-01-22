package org.firstinspires.ftc.teamcode.OpModes.Autonomous.RedPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.clawState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.HSVRedDetection;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "CloseRedLeft")
public class CloseRedLeft extends LinearOpMode {
    Robot bot;
    OpenCvCamera camera;
    HSVRedDetection redDetection;
    String webcamName;

    public void runOpMode() {

        bot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d start = new Pose2d(9, -61, Math.toRadians(270));
        drive.setPoseEstimate(start);
        //initCam();

        TrajectorySequence everything = drive.trajectorySequenceBuilder(start)

                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })
                .lineToConstantHeading(new Vector2d(15, -61))
                .waitSeconds(.25)

//                .addDisplacementMarker(5, () -> {
//                    bot.setVirtualFourBarState(virtualFourBarState.init);
//                    bot.setVirtualFourBarPosition(virtualFourBarState.init, virtualFourBarExtensionState.extending);
//                })
                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                })
                .addDisplacementMarker(15, () -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);

                })

                .lineToLinearHeading(new Pose2d(15,-36,Math.toRadians(325)))

                .addDisplacementMarker(30.75, () -> {
                    
                    
                })
                .waitSeconds(.75)
                .lineToLinearHeading(new Pose2d(20,-36,Math.toRadians(180)))


                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(300);
                })
                .waitSeconds(.25)

                .lineToConstantHeading(new Vector2d(59, -24))
                .addDisplacementMarker(() -> {
                    
                    
                })
                .waitSeconds(.25)
                .lineToConstantHeading(new Vector2d(51, -24))

                .build();
        waitForStart();
        if(isStopRequested()) return;

        //     bot.intakeSlide.setPosition(50);
        bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
        bot.setOuttakeSlideState(outtakeSlidesState.STATION);
        bot.setArmState(armState.intaking);
        bot.setArmPosition(armState.intaking, armExtensionState.extending);
        bot.setWristPosition(wristState.intaking);
        bot.setWristState(wristState.intaking);
        
        


        drive.followTrajectorySequence(everything);


    }
}

