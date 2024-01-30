package org.firstinspires.ftc.teamcode.OpModes.Autonomous.BluePaths;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Commands.clawState;
import org.firstinspires.ftc.teamcode.Commands.armExtensionState;
import org.firstinspires.ftc.teamcode.Commands.armState;
import org.firstinspires.ftc.teamcode.Commands.extensionState;
import org.firstinspires.ftc.teamcode.Commands.outtakeSlidesState;
import org.firstinspires.ftc.teamcode.Commands.wristState;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.drive.SampleMecanumDrive;

public class LongBlueRight extends LinearOpMode {
    Robot bot;
    Pose2d startPose = new Pose2d(-31, 61, Math.toRadians(90));
    private double angleOffset = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        bot = new Robot(hardwareMap, telemetry);

        drive.setPoseEstimate(startPose);

        TrajectorySequence toLeftTape = drive.trajectorySequenceBuilder(startPose)
                //Initialization
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })


                .lineToConstantHeading(new Vector2d(-48, 59))
                .waitSeconds(.25)

                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                })
                .lineToConstantHeading(new Vector2d(-48, 59.01))
                .waitSeconds(.25)
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);
                })


                .lineToConstantHeading(new Vector2d(-48, 44))

                .addDisplacementMarker(() -> {
                    
                    
                })
                .lineToConstantHeading(new Vector2d(-48,47))
                .lineToConstantHeading(new Vector2d(-34,47))
           //     .lineToConstantHeading(new Vector2d(-34,13))

                .lineToLinearHeading(new Pose2d(-35,13,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(39, 12))


                .addDisplacementMarker(() -> {
                    bot.outtakeSlide.setPosition(500);

                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);
                })

                .waitSeconds(.5)
                .lineToConstantHeading(new Vector2d(39,32.5))
                .lineToConstantHeading(new Vector2d(51,32.5))

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

        waitForStart();

        if (!isStopRequested())

            bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
        bot.setOuttakeSlideState(outtakeSlidesState.STATION);
        bot.setArmState(armState.intaking);
        bot.setArmPosition(armState.intaking, armExtensionState.extending);
        bot.setWristPosition(wristState.intaking);
        bot.setWristState(wristState.intaking);
        
        

        drive.followTrajectorySequence(toLeftTape);
    }
}
