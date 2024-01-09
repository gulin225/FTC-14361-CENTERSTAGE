package org.firstinspires.ftc.teamcode.OpModes.Autonomous.RedPaths;
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

@Autonomous(name = "LongRedMid")
public class LongRedMid extends LinearOpMode
{

    Robot bot;
    Pose2d startPose = new Pose2d(-31, -61, Math.toRadians(270));
    private double angleOffset = 2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        bot = new Robot(hardwareMap, telemetry);
        //bot.setInBrake();

        drive.setPoseEstimate(startPose);

        TrajectorySequence toLeftTape = drive.trajectorySequenceBuilder(startPose)

                .addDisplacementMarker(() -> {
                    bot.setWristPosition(wristState.init);
                    bot.setWristState(wristState.init);
                    bot.setArmState(armState.init);
                    bot.setArmPosition(armState.init, armExtensionState.extending);
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(-24, -60))
                .addDisplacementMarker( () -> {
                    bot.setWristPosition(wristState.outtaking);
                    bot.setWristState(wristState.outtaking);
                })
                .lineToConstantHeading(new Vector2d(-24,-60.01))
                .waitSeconds(.5)
                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.autoDrop, armExtensionState.extending);
                    bot.setArmState(armState.autoDrop);

                })


                .lineToConstantHeading(new Vector2d(-24, -35))
                .addDisplacementMarker(() -> {
                    bot.setClawPosition(clawState.leftClose);
                    bot.setClawState(clawState.leftClose);
                })


                .lineToConstantHeading(new Vector2d(-24,-40))
                .lineToConstantHeading(new Vector2d(-36,-40))
                .lineToConstantHeading(new Vector2d(-36,-15))
                .lineToLinearHeading(new Pose2d(-34,-10,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(44, -10))


                .addDisplacementMarker( () -> {
                    bot.setArmPosition(armState.outtaking, armExtensionState.extending);
                    bot.setArmState(armState.outtaking);

                    bot.outtakeSlide.setPosition(500);
                })

                .waitSeconds(.25)

                .lineToConstantHeading(new Vector2d(44, -30))
                .lineToConstantHeading(new Vector2d(60,-30))
                .addDisplacementMarker( () -> {
                    bot.setClawPosition(clawState.rightClose);
                    bot.setClawState(clawState.rightClose);
                })

                .lineToConstantHeading(new Vector2d(45,-31))
                .addDisplacementMarker(() -> {
                    bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
                    bot.setArmPosition(armState.intaking, armExtensionState.extending);
                    bot.setWristPosition(wristState.intaking);
                })
                .lineToLinearHeading(new Pose2d(50,-28,Math.toRadians(450)))



                .build();
        waitForStart();


        if (isStopRequested()) return;
        bot.setOuttakeSlidePosition(outtakeSlidesState.STATION, extensionState.extending);
        bot.setOuttakeSlideState(outtakeSlidesState.STATION);
        bot.setArmState(armState.intaking);
        bot.setArmPosition(armState.intaking, armExtensionState.extending);
        bot.setWristPosition(wristState.intaking);
        bot.setWristState(wristState.intaking);
        bot.setClawPosition(clawState.open);
        bot.setClawState(clawState.open);

        drive.followTrajectorySequence(toLeftTape);
    }
}
