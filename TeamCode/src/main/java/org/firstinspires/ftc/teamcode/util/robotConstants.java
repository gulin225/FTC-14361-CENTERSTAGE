package org.firstinspires.ftc.teamcode.util;

public class robotConstants
{
    public static class outtakeSlide
    {
        public static double P = 0.0;
        public static double I = 0.0;
        public static double D = 0.0;

        public static double pulleyCircumference = 0.0;
        public static double ticksPerRevolution = 0.0;

        public static int MOSTHIGHLEFT = 1300;
        public static int MOSTHIGHRIGHT = 1300;

        public static int HIGHLEFT = 1275;
        public static int HIGHRIGHT = 1275;

        public static int MEDIUMLEFT = 1050;
        public static int MEDIUMRIGHT = 1050;
        public static int LOWMEDLEFT = 850;
        public static int LOWMEDRIGHT=850;

        public static int LOWLEFT = 550;
        public static int LOWRIGHT = 550;

        public static int AUTOLOWLEFT = 100;
        public static int AUTOLOWRIGHT = 100;

        public static int GROUNDLEFT = 0;
        public static int GROUNDRIGHT = 0;
    }

    public static class intakeSlide
    {
        public static double P = 0.5;
        public static double I = 0.5;
        public static double D = 0.5;

        public static double pulleyCircumference = 0.0;
        public static double ticksPerRevolution = 0.0;

        public static int highExtension = 715;
        public static int mediumExtension = 230;
        public static int retracted = 0;
    }

    public static class Claw
    {
        public static double intakeAuto = 0.0;
        public static double intakeTeleOp = 0.0;

        public static double leftClose = .965;
        public static double rightClose = .31;
        public static double leftOpen = .74;
        public static double rightOpen = .55;
        public static double autoLeftClose = .65;
        public static double autoRightClose = .63;
        public static double rightCloseOnePixel = .58;
    }

    public static class activeIntake
    {

        public static double active = -.9;
        public static double reverseActive = .9;

        public static double autoActive = 0.34;
        public static double autoReverseActive = -0.34;
    }

    public static class Climb
    {
        public static int climbPosition = 0;
    }

    public static class Wrist
    {
        public static double outtaking = .2;
        public static double intaking = 1;
//        public static double intaking = .79;
        public static double init = .65;
    }
    public static class Lid
    {
        public static double close = 0;
        public static double open =.4 ;
    }

    public static class virtualFourBar

    {
//        public static double intakingLeft = 0.01;
//
//        public static double intakingRight = 1;
//
//
//        public static double intakingInitLeft = 0.06;
//
//        public static double intakingInitRight = .70;
//
//
//        public static double outtakingLeft = .52;
//        public static double outtakingRight = .25;

//        public static double outtakingLeftHigh = .59;
//        public static double outtakingRightHigh = .29;

        public static double intakingLeft = 0.045;

        public static double intakingRight = 0.045;


        public static double intakingInitLeft = .5;

        public static double intakingInitRight = .5;

        public static double outtakingLeft = .565;
        public static double outtakingRight = .53;

        public static double initLeft = .21;

        public static double initRight = .2;

        public static double outtakingDownLeft = .62;
        public static double outtakingDownRight = .08;

        public static double autoDropLeft = .57;
        public static double autoDropRight = .15;

        public static double intakingLeftAuton = 0.5;
        public static double intakingRightAuton = 0.6;
    }

    public static class linkage
    {
        public static double highPosition = .8;
        public static double mediumPosition = .9;
        public static double lowPosition = 1;

    }

    public static class drone{
        public static double droneLaunch = .4;
        public static double droneReset = .6;
    }


}