package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class DistanceSensor
{
    com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;

    boolean redSide = false;
    boolean blueSide = false;

    double directionOffset = -1;

    int stackYPose = 36;
    int stackXPose = 48;
    int robotFrontBackOffset = 9;
    int robotSideOffset = 8;

    public  DistanceSensor(HardwareMap hardwareMap)
    {
        //distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "forwardDistanceSensor");
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "leftDistanceSensor");
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "rightDistanceSensor");
    }

    // Remember to account for the distance between the detector and the center of the robot.
    // This will effect the x and Y.
    // This can be changed after the sensor positions are found.2

    public double getLeftStackDistance()
    {
        double yOffset = 0;

        if(redSide == (true))
        {
            double currentLeftdistance = -(72 - distanceSensor.getDistance(DistanceUnit.INCH)) + robotSideOffset;

            if(currentLeftdistance < stackYPose * directionOffset)
            {
                yOffset = -(currentLeftdistance + stackYPose);
                return yOffset;
            }
            else if(currentLeftdistance > stackYPose * directionOffset)
            {
                yOffset = currentLeftdistance + stackYPose;
                return yOffset;
            }
        }
       return yOffset;
    }

    public double getRightStackDistance()
    {
        double yOffset = 0;

        if(redSide == (true))
        {
            double currentRightdistance = (72 - distanceSensor.getDistance(DistanceUnit.INCH)) - robotSideOffset;

            if(currentRightdistance < stackYPose)
            {
                yOffset = stackYPose - currentRightdistance;
                return yOffset;
            }
            else if(currentRightdistance > stackYPose)
            {
                yOffset = -(currentRightdistance - stackYPose);
                return yOffset;
            }
        }
        return yOffset;
    }

    public double getFrontStackDistance()
    {
        double xOffset = 0;

        double currentFrontdistance = - (72 - distanceSensor.getDistance(DistanceUnit.INCH)) + robotFrontBackOffset;

        if(currentFrontdistance < stackXPose)
        {
            xOffset = -(currentFrontdistance + stackXPose);
            return xOffset;
        }
        else if(currentFrontdistance > stackXPose)
        {
            xOffset = stackXPose - currentFrontdistance;
            return xOffset;
        }
        return xOffset;
    }

    public void setRedAlliance()
    {
        if(redSide = false)
        {
            redSide = true;
        }
        else
        {
            redSide = false;
        }
    }

    public void setBlueAlliance()
    {
        if(blueSide = false)
        {
            blueSide = true;
        }
        else
        {
            blueSide = false;
        }
    }

    public double getDistance()
    {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
}

