package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class DistanceSensor
{
    com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;

    int yPose = -36;
    int xPose = -48;

    public  DistanceSensor(HardwareMap hardwareMap)
    {
        distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "forwardDistanceSensor");
    }



    // Remember to account for the distance between the detector and the center of the robot.
    // This will effect the x and Y.
    // This can be changed after the sensor positions are found.2




    public double getRedLeftStackDistance()
    {
        double currentLeftDistance = -distanceSensor.getDistance(DistanceUnit.INCH);
        double diff = 0;

        if(currentLeftDistance < yPose)
        {
            diff = currentLeftDistance + (-1 * yPose);
            return diff;
        }
        else if(currentLeftDistance > yPose)
        {
            diff = (-1 * currentLeftDistance) + yPose;
            return diff;
        }
        return diff;
    }

    public double getRedRightStackDistance()
    {
        double currentLeftdistance = distanceSensor.getDistance(DistanceUnit.INCH);
        double diff = 0;

        return diff;
    }

    public double getRedFrontStackDistance()
    {
        double currentLeftdistance = -distanceSensor.getDistance(DistanceUnit.INCH);
        double diff = 0;

        if(currentLeftdistance < yPose)
        {
            diff = currentLeftdistance + (-1 * yPose);
            return diff;
        }
        else if(currentLeftdistance > yPose)
        {
            diff = (-1 * currentLeftdistance) + yPose;
            return diff;
        }
        return diff;
    }

    public double getRedBackStackDistance()
    {
        double currentLeftdistance = distanceSensor.getDistance(DistanceUnit.INCH);
        double diff = 0;

        return diff;
    }
}

