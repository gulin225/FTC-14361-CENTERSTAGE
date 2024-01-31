package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class distanceSensor
{
//    DistanceSensor frontDistanceSensor, rightDistanceSensor, backDistanceSensor, leftDistanceSensor;
//
//    int yPose = -36;
//    int xPose = -48;
//
//    public distanceSensor(HardwareMap hardwareMap)
//    {
//        frontDistanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "frontDistanceSensor");
//        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
//       // rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
//
//    }
//
//
//
//    // Remember to account for the distance between the detector and the center of the robot.
//    // This will effect the x and Y.
//    // This can be changed after the sensor positions are found.2
//
//
//
//public double getFrontDistance(){
//        return frontDistanceSensor.getDistance(DistanceUnit.INCH);
//}
////public double getLeftDistance(){
////    return leftDistanceSensor.getDistance(DistanceUnit.INCH);
////}
//
//    public double getRedLeftStackDistance()
//    {
//        double currentLeftDistance = -leftDistanceSensor.getDistance(DistanceUnit.INCH);
//        double diff = 0;
//
//        if(currentLeftDistance < yPose)
//        {
//            diff = currentLeftDistance + (-1 * yPose);
//            return diff;
//        }
//        else if(currentLeftDistance > yPose)
//        {
//            diff = (-1 * currentLeftDistance) + yPose;
//            return diff;
//        }
//        return diff;
//    }
//
//    public double getRedRightStackDistance()
//    {
//        double currentLeftdistance = rightDistanceSensor.getDistance(DistanceUnit.INCH);
//        double diff = 0;
//
//        return diff;
//    }
//
//    public double getRedFrontStackDistance()
//    {
//        double currentLeftdistance = -frontDistanceSensor.getDistance(DistanceUnit.INCH);
//        double diff = 0;
//
//        if(currentLeftdistance < yPose)
//        {
//            diff = currentLeftdistance + (-1 * yPose);
//            return diff;
//        }
//        else if(currentLeftdistance > yPose)
//        {
//            diff = (-1 * currentLeftdistance) + yPose;
//            return diff;
//        }
//        return diff;
//    }
//
//    public double getRedBackStackDistance()
//    {
//        double currentLeftdistance = backDistanceSensor.getDistance(DistanceUnit.INCH);
//        double diff = 0;
//
//        return diff;
//    }

}
