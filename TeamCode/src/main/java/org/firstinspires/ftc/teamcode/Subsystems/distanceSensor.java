package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.arcrobotics.ftclib.hardware.SensorDistanceEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
import java.util.Map;

public class distanceSensor {
    DistanceSensor distanceSensor;
    public distanceSensor(HardwareMap hardwareMap){

        distanceSensor = hardwareMap.get(DistanceSensor.class, "forwardDistanceSensor");

        }

        public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.INCH);
        }
    }

