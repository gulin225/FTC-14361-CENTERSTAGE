package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.lidState;
import org.firstinspires.ftc.teamcode.util.robotConstants;


public class Lid
{
    private ServoEx lid;
    double minAngle = 0, maxAngle= 360;
    public Lid(HardwareMap hardwareMap)
    {
        lid = new SimpleServo(hardwareMap, "lidServo", minAngle, maxAngle, AngleUnit.DEGREES);
    }

    public void setLidPosition(lidState lidState)
    {
        switch(lidState)
        {
            case open:
                lid.setPosition(robotConstants.Lid.open);
                break;
            case close:
                lid.setPosition(robotConstants.Lid.close);
                break;
            default:
                lid.setPosition(robotConstants.Lid.open);
        }
    }
    public double getLidPosition()
    {
        return lid.getPosition();
    }
    public void setLidCustomPosition(double position){
        lid.setPosition(position);
    }

}