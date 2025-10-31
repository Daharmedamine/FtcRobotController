package org.firstinspires.ftc.teamcode  ;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public abstract class Simple_autonomous_test extends LinearOpMode
{
    private Autonomous_PID robot = new Autonomous_PID();

    @Override
    public void runOpMode()
    {
        robot.initialize();
        robot.move(10,10,0,0.75);
    }
}
