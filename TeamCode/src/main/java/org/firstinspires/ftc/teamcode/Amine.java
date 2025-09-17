package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Amine extends OpMode {
    @Override
    public void init() {
        telemetry.addData("hi", "amine is a cool person")
    }
}
