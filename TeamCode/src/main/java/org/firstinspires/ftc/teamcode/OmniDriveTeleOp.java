package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "OmniDrive TeleOp (FR/FL/BR/BL)", group = "TeleOp")
public class OmniDriveTeleOp extends OpMode {

    private DcMotor FRwheel, FLwheel, BRwheel, BLwheel;

    private static final double DEADZONE = 0.06;
    private static final double TURN_SCALE = 0.9;
    private static final double SLOW_MODE_SCALE = 0.35;

    @Override
    public void init() {
        FRwheel = hardwareMap.get(DcMotor.class, "FRwheel");
        FLwheel = hardwareMap.get(DcMotor.class, "FLwheel");
        BRwheel = hardwareMap.get(DcMotor.class, "BRwheel");
        BLwheel = hardwareMap.get(DcMotor.class, "BLwheel");

        // Reverse left side so forward stick makes robot go forward
        FLwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        BLwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        FRwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("TeleOp Ready!");
    }

    @Override
    public void loop() {
        // Left stick = drive, right stick X = turn
        double lx = applyDeadzone(gamepad1.left_stick_x);    // strafe
        double ly = applyDeadzone(-gamepad1.left_stick_y);   // forward/back
        double rx = applyDeadzone(gamepad1.right_stick_x) * TURN_SCALE;

        double fl = ly + lx + rx;
        double fr = ly - lx - rx;
        double bl = ly - lx + rx;
        double br = ly + lx - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;

        FLwheel.setPower(fl * scale);
        FRwheel.setPower(fr * scale);
        BLwheel.setPower(bl * scale);
        BRwheel.setPower(br * scale);

        telemetry.addData("LX/LY/RX", "%.2f / %.2f / %.2f", lx, ly, rx);
        telemetry.addData("FL/FR/BL/BR", "%.2f / %.2f / %.2f / %.2f", fl, fr, bl, br);
        telemetry.update();
    }

    private static double applyDeadzone(double v) {
        return (Math.abs(v) < DEADZONE) ? 0.0 : v;
    }
}
