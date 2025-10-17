package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "OmniDrive + Shooter (FR/FL/BR/BL)", group = "TeleOp")
public class OmniDriveTeleOp extends OpMode {

    private DcMotor FRwheel, FLwheel, BRwheel, BLwheel;
    private DcMotor shooter1, shooter2;

    private static final double DEADZONE = 0.06;
    private static final double TURN_SCALE = 0.9;
    private static final double SLOW_MODE_SCALE = 0.35;

    // === Shooter variables ===
    private double shooterPower = 0.8;  // <== Change this to tune shooter speed
    private boolean shooterOn = false;
    private boolean aPressedLast = false;

    @Override
    public void init() {
        FRwheel = hardwareMap.get(DcMotor.class, "FRwheel");
        FLwheel = hardwareMap.get(DcMotor.class, "FLwheel");
        BRwheel = hardwareMap.get(DcMotor.class, "BRwheel");
        BLwheel = hardwareMap.get(DcMotor.class, "BLwheel");

        shooter1 = hardwareMap.get(DcMotor.class, "motor1"); // connected to Expansion Hub port 1
        shooter2 = hardwareMap.get(DcMotor.class, "motor3"); // connected to Expansion Hub port 3

        // Reverse one shooter if they spin opposite directions
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        FLwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        BLwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        FRwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("TeleOp Ready!");
    }

    @Override
    public void loop() {
        // ===== Movement control =====
        double lx = applyDeadzone(Math.pow(gamepad1.left_stick_x, 3));
        double ly = applyDeadzone(Math.pow(-gamepad1.left_stick_y, 3));
        double rx = applyDeadzone(Math.pow(gamepad1.right_stick_x, 3)) * TURN_SCALE;

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

        // ===== Shooter toggle =====
        if (gamepad1.a && !aPressedLast) {
            shooterOn = !shooterOn; // toggle shooter state
        }
        aPressedLast = gamepad1.a;

        if (shooterOn) {
            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }

        // ===== Telemetry =====
        telemetry.addData("LX/LY/RX", "%.2f / %.2f / %.2f", lx, ly, rx);
        telemetry.addData("FL/FR/BL/BR", "%.2f / %.2f / %.2f / %.2f", fl, fr, bl, br);
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Shooter Power", "%.2f", shooterPower);
        telemetry.update();
    }

    private static double applyDeadzone(double v) {
        return (Math.abs(v) < DEADZONE) ? 0.0 : v;
    }
}
