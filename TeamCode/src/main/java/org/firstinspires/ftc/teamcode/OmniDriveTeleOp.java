package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp ", group = "TeleOp")
public class OmniDriveTeleOp extends OpMode {

    private DcMotor FRwheel, FLwheel, BRwheel, BLwheel;
    private DcMotor shooter1, shooter2;
    private DcMotor IntakeMotor;
    private DcMotor Index;

    private static final double DEADZONE = 0.06;
    private static final double TURN_SCALE = 0.9;
    private static final double SLOW_MODE_SCALE = 0.35;

    private double shooterPower = 1;
    private double IntakePower = 1;
    private boolean shooterOn = false;
    private boolean IntakeOn = false;
    private boolean shooterReverse = false;
    private boolean intakeReverse = false;
    private boolean aPressedLast = false;
    private boolean bPressedLast = false;
    private boolean xPressedLast = false;
    private boolean yPressedLast = false;

    @Override
    public void init() {
        FRwheel = hardwareMap.get(DcMotor.class, "FRwheel");
        FLwheel = hardwareMap.get(DcMotor.class, "Flwheel");
        BRwheel = hardwareMap.get(DcMotor.class, "BRwheel");
        BLwheel = hardwareMap.get(DcMotor.class, "BLwheel");

        shooter1 = hardwareMap.get(DcMotor.class, "shooter_left");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter_right");
        IntakeMotor = hardwareMap.get(DcMotor.class, "intake");
        Index = hardwareMap.get(DcMotor.class, "Index");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FLwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        BLwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        FRwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
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

        if (gamepad2.b && !bPressedLast) {
            shooterOn = !shooterOn;
        }
        bPressedLast = gamepad2.b;

        if (gamepad2.x && !xPressedLast) {
            IntakeOn = !IntakeOn;
        }
        xPressedLast = gamepad2.x;

        if (gamepad1.a && !aPressedLast) {
            shooterReverse = !shooterReverse;
            intakeReverse = !intakeReverse;
        }
        aPressedLast = gamepad1.a;

        if (gamepad2.y && !yPressedLast) {
            intakeReverse = !intakeReverse;
        }
        yPressedLast = gamepad2.y;

        boolean shooterActive = (gamepad2.right_trigger > 0.1) || shooterOn;
        boolean intakeActive = (gamepad2.left_trigger > 0.1) || IntakeOn;

        double shooterDir = shooterReverse ? -1 : 1;
        double intakeDir = intakeReverse ? -1 : 1;

        if (shooterActive) {
            shooter1.setPower(shooterPower * shooterDir);
            shooter2.setPower(shooterPower * shooterDir);
            Index.setPower(0.4 * shooterDir);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
            Index.setPower(0);
        }

        if (intakeActive) {
            IntakeMotor.setPower(IntakePower * intakeDir);
        } else {
            IntakeMotor.setPower(0);
        }
    }

    private static double applyDeadzone(double v) {
        return (Math.abs(v) < DEADZONE) ? 0.0 : v;
    }
}
