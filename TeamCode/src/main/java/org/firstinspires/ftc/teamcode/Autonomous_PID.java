package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Autonomous_PID {

    // Motors and encoders
    Motor Fl, Bl, Fr, Br;
    DcMotor Xencoder, Yencoder, Rencoder;

    // PID Controller
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    private final double Inch_per_Rotation_of_Wheel = 1.25984; // Replace with actual value

    // Dependencies from OpMode
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Constructor
    public Autonomous_PID() {
        this.hardwareMap = hardwareMap;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    // Initialization method
    public void initialize() {
        controller = new PIDController(p, i, d);

        Fl = new Motor(hardwareMap, "FLwheel");
        Bl = new Motor(hardwareMap, "BLwheel");
        Fr = new Motor(hardwareMap, "FRwheel");
        Br = new Motor(hardwareMap, "BRwheel");

        Xencoder = hardwareMap.get(DcMotorEx.class, "BRwheel");
        Yencoder = hardwareMap.get(DcMotorEx.class, "FLwheel");
    }

    // Movement method
    public void move(double Xtarget, double Ytarget, double Rtarget, double speed) {
        double targetX = Xtarget / Inch_per_Rotation_of_Wheel;
        double targetY = Ytarget / Inch_per_Rotation_of_Wheel;

        controller.setPID(p, i, d);

        int posX = Xencoder.getCurrentPosition();
        int posY = Yencoder.getCurrentPosition();

        double Xpid = controller.calculate(posX, targetX);
        double Ypid = controller.calculate(posY, targetY);
        double rotation = rotate(Rtarget);

        double Xpower = Xpid;
        double Ypower = Ypid;

        telemetry.addData("X Position", posX * Inch_per_Rotation_of_Wheel);
        telemetry.addData("Y Position", posY  * Inch_per_Rotation_of_Wheel);
        telemetry.addData("Rotation", Math.atan2(posY, posX));
        telemetry.update();

        // Mecanum drive calculation
        Fl.set((Xpower + Ypower + rotation) * speed);
        Bl.set((Xpower - Ypower + rotation) * speed);
        Fr.set((Xpower - Ypower - rotation) * speed);
        Br.set((Xpower + Ypower - rotation) * speed);
    }

    // Rotation logic
    private double rotate(double Rtarget) {
        if (Rtarget == 0) {
            return 0;
        }

        int posX = Xencoder.getCurrentPosition();
        int posY = Yencoder.getCurrentPosition();

        return controller.calculate(Math.atan2(posY, posX), Rtarget);
    }
}