package org.firstinspires.ftc.teamcode.pedroPathing.DriveTrain;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DiffySwerveConstants {

    public String tLMotor = "tLMotor";
    public String bLMotor = "bLMotor";
    public String tRMotor = "tRMotor";
    public String bRMotor = "bRMotor";

    public String leftEncoder  = "lEncoder";
    public String rightEncoder = "rEncoder";

    public double leftOffsetDeg  = 112;
    public double rightOffsetDeg = 83;

    public double maxSpeed = 0.9;

    public double kP = 0.325;
    public double kI = 0.0;
    public double kD = 0.0005;
    public double kF = 0.0;

    public double flipThresholdRad = 11.0 * Math.PI / 18.0;
    public double speedDeadband = 1e-3;

    public DcMotorSimple.Direction tLDir = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction bLDir = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction tRDir = DcMotorSimple.Direction.FORWARD;
    public DcMotorSimple.Direction bRDir = DcMotorSimple.Direction.FORWARD;

    public double maxPower = 1.0;
}
