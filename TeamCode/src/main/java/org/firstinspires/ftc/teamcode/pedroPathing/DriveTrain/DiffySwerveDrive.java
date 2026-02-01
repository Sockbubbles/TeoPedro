package org.firstinspires.ftc.teamcode.pedroPathing.DriveTrain;


import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;

public class DiffySwerveDrive extends Drivetrain {



    private final DiffySwerveConstants c;

    private final DcMotorEx tL, bL, tR, bR;
    private final List<DcMotorEx> motors;

    private final AnalogInput lEnc, rEnc;
    private final VoltageSensor voltageSensor;

    private double lRemember = 0, rRemember = 0;
    private double lInt = 0, rInt = 0;
    private double lLast = 0, rLast = 0;

    public DiffySwerveDrive(HardwareMap hw, DiffySwerveConstants constants) {
        this.c = constants;

        maxPowerScaling = c.maxPower;

        tL = hw.get(DcMotorEx.class, c.tLMotor);
        bL = hw.get(DcMotorEx.class, c.bLMotor);
        tR = hw.get(DcMotorEx.class, c.tRMotor);
        bR = hw.get(DcMotorEx.class, c.bRMotor);

        motors = Arrays.asList(tL, bL, tR, bR);

        lEnc = hw.get(AnalogInput.class, c.leftEncoder);
        rEnc = hw.get(AnalogInput.class, c.rightEncoder);

        voltageSensor = hw.voltageSensor.iterator().next();

        tL.setDirection(c.tLDir);
        bL.setDirection(c.bLDir);
        tR.setDirection(c.tRDir);
        bR.setDirection(c.bRDir);

        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public double[] calculateDrive(Vector corrective, Vector heading, Vector pathing, double robotHeading) {

        Vector[] side = new Vector[2];

        Vector left = corrective.minus(heading);
        Vector right = corrective.plus(heading);

        side[0] = left.plus(pathing);
        side[1] = right.plus(pathing);

        side[0] = side[0].times(2.0);
        side[1] = side[1].times(2.0);

        Vector lVec = side[0].copy();
        Vector rVec = side[1].copy();

        lVec.rotateVector(-robotHeading);
        rVec.rotateVector(-robotHeading);

        double lAngle = Math.atan2(lVec.getYComponent(), lVec.getXComponent());
        double rAngle = Math.atan2(rVec.getYComponent(), rVec.getXComponent());

        double lSpeed = lVec.getMagnitude();
        double rSpeed = rVec.getMagnitude();

        if (lSpeed > c.speedDeadband) lRemember = lAngle;
        if (rSpeed > c.speedDeadband) rRemember = rAngle;

        double lCurr = encoderToRad(lEnc.getVoltage(), c.leftOffsetDeg);
        double rCurr = encoderToRad(rEnc.getVoltage(), c.rightOffsetDeg);

        double lErr = AngleUnit.normalizeRadians(lRemember - lCurr);
        double rErr = AngleUnit.normalizeRadians(rRemember - rCurr);

        if (Math.abs(lErr) > c.flipThresholdRad) {
            lErr -= Math.signum(lErr) * Math.PI;
            lSpeed *= -1;
        }

        if (Math.abs(rErr) > c.flipThresholdRad) {
            rErr -= Math.signum(rErr) * Math.PI;
            rSpeed *= -1;
        }

        double lOut = pid(lErr, true);
        double rOut = pid(rErr, false);

        double tLP = lSpeed + lOut;
        double bLP = -lSpeed + lOut;
        double tRP = rSpeed + rOut;
        double bRP = -rSpeed + rOut;

        return normalize(new double[]{tLP, bLP, tRP, bRP});
    }

    @Override
    public void runDrive(double[] p) {
        tL.setPower(p[0]);
        bL.setPower(p[1]);
        tR.setPower(p[2]);
        bR.setPower(p[3]);
    }

    private double pid(double err, boolean left) {
        if (left) {
            lInt += err;
            double out = c.kP * err + c.kI * lInt + c.kD * (err - lLast);
            lLast = err;
            return out;
        } else {
            rInt += err;
            double out = c.kP * err + c.kI * rInt + c.kD * (err - rLast);
            rLast = err;
            return out;
        }
    }

    private static double encoderToRad(double v, double offsetDeg) {
        return AngleUnit.normalizeRadians((v / 3.3) * (2 * Math.PI) - Math.toRadians(offsetDeg));
    }

    private static double[] normalize(double[] p) {
        double max = 0;
        for (double v : p) max = Math.max(max, Math.abs(v));
        if (max > 1.0) for (int i = 0; i < p.length; i++) p[i] /= max;
        return p;
    }

    @Override public double xVelocity() { return 0; }
    @Override public double yVelocity() { return 0; }
    @Override public void setXVelocity(double x) {}
    @Override public void setYVelocity(double y) {}
    @Override public double getVoltage() { return voltageSensor.getVoltage(); }
    @Override public void updateConstants() {}

    @Override
    public void breakFollowing() {
        tL.setPower(0);
        bL.setPower(0);
        tR.setPower(0);
        bR.setPower(0);
    }

    @Override
    public void startTeleopDrive() {
        for (DcMotorEx m : motors) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        DcMotor.ZeroPowerBehavior zpb = brakeMode
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        for (DcMotorEx m : motors) m.setZeroPowerBehavior(zpb);
    }

    @Override public String debugString() { return "DiffySwerveDrive";}
}
