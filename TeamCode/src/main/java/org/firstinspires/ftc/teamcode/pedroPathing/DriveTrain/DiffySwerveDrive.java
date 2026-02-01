package org.firstinspires.ftc.teamcode.pedroPathing.DriveTrain;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.Drivetrain;
import com.pedropathing.control.LowPassFilter;
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

    private final LowPassFilter lAngleLPF;
    private final LowPassFilter rAngleLPF;

    private double lAngleFiltered = 0.0;
    private double rAngleFiltered = 0.0;
    private boolean lAngleInit = false;
    private boolean rAngleInit = false;

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

        for (DcMotorEx m : motors) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lAngleLPF = new LowPassFilter(c.angleLPFAlpha);
        rAngleLPF = new LowPassFilter(c.angleLPFAlpha);
    }

    @Override
    public double[] calculateDrive(Vector corrective, Vector heading, Vector pathing, double robotHeading) {

        if (corrective.getMagnitude() > maxPowerScaling) corrective.setMagnitude(maxPowerScaling);
        if (heading.getMagnitude() > maxPowerScaling) heading.setMagnitude(maxPowerScaling);
        if (pathing.getMagnitude() > maxPowerScaling) pathing.setMagnitude(maxPowerScaling);

        Vector[] side = new Vector[2];

        if (corrective.getMagnitude() == maxPowerScaling) {
            side[0] = corrective.copy();
            side[1] = corrective.copy();
        } else {
            Vector leftSide = corrective.minus(heading);
            Vector rightSide = corrective.plus(heading);

            if (leftSide.getMagnitude() > maxPowerScaling || rightSide.getMagnitude() > maxPowerScaling) {
                double headingScaling = Math.min(
                        findNormalizingScaling(corrective, heading, maxPowerScaling),
                        findNormalizingScaling(corrective, heading.times(-1), maxPowerScaling)
                );
                side[0] = corrective.minus(heading.times(headingScaling));
                side[1] = corrective.plus(heading.times(headingScaling));
            } else {
                Vector leftWithPath = leftSide.plus(pathing);
                Vector rightWithPath = rightSide.plus(pathing);

                if (leftWithPath.getMagnitude() > maxPowerScaling || rightWithPath.getMagnitude() > maxPowerScaling) {
                    double pathScaling = Math.min(
                            findNormalizingScaling(leftSide, pathing, maxPowerScaling),
                            findNormalizingScaling(rightSide, pathing, maxPowerScaling)
                    );
                    side[0] = leftSide.plus(pathing.times(pathScaling));
                    side[1] = rightSide.plus(pathing.times(pathScaling));
                } else {
                    side[0] = leftWithPath.copy();
                    side[1] = rightWithPath.copy();
                }
            }
        }

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

        double lRaw = encoderToRad(lEnc.getVoltage(), c.leftOffsetDeg);
        double rRaw = encoderToRad(rEnc.getVoltage(), c.rightOffsetDeg);

        if (!lAngleInit) {
            lAngleFiltered = lRaw;
            lAngleLPF.reset(0.0, 0.0, 0.0);
            lAngleInit = true;
        } else {
            double d = AngleUnit.normalizeRadians(lRaw - lAngleFiltered);
            lAngleLPF.update(d, 0.0);
            lAngleFiltered = AngleUnit.normalizeRadians(lAngleFiltered + lAngleLPF.getState());
        }

        if (!rAngleInit) {
            rAngleFiltered = rRaw;
            rAngleLPF.reset(0.0, 0.0, 0.0);
            rAngleInit = true;
        } else {
            double d = AngleUnit.normalizeRadians(rRaw - rAngleFiltered);
            rAngleLPF.update(d, 0.0);
            rAngleFiltered = AngleUnit.normalizeRadians(rAngleFiltered + rAngleLPF.getState());
        }

        double lCurr = lAngleFiltered;
        double rCurr = rAngleFiltered;

        double lErr = AngleUnit.normalizeRadians(lRemember - lCurr);
        double rErr = AngleUnit.normalizeRadians(rRemember - rCurr);

        if (Math.abs(lErr) > c.flipThresholdRad) {
            lErr -= Math.signum(lErr) * Math.PI;
            lErr = AngleUnit.normalizeRadians(lErr);
            lSpeed *= -1.0;
        }

        if (Math.abs(rErr) > c.flipThresholdRad) {
            rErr -= Math.signum(rErr) * Math.PI;
            rErr = AngleUnit.normalizeRadians(rErr);
            rSpeed *= -1.0;
        }

        double maxMag = Math.max(Math.abs(lSpeed), Math.abs(rSpeed));
        if (maxMag > c.maxSpeed && maxMag > 1e-9) {
            lSpeed = (lSpeed / maxMag) * c.maxSpeed;
            rSpeed = (rSpeed / maxMag) * c.maxSpeed;
        }

        double lOut = pid(lErr, true);
        double rOut = pid(rErr, false);

        double tLP = lSpeed + lOut;
        double bLP = -lSpeed + lOut;
        double tRP = rSpeed + rOut;
        double bRP = -rSpeed + rOut;

        return normalizeToMaxPower(new double[]{tLP, bLP, tRP, bRP}, maxPowerScaling);
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
            double out = c.kP * err + c.kI * lInt + c.kD * (err - lLast) + c.kF * Math.signum(err);
            lLast = err;
            return clamp(out);
        } else {
            rInt += err;
            double out = c.kP * err + c.kI * rInt + c.kD * (err - rLast) + c.kF * Math.signum(err);
            rLast = err;
            return clamp(out);
        }
    }

    private static double clamp(double x) {
        if (x > 1.0) return 1.0;
        if (x < -1.0) return -1.0;
        return x;
    }

    private static double encoderToRad(double v, double offsetDeg) {
        return AngleUnit.normalizeRadians((v / 3.3) * (2 * Math.PI) - Math.toRadians(offsetDeg));
    }

    private static double[] normalizeToMaxPower(double[] p, double maxPower) {
        double max = 0.0;
        for (double v : p) max = Math.max(max, Math.abs(v));
        if (max > maxPower && max > 1e-9) {
            double s = maxPower / max;
            for (int i = 0; i < p.length; i++) p[i] *= s;
        }
        return p;
    }

    @Override public double xVelocity() { return c.xVelocity; }
    @Override public double yVelocity() { return c.yVelocity; }
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
        lAngleInit = false;
        rAngleInit = false;
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

    @Override
    public String debugString() {
        return "DiffySwerveDrive";
    }
}
