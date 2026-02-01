package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.DriveTrain.DiffySwerveConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.DriveTrain.DiffySwerveDrive;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5)
            .forwardZeroPowerAcceleration(0)
            .lateralZeroPowerAcceleration(0)
            .centripetalScaling(0.005)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.01, 0.0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.01, 0.0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0.0, 0.01, 0.6, 0.0));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static DiffySwerveConstants driveConstants = new DiffySwerveConstants();

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setDrivetrain(new DiffySwerveDrive(hardwareMap, driveConstants))
                .build();
    }
}
