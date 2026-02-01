package org.firstinspires.ftc.teamcode.OpMode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;

@TeleOp(name = "TeleOpDiffy", group = "test")
public class TeleOpDiffy extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false,Math.toRadians(180) // 0 pour RED side
        );

        follower.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}

