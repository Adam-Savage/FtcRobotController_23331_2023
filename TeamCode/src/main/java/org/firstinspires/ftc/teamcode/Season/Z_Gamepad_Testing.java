package org.firstinspires.ftc.teamcode.Season;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Z_Gamepad_Testing extends OpMode {

    @Override
    public void init() {
        //Code to run ONCE when the driver hits INIT

        //Send telemetry to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    @Override
    public void loop() {

        //Telemetry Update
        telemetry.addData("y", gamepad1.x);
        telemetry.addData("x", gamepad1.y);
        telemetry.addData("b", gamepad1.b);
        telemetry.addData("a", gamepad1.a);
        telemetry.addData("dpad_down", gamepad1.dpad_down);
        telemetry.addData("dpad_left", gamepad1.dpad_left);
        telemetry.addData("dpad_right", gamepad1.dpad_right);
        telemetry.addData("dpad_up", gamepad1.dpad_up);
        telemetry.addData("left_trigger", gamepad1.left_trigger);
        telemetry.addData("right_trigger", gamepad1.right_trigger);
        telemetry.addData("left_bumper", gamepad1.left_bumper);
        telemetry.addData("right_bumper", gamepad1.right_bumper);
        telemetry.addData("back", gamepad1.back);
        telemetry.addData("start", gamepad1.start);
        telemetry.addData("guide", gamepad1.guide);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("left_stick_button", gamepad1.left_stick_button);
        telemetry.addData("right_stick_button", gamepad1.right_stick_button);

    }
}
