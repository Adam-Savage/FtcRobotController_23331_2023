package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class VertExtensionPositionControl extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get("test");

        int minPosition = 0;
        int maxPosition = 500;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(0);
        motor.setPower(0);
        //motor.setTargetPositionTolerance(10);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //double extend = gamepad1.right_stick_y;
            //double verticalExtensionPower = (extend);
            //verticalExtension.setPower(verticalExtensionPower);
            if (gamepad1.x) {
                motor.setTargetPosition(500);
                motor.setPower(0.5);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            };
            if (gamepad1.y) {
                motor.setTargetPosition(0);
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            };




            //telemetry.addData("Right Stick Y", extend);
            telemetry.update();
        }
    }

}
