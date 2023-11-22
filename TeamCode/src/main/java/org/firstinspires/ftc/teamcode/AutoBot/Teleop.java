package org.firstinspires.ftc.teamcode.AutoBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //Motor Declaration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("Leftfront");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("Rightfront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("Leftback");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("Rightback");

        //Motor Reverse
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Motor Caculations
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            frontLeftMotor.setPower(Range.clip(drive+turn,-1.0,1.0));
            backLeftMotor.setPower(Range.clip(drive+turn,-1.0,1.0));
            frontRightMotor.setPower(Range.clip(drive-turn,-1.0,1.0));
            backRightMotor.setPower(Range.clip(drive-turn,-1.0,1.0));

            //Telemetry Update
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}
