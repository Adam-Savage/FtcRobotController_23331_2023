package org.firstinspires.ftc.teamcode.Season.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp
public class L_Drive_FieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

//---------------------------------------------------------------------------

        //Motor Declaration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("Leftfront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("Leftback");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("Rightfront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("Rightback");

        //Motor Reverse
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //IMU Declaration
        IMU imu = hardwareMap.get(IMU.class, "imu");
        //Parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        //Driving Variables
        double LeftStickY;
        double LeftStickX;
        double RX;

//---------------------------------------------------------------------------

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

//---------------------------------------------------------------------------

            //Drive Control
            //Slow Driving
            if (gamepad1.left_bumper){
                LeftStickY = -gamepad1.left_stick_y * 0.3;
                LeftStickX = gamepad1.left_stick_x * 0.3;
                RX = gamepad1.right_stick_x * 0.35;
            }
            else {
                LeftStickY = -gamepad1.left_stick_y;
                LeftStickX = gamepad1.left_stick_x;
                RX = gamepad1.right_stick_x * 0.8;
            }

//            //Mecanum Driving with Triggers
//            if (gamepad1.left_trigger>0.1){
//                frontLeftMotor.setPower(-gamepad1.left_trigger);
//                frontRightMotor.setPower(gamepad1.left_trigger);
//                backLeftMotor.setPower(gamepad1.left_trigger);
//                backRightMotor.setPower(-gamepad1.left_trigger);
//            }
//            else if (gamepad1.right_trigger>0.1){
//                frontLeftMotor.setPower(gamepad1.right_trigger);
//                frontRightMotor.setPower(-gamepad1.right_trigger);
//                backLeftMotor.setPower(-gamepad1.right_trigger);
//                backRightMotor.setPower(gamepad1.right_trigger);
//            }
//            else{
//                double drive = LeftStickY;
//                double turn = LeftStickX;
//                frontLeftMotor.setPower(Range.clip(drive+turn,-1.0,1.0));
//                backLeftMotor.setPower(Range.clip(drive+turn,-1.0,1.0));
//                frontRightMotor.setPower(Range.clip(drive-turn,-1.0,1.0));
//                backRightMotor.setPower(Range.clip(drive-turn,-1.0,1.0));
//            }

            //Field Centric
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = LeftStickX * Math.cos(-botHeading) - LeftStickY * Math.sin(-botHeading);
            double rotY = LeftStickX * Math.sin(-botHeading) + LeftStickY * Math.cos(-botHeading);

//            rotX = rotX * 1.1

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(RX), 1);
            double frontLeftPower = (rotY + rotX + RX) / denominator;
            double backLeftPower = (rotY - rotX + RX) / denominator;
            double frontRightPower = (rotY - rotX - RX) / denominator;
            double backRightPower = (rotY + rotX - RX) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

//---------------------------------------------------------------------------

            //Telemetry Update
            //Drive Information
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Strafe Left", gamepad1.left_trigger);
            telemetry.addData("Strafe Right", gamepad1.right_trigger);
            //Update
            telemetry.update();
            }
        }
    }

