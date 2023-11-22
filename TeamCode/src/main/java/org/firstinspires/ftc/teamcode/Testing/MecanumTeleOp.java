package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("Leftfront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("Leftback");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("Rightfront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("Rightback");
        //DcMotor verticalExtension = hardwareMap.dcMotor.get("test");

        //Servo Servotest = hardwareMap.servo.get("Testservo");
        Servo Servo1 = hardwareMap.servo.get("1");
        Servo Servo2 = hardwareMap.servo.get("2");

        //int z = 0;

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double extend = gamepad1.right_stick_y;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double verticalExtensionPower = (extend);

            if (gamepad1.left_trigger>0.1){
                frontLeftMotor.setPower(-gamepad1.left_trigger);
                frontRightMotor.setPower(gamepad1.left_trigger);
                backLeftMotor.setPower(gamepad1.left_trigger);
                backRightMotor.setPower(-gamepad1.left_trigger);
            }
            else if (gamepad1.right_trigger>0.1){
                frontLeftMotor.setPower(gamepad1.right_trigger);
                frontRightMotor.setPower(-gamepad1.right_trigger);
                backLeftMotor.setPower(-gamepad1.right_trigger);
                backRightMotor.setPower(gamepad1.right_trigger);
            }
            else{
//                frontLeftMotor.setPower(-gamepad1.left_stick_y);
//                frontRightMotor.setPower(-gamepad1.right_stick_y);
//                backLeftMotor.setPower(-gamepad1.left_stick_y);
//                backRightMotor.setPower(-gamepad1.right_stick_y);
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                frontLeftMotor.setPower(Range.clip(drive+turn,-1.0,1.0));
                backLeftMotor.setPower(Range.clip(drive+turn,-1.0,1.0));
                frontRightMotor.setPower(Range.clip(drive-turn,-1.0,1.0));
                backRightMotor.setPower(Range.clip(drive-turn,-1.0,1.0));
            }

            /*frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            verticalExtension.setPower(verticalExtensionPower);*/

//            if(gamepad1.x) {
//                double servopower = 0.5;
//            } if(gamepad1.b) {
//                double servopower = 0;
//            } else {
//                double servopower = 0.25;
//            };
            if(gamepad1.a) {
                Servo1.setPosition(0.55);
                Servo2.setPosition(0.05);
            }
//            else if (gamepad1.b) {
//                Servo1.setPosition(0.3);
//                Servo2.setPosition(0.3);
//            }
            else {
                Servo1.setPosition(0.3);
                Servo2.setPosition(0.3);
            }


                // Control the servo position
                //Servotest.setPosition(0.5); // Set servo position to 0.5 (range: 0.0 to 1.0)
                // You can use gamepad inputs or other logic to control the servo position

                //telemetry.addData("Servo Position", Servotest.getPosition());
                telemetry.addData("Left Stick X", y);
                telemetry.addData("Left Stick Y", x);
                telemetry.addData("Right Stick X", rx);
                telemetry.addData("Right Stick Y", extend);
                telemetry.update();

        }
    }
}