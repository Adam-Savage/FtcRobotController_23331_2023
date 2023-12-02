package org.firstinspires.ftc.teamcode.Season.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Disabled
@TeleOp
public class Z_LimitSwitchTest extends LinearOpMode {

    //Set Speed
    static final double LiftSpeed = -0.5;

    //Set Endpoints
    int maxLiftEncoderCount = 5000;
    int minLiftEncoderCount = 0;

    //Set Set points
    int LiftSetPtIntake = 100;
    int LiftSetPtLvl1 = 2000;
    int LiftSetPtLvl2 = 4000;

    @Override
    public void runOpMode() throws InterruptedException {

        //Motor Declaration
        DcMotor Lift = hardwareMap.dcMotor.get("Lift");

        //Sensor Declaration
        DigitalChannel limitSwitch = hardwareMap.digitalChannel.get("limit");

        //Encoder Mode
//        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Enable Break
        Lift.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Init Switch State
            boolean isLimitSwitchPressed = limitSwitch.getState();

            //Initialise Encoders
            int currentLiftPosition = Lift.getCurrentPosition();

            //Control Lift
//            if (gamepad1.right_stick_y > 0.2)
//                Lift.setPower(LiftSpeedUp);
//            else if (gamepad1.right_stick_y < 0.2)
//                Lift.setPower(LiftSpeedDown);
//            else
//                Lift.setPower(0.0);

//            double lifting = -gamepad1.right_stick_y;
//            Lift.setPower(lifting*LiftSpeed);

            //Switch Reset Encoder
            if (isLimitSwitchPressed) {
                Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //Lift Soft Limits
            if (currentLiftPosition < minLiftEncoderCount) {
                Lift.setPower(0.0);
                Lift.setTargetPosition(minLiftEncoderCount);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (currentLiftPosition > maxLiftEncoderCount) {
                Lift.setPower(0.0);
                Lift.setTargetPosition(maxLiftEncoderCount);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //Lift Set Points
            if (gamepad1.dpad_right) {
                Lift.setTargetPosition(LiftSetPtIntake);
                Lift.setPower(LiftSpeed);
            } else if (gamepad1.right_bumper) {
                Lift.setTargetPosition(LiftSetPtLvl1);
                Lift.setPower(LiftSpeed);
            } else if (gamepad1.left_bumper) {
                Lift.setTargetPosition(LiftSetPtLvl2);
                Lift.setPower(LiftSpeed);
            }

            //Telemetry Update
            telemetry.addData("Lift Power", gamepad1.right_stick_y);
            telemetry.addData("Lift Position", currentLiftPosition);
            telemetry.addData("Limit Switch", isLimitSwitchPressed);
            telemetry.update();

        }
    }
}

