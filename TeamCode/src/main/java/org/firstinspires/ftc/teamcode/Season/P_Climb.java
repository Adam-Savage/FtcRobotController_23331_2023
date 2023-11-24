package org.firstinspires.ftc.teamcode.Season;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

//@Disabled
@TeleOp
public class P_Climb extends LinearOpMode {

    //Set Speed
    static final double ClimbSpeedUp = -0.5;
    static final double ClimbSpeedDown = 1;

    //Set Points
    int ClimbSetPtUp = -1000;
    int ClimbSetPtDown = -100;

    @Override
    public void runOpMode() throws InterruptedException {

        //Motor Declaration
        DcMotor Climb = hardwareMap.dcMotor.get("Climb");

        //Encoder Mode
        Climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Enable Break
        Climb.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        //Verify Robot Waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Climb Control
            if (gamepad1.dpad_up)
                Climb.setPower(ClimbSpeedUp);
            else if (gamepad1.dpad_down)
                Climb.setPower(ClimbSpeedDown);
            else
                Climb.setPower(0.0);

            //Climb Soft Limits
//            if (currentClimbPosition < minClimbEncoderCount) {
//                Climb.setPower(0.0);
//                Climb.setTargetPosition(minClimbEncoderCount);
//                Climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else if (currentClimbPosition > maxClimbEncoderCount) {
//                Climb.setPower(0.0);
//                Climb.setTargetPosition(maxClimbEncoderCount);
//                Climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }

//            //Climb Set Points
//            if (gamepad1.dpad_up) {
//                Climb.setTargetPosition(ClimbSetPtDown);
//                Climb.setPower(ClimbSpeedUp);
//            } else if (gamepad1.dpad_down) {
//                Climb.setTargetPosition(ClimbSetPtUp);
//                Climb.setPower(ClimbSpeedDown);
//            }

            //Telemetry Update
            telemetry.addData("Clumb Position", Climb.getCurrentPosition());
            telemetry.addData("Climb", gamepad1.dpad_up ? "Up" : "Down");
            telemetry.update();
        }
    }
}
