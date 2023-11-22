import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@TeleOp
public class MotorWithJoystickControlOpMode extends LinearOpMode {
    private DcMotor motor;
    private int minPosition = 0; // Minimum allowed position (encoder ticks)
    private int maxPosition = 1000; // Maximum allowed position (encoder ticks)

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "test"); // Replace with your motor name

        // Set motor direction (forward or reverse)
        motor.setDirection(DcMotorSimple.Direction.FORWARD); // Adjust as needed

        waitForStart();

        while (opModeIsActive()) {
            double joystickValue = gamepad1.right_stick_y; // Assuming you're using the left joystick for control

            // Map the joystick value to the soft limits
            int targetPosition = (int) (joystickValue * (maxPosition - minPosition) / 2 + (maxPosition + minPosition) / 2);

            // Ensure the target position stays within the soft limits
            if (targetPosition < minPosition) {
                targetPosition = minPosition;
            } else if (targetPosition > maxPosition) {
                targetPosition = maxPosition;
            }

            motor.setTargetPosition(targetPosition);
            motor.setPower(0.5); // Apply power to the motor
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Your code here...

            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.update();
        }
    }
}
