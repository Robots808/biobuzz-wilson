package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="launcher_test", group="Robot")
public class launcher_test extends LinearOpMode {

    private DcMotor launcher = null;

    @Override
    public void runOpMode() {
        try {
            launcher = hardwareMap.get(DcMotor.class, "launcher");
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcher.setDirection(DcMotor.Direction.FORWARD);
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addLine("Launcher Motor Found");
        } catch (Exception e) {
            telemetry.addLine("!!! LAUNCHER MOTOR NOT FOUND !!! Check hardware config for 'launcher'");
        }

        telemetry.addData("Status", "Initialized. Hold X to fire launcher.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Check face buttons (X, A, Y, B) in case controller layout / mode maps physical X to another button
            boolean isFiring = gamepad1.x || gamepad1.a || gamepad1.y || gamepad1.b;

            if (launcher != null) {
                if (isFiring) {
                    launcher.setPower(1.0);
                    telemetry.addData("Launcher Status", "RUNNING (100%)");
                } else {
                    launcher.setPower(0.0);
                    telemetry.addData("Launcher Status", "STOPPED");
                }
            } else {
                telemetry.addLine("!!! LAUNCHER MOTOR NOT FOUND !!!");
            }

            // Live Gamepad Diagnostic Telemetry
            telemetry.addLine("--- Gamepad 1 Button Debug ---");
            telemetry.addData("Face Buttons", "X:%b | A:%b | Y:%b | B:%b", gamepad1.x, gamepad1.a, gamepad1.y, gamepad1.b);
            telemetry.addData("Bumpers/Triggers", "RB:%b | LB:%b | RT:%.2f | LT:%.2f", 
                    gamepad1.right_bumper, gamepad1.left_bumper, gamepad1.right_trigger, gamepad1.left_trigger);

            telemetry.update();
        }
    }
}
