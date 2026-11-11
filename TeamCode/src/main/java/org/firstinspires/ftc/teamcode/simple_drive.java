package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="simple-drive", group="Robot")
public class simple_drive extends LinearOpMode {

    HardwareIntake robot = new HardwareIntake(this);

    @Override
    public void runOpMode() {
        robot.init();

        telemetry.addLine("Robot-Centric Simple Drive Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Manual Controls (Robot-Centric)
            double rotY = (Math.abs(gamepad1.left_stick_y) > 0.05) ? -gamepad1.left_stick_y : 0;
            double rotX = (Math.abs(gamepad1.left_stick_x) > 0.05) ? gamepad1.left_stick_x : 0;
            double rx = (Math.abs(gamepad1.right_stick_x) > 0.05) ? gamepad1.right_stick_x : 0;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double lfPower = (rotY + rotX + rx) / denominator;
            double lbPower = (rotY - rotX + rx) / denominator;
            double rfPower = (rotY - rotX - rx) / denominator;
            double rbPower = (rotY + rotX - rx) / denominator;

            robot.leftFront.setPower(lfPower);
            robot.leftBack.setPower(lbPower);
            robot.rightFront.setPower(rfPower);
            robot.rightBack.setPower(rbPower);

            // Intake Controls
            if (gamepad1.b) {
                robot.intake.setPower(-1.0);
            } else {
                robot.intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            }

            // Pollenlauncher Control (Button A: 70% power)
            if (robot.pollenlauncher != null) {
                if (gamepad1.a) {
                    robot.pollenlauncher.setPower(0.54);
                } else {
                    robot.pollenlauncher.setPower(0.0);
                }
            }

            // Mover Control (Left Bumper: 85% power)
            if (robot.mover != null) {
                if (gamepad1.left_bumper) {
                    robot.mover.setPower(0.85);
                } else {
                    robot.mover.setPower(0.0);
                }
            }

            telemetry.addData("Drive Mode", "Robot-Centric (Simple Drive)");
            telemetry.addData("Motor Powers", "LF: %.2f | LB: %.2f | RF: %.2f | RB: %.2f", lfPower, lbPower, rfPower, rbPower);
            telemetry.addData("Stick Inputs", "rotX (Strafe): %.2f | rotY (Drive): %.2f | rx (Turn): %.2f", rotX, rotY, rx);
            telemetry.update();
            sleep(10);
        }
    }
}
