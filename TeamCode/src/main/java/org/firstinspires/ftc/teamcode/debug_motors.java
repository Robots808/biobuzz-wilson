package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="debug-motors", group="Robot")
public class debug_motors extends LinearOpMode {

    HardwareIntake robot = new HardwareIntake(this);

    @Override
    public void runOpMode() {
        robot.init();

        telemetry.addLine("Debug Motors Ready.");
        telemetry.addLine("Press & hold buttons to test individual motors:");
        telemetry.addLine("  - Square (X): leftFront");
        telemetry.addLine("  - X (A): leftBack");
        telemetry.addLine("  - Triangle (Y): rightFront");
        telemetry.addLine("  - Circle (B): rightBack");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double rfPower = (gamepad1.y || gamepad1.triangle) ? 0.5 : 0.0;
            double lfPower = (gamepad1.x || gamepad1.square) ? 0.5 : 0.0;
            double rbPower = (gamepad1.b || gamepad1.circle) ? 0.5 : 0.0;
            double lbPower = (gamepad1.a || gamepad1.cross) ? 0.5 : 0.0;

            if (robot.rightFront != null) robot.rightFront.setPower(rfPower);
            if (robot.leftFront != null) robot.leftFront.setPower(lfPower);
            if (robot.rightBack != null) robot.rightBack.setPower(rbPower);
            if (robot.leftBack != null) robot.leftBack.setPower(lbPower);

            telemetry.addData("Testing Motor", "Power");
            telemetry.addData("leftFront (Square)", "%.2f", lfPower);
            telemetry.addData("leftBack (X)", "%.2f", lbPower);
            telemetry.addData("rightFront (Triangle)", "%.2f", rfPower);
            telemetry.addData("rightBack (Circle)", "%.2f", rbPower);
            telemetry.update();

            sleep(10);
        }
    }
}
