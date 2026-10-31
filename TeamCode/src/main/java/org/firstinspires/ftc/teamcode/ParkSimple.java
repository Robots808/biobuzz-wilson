package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Park-simple", group="Robot")
public class ParkSimple extends LinearOpMode {

    HardwareIntake robot = new HardwareIntake(this);

    @Override
    public void runOpMode() {
        robot.init();

        telemetry.addData("Status", "Initialized. Ready for Park-simple.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Drive forward at 50% power for 1 second (1000 ms)
            setDrivePower(0.5, 0.5, 0.5, 0.5);
            telemetry.addData("Status", "Driving Forward...");
            telemetry.update();

            sleep(1000);

            // Stop all motors
            setDrivePower(0, 0, 0, 0);
            telemetry.addData("Status", "Parked!");
            telemetry.update();
        }
    }

    private void setDrivePower(double lf, double lb, double rf, double rb) {
        if (robot.leftFront != null) robot.leftFront.setPower(lf);
        if (robot.leftBack != null) robot.leftBack.setPower(lb);
        if (robot.rightFront != null) robot.rightFront.setPower(rf);
        if (robot.rightBack != null) robot.rightBack.setPower(rb);
    }
}
