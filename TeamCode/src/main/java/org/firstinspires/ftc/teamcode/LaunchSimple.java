package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Launch-simple", group="Robot")
public class LaunchSimple extends LinearOpMode {

    HardwareIntake robot = new HardwareIntake(this);

    @Override
    public void runOpMode() {
        robot.init();

        telemetry.addData("Status", "Initialized. Ready for Launch-simple.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Second 1: Start pollenlauncher (spin up)
            if (robot.pollenlauncher != null) robot.pollenlauncher.setPower(0.85);
            telemetry.addData("Status", "Second 1: Spinning up pollenlauncher...");
            telemetry.update();
            sleep(1000);

            // Seconds 2 - 5: Start mover; run both pollenlauncher and mover
            if (robot.mover != null) robot.mover.setPower(0.85);
            telemetry.addData("Status", "Seconds 2-5: Running pollenlauncher & mover...");
            telemetry.update();
            sleep(4000);

            // Second 6: Stop pollenlauncher and mover
            if (robot.pollenlauncher != null) robot.pollenlauncher.setPower(0.0);
            if (robot.mover != null) robot.mover.setPower(0.0);
            telemetry.addData("Status", "Second 6: Stopped pollenlauncher & mover.");
            telemetry.update();
            sleep(1000);

            // Second 7: Move robot forward
            setDrivePower(0.5, 0.5, 0.5, 0.5);
            telemetry.addData("Status", "Second 7: Moving robot forward...");
            telemetry.update();
            sleep(1000);

            // Second 8: Stop robot
            setDrivePower(0, 0, 0, 0);
            telemetry.addData("Status", "Second 8: Robot stopped. Complete!");
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
