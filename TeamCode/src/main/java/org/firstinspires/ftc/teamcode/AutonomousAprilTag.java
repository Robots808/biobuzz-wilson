package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@Autonomous(name="Auto: Drive to AprilTag 24", group="Robot")
public class AutonomousAprilTag extends LinearOpMode {

    HardwareIntake robot = new HardwareIntake(this);

    // Tune these values
    final double TARGET_DISTANCE = 50.0; // Inches
    final double SPEED_GAIN = 0.04;      
    final double STRAFE_GAIN = 0.04;    
    final double TURN_GAIN = 0.03;      
    final double MAX_AUTO_SPEED = 0.5;   

    @Override
    public void runOpMode() {
        robot.init();

        if (robot.limelight != null) {
            robot.limelight.pipelineSwitch(0);
            robot.limelight.start();
        }

        telemetry.addData("Status", "Initialized. Looking for Tag 24");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean tag24Found = false;
            double drive = 0;
            double strafe = 0;
            double turn = 0;

            if (robot.limelight != null) {
                LLResult result = robot.limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult tag : fiducials) {
                        if (tag.getFiducialId() == 24) {
                            tag24Found = true;
                            Pose3D tagPose = tag.getTargetPoseRobotSpace();
                            
                            double lateralError = tagPose.getPosition().toUnit(DistanceUnit.INCH).y;
                            double distanceError = tagPose.getPosition().toUnit(DistanceUnit.INCH).x - TARGET_DISTANCE;
                            double yawError = tagPose.getOrientation().getYaw(AngleUnit.DEGREES);

                            drive = distanceError * SPEED_GAIN;
                            strafe = lateralError * STRAFE_GAIN;
                            turn = 0; // Turning removed
                            break;
                        }
                    }
                }
            }

            if (tag24Found) {
                // If we are within 1 inch and 2 degrees, stop
                if (Math.abs(drive) < 0.05 && Math.abs(strafe) < 0.05 && Math.abs(turn) < 0.05) {
                    driveRobot(0, 0, 0);
                    telemetry.addLine("LOCKED ON!");
                } else {
                    drive  = Math.min(Math.max(drive, -MAX_AUTO_SPEED), MAX_AUTO_SPEED);
                    strafe = Math.min(Math.max(strafe, -MAX_AUTO_SPEED), MAX_AUTO_SPEED);
                    turn   = Math.min(Math.max(turn, -MAX_AUTO_SPEED), MAX_AUTO_SPEED);
                    driveRobot(drive, strafe, turn);
                }
            } else {
                driveRobot(0, 0, 0);
                telemetry.addLine("Searching for Tag 24...");
            }

            telemetry.addData("Tag 24 Found", tag24Found);
            telemetry.update();
        }
        
        if (robot.limelight != null) robot.limelight.stop();
    }

    public void driveRobot(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        robot.leftFront.setPower((y + x + rx) / denominator);
        robot.leftBack.setPower((y - x + rx) / denominator);
        robot.rightFront.setPower((y - x - rx) / denominator);
        robot.rightBack.setPower((y + x - rx) / denominator);
    }
}
