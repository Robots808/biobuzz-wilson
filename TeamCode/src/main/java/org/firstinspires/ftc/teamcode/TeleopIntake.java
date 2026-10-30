package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@TeleOp(name="Intake", group="Robot")
public class TeleopIntake extends LinearOpMode {

    // Limelight mounting constants (in inches)
    final double TARGET_HEIGHT = 28.0;
    final double CAMERA_HEIGHT = 5.0;
    // Updated MOUNTING_ANGLE to match Web UI 13.5
    final double MOUNTING_ANGLE = 13.5; 

    /* 
     * IMPORTANT CALIBRATION:
     * 1. Go to Limelight Web UI (http://10.te.am.ip:5801)
     * 2. Set 'Mounting Pitch' to 13.5 (Updated to fix scaling)
     * 3. Set 'Camera Transform' X, Y, Z to the camera's position relative to the center of the robot.
     *    Without the Y-offset, the robot will have lateral errors when starting at an angle.
     */

    HardwareIntake robot = new HardwareIntake(this);
    boolean alignDone = false; // Tracks if the current alignment is finished

    @Override
    public void runOpMode() {
        robot.init();

        while (!isStarted() && !isStopRequested()) {
            if (robot.imu == null) {
                telemetry.addLine("!!! NO IMU FOUND !!!");
            } else {
                telemetry.addLine("IMU Found! Ready to start.");
            }
            telemetry.update();
        }

        waitForStart();
        if (robot.imu != null) robot.imu.resetYaw();
        if (robot.limelight != null) robot.limelight.start();

        while (opModeIsActive()) {
            // Manual Controls with small deadzone to ensure immediate stop
            double gamepadY = (Math.abs(gamepad1.left_stick_y) > 0.05) ? -gamepad1.left_stick_y : 0;
            double gamepadX = (Math.abs(gamepad1.left_stick_x) > 0.05) ? gamepad1.left_stick_x : 0;
            double gamepadRX = (Math.abs(gamepad1.right_stick_x) > 0.05) ? gamepad1.right_stick_x : 0;
            
            double x = gamepadX;
            double y = gamepadY;
            double rx = gamepadRX;

            // Auto-Align to ID 24 (Hold button X)
            boolean isAutoAlign = false;
            if (!gamepad1.x) {
                alignDone = false; // Reset lock when button is released
            }

            if (gamepad1.x && !alignDone && robot.limelight != null) {
                LLResult result = robot.limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult tag : fiducials) {
                        if (tag.getFiducialId() == 24) {
                            isAutoAlign = true;
                            Pose3D tagPose = tag.getTargetPoseRobotSpace();
                            
                            // Distance Estimation using "Classic" Trig
                            double ty = result.getTy();
                            // Adding a 5.0 offset to MOUNTING_ANGLE based on 145 vs 84 error
                            double angleToGoalRadians = Math.toRadians(MOUNTING_ANGLE + ty);
                            double xDist = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
                            
                            // Clamp distance to avoid crazy values if tag is lost
                            if (xDist < 0) xDist = 0;
                            if (xDist > 200) xDist = 200;
                            
                            double yawErr = tagPose.getOrientation().getYaw(AngleUnit.DEGREES);
                            double tx = result.getTx(); // Raw horizontal angle from crosshair

                            // CENTERING TEST: Set target to 0.0 to verify signs from both tiles
                            double txTarget = 0.0;
                            double txErr = tx - txTarget;

                            // P-Controller Gains
                            double driveGain = 0.025;
                            double strafeGain = 0.02;  // Increased for decisive movement
                            double turnGain = 0.015;   
                            double minPower = 0.05;    // Balanced floor for GoBILDA

                            // Distance (Target 42.5 inches to reach actual 50)
                            double distErr = xDist - 42.5;
                            // FLIPPED SIGN: Positive distErr (too far) needs Negative Y to move forward
                            y = -distErr * driveGain;
                            
                            // Lateral (Centering Test)
                            // FLIPPED SIGN: Negative txErr (Tag Left) needs Positive power to move Left
                            x = -txErr * strafeGain;
                            
                            // Rotation (Face tag parallel) using 3D Yaw
                            rx = -yawErr * turnGain;
                            
                            // "Straighten First" Logic
                            double alignScale = Math.max(0.4, 1.0 - (Math.abs(yawErr) / 20.0));
                            x *= alignScale;
                            y *= alignScale;

                            // Apply min power only if outside tolerance (Deadzone Logic)
                            if (Math.abs(distErr) > 0.5) {
                                y += Math.signum(-distErr) * minPower;
                            } else {
                                y = 0;
                            }
                            
                            if (Math.abs(txErr) > 1.5) {
                                x += Math.signum(-txErr) * minPower;
                            } else {
                                x = 0;
                            }

                            if (Math.abs(yawErr) > 1.5) {
                                rx += Math.signum(-yawErr) * minPower;
                            } else {
                                rx = 0;
                            }
                            
                            // Check if alignment is complete
                            if (Math.abs(distErr) < 0.5 && Math.abs(txErr) < 1.5 && Math.abs(yawErr) < 1.5) {
                                alignDone = true;
                                x = 0; y = 0; rx = 0;
                            }

                            // Safety speed limit
                            double limit = 0.45;
                            y = Math.max(-limit, Math.min(limit, y));
                            x = Math.max(-limit, Math.min(limit, x));
                            rx = Math.max(-limit, Math.min(limit, rx));
                            
                            telemetry.addData("Auto Align", "ACTIVE on ID 24");
                            telemetry.addData("Forward Dist", "%.1f in", xDist);
                            telemetry.addData("Dist Err", "%.1f in", distErr);
                            telemetry.addData("TX (Horiz Angle)", "%.1f Deg", tx);
                            telemetry.addData("Yaw Err", "%.1f Deg", yawErr);
                            telemetry.addData("X Pwr", "%.2f", x);
                            telemetry.addData("Y Pwr", "%.2f", y);
                            break;
                        }
                    }
                }
            }

            double botHeading = 0;
            if (robot.imu != null) {
                botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                if (gamepad1.options) robot.imu.resetYaw();
            }

            double rotX, rotY;
            if (isAutoAlign) {
                rotX = x; rotY = y; // Auto-align values are robot-centric
            } else if (!gamepad1.dpad_up) {
                rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            } else {
                rotX = x; rotY = y;
            }

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            robot.leftFront.setPower((rotY + rotX + rx) / denominator);
            robot.leftBack.setPower((rotY - rotX + rx) / denominator);
            robot.rightFront.setPower((rotY - rotX - rx) / denominator);
            robot.rightBack.setPower((rotY + rotX - rx) / denominator);

            // Intake Controls
            if (gamepad1.b) {
                robot.intake.setPower(-1.0);
            } else {
                robot.intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            }

            // Pollenlauncher Control (Button A: 85% power)
            if (robot.pollenlauncher != null) {
                if (gamepad1.a) {
                    robot.pollenlauncher.setPower(0.85);
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

            telemetry.addData("Heading", "%.1f Deg", Math.toDegrees(botHeading));
            telemetry.addData("Limelight Angle", "%.2f Deg", MOUNTING_ANGLE);
            telemetry.update();
            sleep(10);
        }
        if (robot.limelight != null) robot.limelight.stop();
    }
}
