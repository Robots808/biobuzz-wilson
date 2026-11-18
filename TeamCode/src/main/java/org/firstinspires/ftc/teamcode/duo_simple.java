package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@TeleOp(name="duo-simple", group="Robot")
public class duo_simple extends LinearOpMode {

    // Limelight mounting constants (in inches)
    final double TARGET_HEIGHT = 28.0;
    final double CAMERA_HEIGHT = 5.0;
    final double MOUNTING_ANGLE = 13.5; 

    HardwareIntake robot = new HardwareIntake(this);
    boolean alignDone = false; // Tracks if the current alignment is finished

    @Override
    public void runOpMode() {
        robot.init();

        while (!isStarted() && !isStopRequested()) {
            if (robot.imu == null) {
                telemetry.addLine("!!! NO IMU FOUND !!!");
            } else {
                telemetry.addLine("IMU Found! Duo-Simple Mode Ready.");
            }
            telemetry.update();
        }

        waitForStart();
        if (robot.limelight != null) robot.limelight.start();

        while (opModeIsActive()) {
            // Gamepad 1: Manual Drive Controls (Robot-Centric)
            double gamepadY = (Math.abs(gamepad1.left_stick_y) > 0.05) ? -gamepad1.left_stick_y : 0;
            double gamepadX = (Math.abs(gamepad1.left_stick_x) > 0.05) ? gamepad1.left_stick_x : 0;
            double gamepadRX = (Math.abs(gamepad1.right_stick_x) > 0.05) ? gamepad1.right_stick_x : 0;
            
            double x = gamepadX;
            double y = gamepadY;
            double rx = gamepadRX;

            // Auto-Align to ID 24 (Hold button X on Gamepad 1)
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
                            
                            double ty = result.getTy();
                            double angleToGoalRadians = Math.toRadians(MOUNTING_ANGLE + ty);
                            double xDist = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
                            
                            if (xDist < 0) xDist = 0;
                            if (xDist > 200) xDist = 200;
                            
                            double yawErr = tagPose.getOrientation().getYaw(AngleUnit.DEGREES);
                            double tx = result.getTx();

                            double txTarget = 0.0;
                            double txErr = tx - txTarget;

                            double driveGain = 0.025;
                            double strafeGain = 0.02;  
                            double turnGain = 0.015;   
                            double minPower = 0.05;    

                            double distErr = xDist - 42.5;
                            y = -distErr * driveGain;
                            x = -txErr * strafeGain;
                            rx = -yawErr * turnGain;
                            
                            double alignScale = Math.max(0.4, 1.0 - (Math.abs(yawErr) / 20.0));
                            x *= alignScale;
                            y *= alignScale;

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
                            
                            if (Math.abs(distErr) < 0.5 && Math.abs(txErr) < 1.5 && Math.abs(yawErr) < 1.5) {
                                alignDone = true;
                                x = 0; y = 0; rx = 0;
                            }

                            double limit = 0.45;
                            y = Math.max(-limit, Math.min(limit, y));
                            x = Math.max(-limit, Math.min(limit, x));
                            rx = Math.max(-limit, Math.min(limit, rx));
                            
                            telemetry.addData("Auto Align", "ACTIVE on ID 24");
                            break;
                        }
                    }
                }
            }

            // Robot-Centric Drive directly (no IMU rotation)
            double rotX = x;
            double rotY = y;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            robot.leftFront.setPower((rotY + rotX + rx) / denominator);
            robot.leftBack.setPower((rotY - rotX + rx) / denominator);
            robot.rightFront.setPower((rotY - rotX - rx) / denominator);
            robot.rightBack.setPower((rotY + rotX - rx) / denominator);

            // Gamepad 2: Intake Controls
            if (gamepad2.b) {
                robot.intake.setPower(-1.0);
            } else {
                robot.intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
            }

            // Gamepad 2: Pollenlauncher Control (Button A: 70% power)
            if (robot.pollenlauncher != null) {
                if (gamepad2.a) {
                    robot.pollenlauncher.setPower(0.70);
                } else {
                    robot.pollenlauncher.setPower(0.0);
                }
            }

            // Gamepad 2: Mover Control (Left Bumper: 85% power + servos at 100%)
            if (robot.mover != null) {
                if (gamepad2.left_bumper) {
                    robot.mover.setPower(0.85);
                    if (robot.servo1 != null) robot.servo1.setPower(1.0);
                    if (robot.servo2 != null) robot.servo2.setPower(1.0);
                    if (robot.servo2Standard != null) robot.servo2Standard.setPosition(1.0);
                } else {
                    robot.mover.setPower(0.0);
                    if (robot.servo1 != null) robot.servo1.setPower(0.0);
                    if (robot.servo2 != null) robot.servo2.setPower(0.0);
                    if (robot.servo2Standard != null) robot.servo2Standard.setPosition(0.0);
                }
            }

            telemetry.addData("Control Mode", "Duo-Simple (Robot-Centric Driver 1 | Mechanisms Driver 2)");
            telemetry.update();
            sleep(10);
        }
        if (robot.limelight != null) robot.limelight.stop();
    }
}
