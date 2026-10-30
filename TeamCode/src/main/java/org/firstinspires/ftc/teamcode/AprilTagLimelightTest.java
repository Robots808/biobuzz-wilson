package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "AprilTag Limelight Auto Test", group = "Test")
public class AprilTagLimelightTest extends LinearOpMode {
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
            imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        } catch (Exception e) {
            telemetry.addData("IMU", "Not found or failed to init");
        }
        
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            limelight.start();

            while (opModeIsActive()) {
                if (imu != null) {
                    YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                    limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
                }
                
                LLResult llResult = limelight.getLatestResult();
                if (llResult != null && llResult.isValid()) {
                    Pose3D botPose = llResult.getBotpose_MT2();
                    if (botPose != null) {
                        telemetry.addData("BotPose X", "%.2f m", botPose.getPosition().x);
                        telemetry.addData("BotPose Y", "%.2f m", botPose.getPosition().y);
                        telemetry.addData("BotPose Z", "%.2f m", botPose.getPosition().z);
                    }
                    telemetry.addData("Tx", "%.2f", llResult.getTx());
                    telemetry.addData("Ty", "%.2f", llResult.getTy());
                    telemetry.addData("Target Area", "%.2f%%", llResult.getTa());
                } else {
                    telemetry.addData("Limelight", "No Valid Result");
                }
                telemetry.update();
            }

            limelight.stop();
        }
    }
}
