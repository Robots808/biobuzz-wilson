package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

public class HardwareIntake {

    private LinearOpMode myOpMode = null;

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor intake = null;
    public DcMotor pollenlauncher = null;
    public DcMotor mover = null;
    public IMU imu = null;
    public Limelight3A limelight = null;

    /*
    public CRServo rightintake = null;
    public CRServo leftintake = null;
    */

    public HardwareIntake(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        leftFront = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = myOpMode.hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = myOpMode.hardwareMap.get(DcMotor.class, "rightBack");
        intake = myOpMode.hardwareMap.get(DcMotor.class, "intake");
        
        /*
        leftintake = myOpMode.hardwareMap.get(CRServo.class, "leftintake");
        rightintake = myOpMode.hardwareMap.get(CRServo.class, "rightintake");
        */

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        /*
        leftintake.setDirection(CRServo.Direction.REVERSE);
        rightintake.setDirection(CRServo.Direction.FORWARD);
        */

        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motors to BRAKE to stop immediately when power is zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Limelight
        try {
            limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            limelight = null;
        }

        // Pollenlauncher
        try {
            pollenlauncher = myOpMode.hardwareMap.get(DcMotor.class, "pollenlauncher");
            pollenlauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pollenlauncher.setDirection(DcMotor.Direction.FORWARD);
            pollenlauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pollenlauncher.setPower(0);
        } catch (Exception e) {
            pollenlauncher = null;
        }

        // Mover
        try {
            mover = myOpMode.hardwareMap.get(DcMotor.class, "mover");
            mover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mover.setDirection(DcMotor.Direction.FORWARD);
            mover.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mover.setPower(0);
        } catch (Exception e) {
            mover = null;
        }

        // Hub IMU
        try {
            imu = myOpMode.hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
            );
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();
        } catch (Exception e) {
            imu = null;
        }

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }
}
