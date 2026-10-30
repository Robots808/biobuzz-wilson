package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Linear Slide Test", group="Robot")
public class linear_slide extends LinearOpMode {

    private DcMotor slideMotor = null;

    // GoBilda 117 RPM Motor: 1425.1 ticks per rev
    // Observed timing: 5.6s to fully extend. 
    // 5.6s * (117/60 rps) * 1425.1 ticks/rev = ~15562 ticks
    static final int MAX_LIMIT_TICKS = 15562;
    static final double TICKS_PER_MM = MAX_LIMIT_TICKS / 698.4;

    @Override
    public void runOpMode() {
        // Initialize the motor. Change "linearSlide" to match your configuration.
        try {
            slideMotor = hardwareMap.get(DcMotor.class, "linearSlide");
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setDirection(DcMotor.Direction.FORWARD);
            
            // Reset and start using encoders
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
        } catch (Exception e) {
            telemetry.addLine("!!! LINEAR SLIDE MOTOR NOT FOUND !!!");
            telemetry.update();
        }

        telemetry.addData("Status", "Initialized. Limit calibrated to 5.6s");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = 0;
            int currentPos = (slideMotor != null) ? slideMotor.getCurrentPosition() : 0;

            if (gamepad1.x) { // Square - Contract
                // Only allow extension if we are below the limit
                if (currentPos < MAX_LIMIT_TICKS) {
                    power = 1.0;
                } else {
                    power = 0; // Force stop/brake
                    telemetry.addLine("!!! MAX EXTENSION REACHED - BRAKING !!!");
                }
            } else if (gamepad1.y) { // Triangle - Extend
                // Only allow contraction if we are above the starting point
                if (currentPos < 10) {
                    power = -1.0;
                } else {
                    power = 0;
                    telemetry.addLine("!!! FULLY RETRACTED - BRAKING !!!");
                }
            }

            if (slideMotor != null) {
                slideMotor.setPower(power);
            }

            telemetry.addData("Slide Extension", "%.1f mm", currentPos / TICKS_PER_MM);
            telemetry.addData("Encoder Ticks", currentPos);
            telemetry.addData("Slide Power", "%.2f", power);
            telemetry.update();
        }
    }
}
