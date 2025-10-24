package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BotzillaTeleOp", group = "Botzilla")
public class BotzillaTeleOp extends LinearOpMode {

    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor intake;
    private DcMotor flywheel;
    private Servo flywheel_arm;

    // Servo positions — tune these for your robot
    private final double ARM_UP_POS = 0.8;
    private final double ARM_DOWN_POS = 0.2;

    @Override
    public void runOpMode() {
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel_arm = hardwareMap.get(Servo.class, "flywheel_arm");

        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.FORWARD);

        flywheel_arm.setPosition(ARM_UP_POS);

        telemetry.addLine("Botzilla Auto Ready!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // -------------------- Autonomous Sequence --------------------

            // 1. Drive forward
            drive(0.5, 1000);

            // 2. Stop driving
            stopDriving();

            // 3. Run intake briefly to load a ball
            intake.setPower(1.0);
            sleep(1000);
            intake.setPower(0.0);

            // 4. Spin up flywheel
            flywheel.setPower(1.0);
            telemetry.addLine("Spinning up flywheel...");
            telemetry.update();
            sleep(1500); // wait for spin-up

            // 5. Lower arm to shoot
            flywheel_arm.setPosition(ARM_DOWN_POS);
            telemetry.addLine("Launching ball!");
            telemetry.update();
            sleep(700);

            // 6. Raise arm back up
            flywheel_arm.setPosition(ARM_UP_POS);
            sleep(300);

            // 7. Turn off flywheel
            flywheel.setPower(0.0);

            // 8. Drive backward to finish
            drive(-0.4, 800);

            stopDriving();

            telemetry.addLine("Auto Complete!");
            telemetry.update();
        }
    }

    // -------------------- Helper Methods --------------------

    private void drive(double power, long durationMs) {
        left_drive.setPower(power);
        right_drive.setPower(power);
        sleep(durationMs);
        stopDriving();
    }

    private void stopDriving() {
        left_drive.setPower(0.0);
        right_drive.setPower(0.0);
    }
}
