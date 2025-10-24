package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.Queue;
import java.util.LinkedList;

@Autonomous(name = "SequenceCollectorAuto_Full", group = "Botzilla")
public class SequenceCollectorAuto_Full extends LinearOpMode {

    // --------------------------
    // Hardware
    // --------------------------
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor intake;
    private DcMotor flywheel;
    private Servo flywheelArm;
    private ColorSensor intakeColorSensor;

    // Servo positions
    private final double ARM_UP_POS = 0.8;
    private final double ARM_DOWN_POS = 0.2;

    // Intake & verification settings
    private final long INTAKE_TIME_MS = 900;
    private final long REJECT_TIME_MS = 400;
    private final int MAX_RETRIES_PER_ITEM = 3;

    @Override
    public void runOpMode() {

        // --- Hardware map ---
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheelArm = hardwareMap.get(Servo.class, "flywheel_arm");
        intakeColorSensor = hardwareMap.get(ColorSensor.class, "intake_color");

        // Motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set arm up
        flywheelArm.setPosition(ARM_UP_POS);

        telemetry.addLine("Botzilla Auto Ready!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // -------------------- 1) Read obelisk sequence --------------------
            Queue<Integer> sequence = readObeliskSequence();
            telemetry.addData("Obelisk sequence", sequence.toString());
            telemetry.update();

            if (sequence.isEmpty()) {
                telemetry.addLine("No obelisk sequence detected. Aborting.");
                telemetry.update();
                return;
            }

            // -------------------- 2) Collect & verify balls --------------------
            int collectedCount = 0;
            while (!sequence.isEmpty() && opModeIsActive()) {
                int expected = sequence.peek();
                telemetry.addData("Expecting", expected);
                telemetry.update();

                boolean ok = collectAndVerify(expected);

                if (ok) {
                    sequence.poll();
                    collectedCount++;
                    telemetry.addData("Collected & Verified", expected);
                } else {
                    telemetry.addData("Failed to collect expected", expected);
                }
                telemetry.update();
            }

            // -------------------- 3) Navigate to goal --------------------
            navigateToGoal();

            // -------------------- 4) Fire collected balls --------------------
            fireCollectedBalls(collectedCount);

            telemetry.addLine("Auto Complete!");
            telemetry.update();
        }
    }

    // --------------------------
    // Helper Methods
    // --------------------------
    private Queue<Integer> readObeliskSequence() {
        // Placeholder: normally use vision
        // 1=green, 2=purple
        Queue<Integer> q = new LinkedList<>();
        q.add(1);
        q.add(2);
        q.add(1);
        return q;
    }

    private boolean collectAndVerify(int expectedValue) {
        for (int attempt = 1; attempt <= MAX_RETRIES_PER_ITEM && opModeIsActive(); attempt++) {
            telemetry.addData("Collect attempt", attempt);
            telemetry.update();

            // 1) Approach ball
            drive(0.3, 500);

            // 2) Intake
            intake.setPower(1.0);
            sleep(INTAKE_TIME_MS);
            intake.setPower(0.0);

            // 3) Verify color
            int observed = readColorValueFromSensor();
            telemetry.addData("Observed color", observed);
            telemetry.update();

            if (observed == expectedValue) {
                telemetry.addLine("Ball verified");
                telemetry.update();
                return true;
            } else {
                telemetry.addLine("Wrong ball - rejecting");
                telemetry.update();
                intake.setPower(-1.0);
                sleep(REJECT_TIME_MS);
                intake.setPower(0.0);
                drive(-0.2, 300); // back up a bit
            }
        }
        return false;
    }

    private int readColorValueFromSensor() {
        int r = intakeColorSensor.red();
        int g = intakeColorSensor.green();
        int b = intakeColorSensor.blue();

        // Simple heuristic
        if (g > r && g > b) return 1; // green
        if (r > 20 && b > 20 && g < Math.min(r,b)*0.6) return 2; // purple
        return 0;
    }

    private void navigateToGoal() {
        telemetry.addLine("Navigating to goal...");
        telemetry.update();
        drive(0.5, 1000); // placeholder, adjust based on field
    }

    private void fireCollectedBalls(int count) {
        if (count <= 0) return;
        telemetry.addData("Shooting", "Shots: " + count);
        telemetry.update();

        flywheel.setPower(1.0);
        sleep(700); // spin up

        for (int i = 0; i < count; i++) {
            intake.setPower(1.0);
            sleep(350);
            intake.setPower(0.0);
            sleep(650); // wait between shots
        }

        flywheel.setPower(0.0);
    }

    private void drive(double power, long durationMs) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        sleep(durationMs);
        stopDriving();
    }

    private void stopDriving() {
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }
}
