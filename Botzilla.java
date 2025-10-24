package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Botzilla TeleOp", group = "TeleOp")
public class BotzillaTeleOp extends LinearOpMode {

    private DcMotor left_drive;
    private DcMotor right_drive;
    private DcMotor intake;
    private DcMotor flywheel;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Botzilla Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            left_drive.setPower(drive + turn);
            right_drive.setPower(drive - turn);

            // Intake
            if (gamepad1.left_trigger > 0.1) intake.setPower(1.0);
            else if (gamepad1.left_bumper) intake.setPower(-1.0);
            else intake.setPower(0.0);

            // Flywheel
            if (gamepad1.right_bumper) flywheel.setPower(1.0);
            else if (gamepad1.right_trigger > 0.1) flywheel.setPower(0.0);

            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }
}
