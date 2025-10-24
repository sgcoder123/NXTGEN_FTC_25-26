package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BotzillaTeleOp", group = "Botzilla")
public class BotzillaTeleOp extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor intake;
    private DcMotor flywheel;
    private Servo flywheelArm;

    private final double ARM_UP_POS = 0.8;
    private final double ARM_DOWN_POS = 0.2;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheelArm = hardwareMap.get(Servo.class, "flywheel_arm");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        flywheelArm.setPosition(ARM_UP_POS);

        telemetry.addLine("Botzilla TeleOp Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // -------------------- Tank drive --------------------
            double leftPower = -gamepad1.left_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // -------------------- Intake control --------------------
            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(gamepad1.right_trigger); // intake
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-gamepad1.left_trigger); // outtake
            } else {
                intake.setPower(0.0);
            }

            // -------------------- Flywheel control --------------------
            if (gamepad1.b) {
                flywheel.setPower(1.0); // spin up
            } else if (gamepad1.a) {
                flywheel.setPower(0.0); // stop
            }

            // -------------------- Flywheel arm control --------------------
            if (gamepad1.x) {
                flywheelArm.setPosition(ARM_DOWN_POS);
            } else if (gamepad1.y) {
                flywheelArm.setPosition(ARM_UP_POS);
            }

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Flywheel", flywheel.getPower());
            telemetry.addData("Arm Pos", flywheelArm.getPosition());
            telemetry.update();
        }
    }
}
