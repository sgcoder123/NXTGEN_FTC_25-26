package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// OpenCV / EasyOpenCV
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Moments;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.LinkedList;

@Autonomous(name = "SequenceCollectorAuto_Full", group = "Autonomous")
public class SequenceCollectorAuto_Full extends LinearOpMode {

    // --------------------------
    // Hardware
    // --------------------------
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intake = null;
    private DcMotor flywheel = null;
    private Servo feederServo = null; // optional
    private ColorSensor intakeColorSensor = null;
    private BNO055IMU imu = null;

    // Camera
    private OpenCvCamera camera;
    private ObeliskAndBallPipeline pipeline;

    // --------------------------
    // Tunable constants (tune these)
    // --------------------------
    private static final String WEBCAM_NAME = "Webcam 1"; // change to your webcam config name
    private static final double DRIVE_SPEED = 0.45;
    private static final double TURN_SPEED = 0.32;
    private static final double COUNTS_PER_MOTOR_REV = 1120; // For NeveRest 40; change to your motors
    private static final double WHEEL_DIAMETER_INCHES = 4.0;  // change to your wheels
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double FLYWHEEL_POWER_30IN = 0.70; // tune on field
    private static final long INTAKE_TIME_MS = 900; // how long intake runs to pull ball
    private static final long REJECT_TIME_MS = 400; // reverse intake to eject wrong ball
    private static final int MAX_RETRIES_PER_ITEM = 3;
    private static final int OBELISK_NUM_PANELS = 4; // how many panels to read left->right
    private static final long VISION_STABLE_MS = 300; // time to wait for camera results to stabilize

    private ElapsedTime runtime = new ElapsedTime();

    // --------------------------
    // OpMode lifecycle
    // --------------------------
    @Override
    public void runOpMode() {

        // --- hardware map ---
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        // optional:
        // feederServo = hardwareMap.get(Servo.class, "feeder");
        intakeColorSensor = hardwareMap.get(ColorSensor.class, "intake_color");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU init
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(imuParams);

        // --- camera & pipeline ---
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        pipeline = new ObeliskAndBallPipeline(OBELISK_NUM_PANELS);
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(com.qualcomm.robotcore.hardware.WebcamName.class, WEBCAM_NAME),
                        cameraMonitorViewId);
        camera.setPipeline(pipeline);

        // start camera streaming (320x240 is a good tradeoff; you can increase)
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override public void onError(int errorCode) {
                telemetry.addData("Camera", "Failed to open. error: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Init complete. Waiting for start...");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Wait a short time for pipeline to produce stable readings
        sleep(VISION_STABLE_MS);

        // 1) Read obelisk -> produce a queue of expected values (1=green,2=purple)
        Queue<Integer> sequence = readObeliskSequence();
        telemetry.addData("Obelisk sequence", sequence.toString());
        telemetry.update();

        if (sequence.isEmpty()) {
            telemetry.addLine("No obelisk sequence detected. Aborting.");
            telemetry.update();
            stopCamera();
            return;
        }

        // 2) For each expected value, find and collect verified ball
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
                // policy here: move on or retry depending on strategy; we continue attempts up to MAX_RETRIES inside collectAndVerify
            }
            telemetry.update();
        }

        // 3) Navigate to goal (use encoders + IMU)
        navigateToGoal();

        // 4) Fire collected balls
        fireCollectedBalls(collectedCount);

        telemetry.addLine("Auto complete");
        telemetry.update();

        stopCamera();
    }

    private void stopCamera() {
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }

    // --------------------------
    // Vision helpers
    // --------------------------
    // Reads the pipeline's obelisk panel results and converts to queue
    private Queue<Integer> readObeliskSequence() {
        Queue<Integer> q = new LinkedList<>();

        // Wait a short time to ensure pipeline processed frames
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < VISION_STABLE_MS && opModeIsActive()) {
            sleep(20);
        }

        int[] panels = pipeline.getObeliskPanels(); // 0 = unknown, 1 = green, 2 = purple
        if (panels == null || panels.length == 0) return q;

        for (int v : panels) {
            if (v == 1 || v == 2) q.add(v);
            else {
                // unknown panel: you can choose fallback (skip, abort, or set default)
                telemetry.addData("Panel unknown", v);
                telemetry.update();
                // For safety, abort if unknown
                // return new LinkedList<>();
            }
        }
        return q;
    }

    // Attempts to find a ball using vision and collect & verify it matches expectedValue.
    private boolean collectAndVerify(int expectedValue) {
        for (int attempt = 1; attempt <= MAX_RETRIES_PER_ITEM && opModeIsActive(); attempt++) {
            telemetry.addData("Collect attempt", attempt);
            telemetry.update();

            // 1) Search for ball via camera
            boolean found = searchForBallWithCamera(4000); // timeout ms
            if (!found) {
                telemetry.addData("Search", "no ball found this attempt");
                telemetry.update();
                continue;
            }

            // 2) Align & approach the ball: when pipeline reports center near image center, drive forward a fixed distance
            boolean aligned = alignToBall(); // uses pipeline.getLatestBallCenterX()
            if (!aligned) {
                telemetry.addData("Align", "failed");
                telemetry.update();
                continue;
            }

            // Drive forward to intake the ball (fixed distance)
            driveForwardInches(20); // tune this distance based on mounting and camera FOV

            // Run intake to pull ball in
            intake.setPower(1.0);
            sleep(INTAKE_TIME_MS);
            intake.setPower(0.0);

            // 3) Verify color via color sensor
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
                // Reject: reverse intake then back away and rotate a bit
                intake.setPower(-1.0);
                sleep(REJECT_TIME_MS);
                intake.setPower(0.0);
                driveBackwardInches(6);
                rotateDegrees(20); // avoid same ball
            }
        }
        return false; // failed after retries
    }

    // uses pipeline to rotate until a ball is detected (center X available).
    // timeoutMs: how long to search
    private boolean searchForBallWithCamera(long timeoutMs) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < timeoutMs && opModeIsActive()) {
            if (pipeline.hasBall()) {
                telemetry.addData("Vision", "ball found");
                telemetry.update();
                return true;
            } else {
                // rotate a small amount, then check again
                rotateDegrees(18);
            }
            sleep(50);
        }
        return false;
    }

    // Align robot so ball center is near image center using small turns.
    // Returns true if alignment achieved (or no pipeline data).
    private boolean alignToBall() {
        final double CENTER_TOLERANCE_PIXELS = 20.0;
        final int MAX_ALIGN_TRIES = 6;

        for (int i = 0; i < MAX_ALIGN_TRIES && opModeIsActive(); i++) {
            if (!pipeline.hasBall()) return false;

            double cx = pipeline.getLatestBallCenterX();
            double imgW = pipeline.getImageWidth();
            double delta = cx - (imgW / 2.0);

            telemetry.addData("Align", "cx=%.1f delta=%.1f", cx, delta);
            telemetry.update();

            if (Math.abs(delta) < CENTER_TOLERANCE_PIXELS) {
                // already centered
                return true;
            } else {
                // Turn proportionally: positive delta => ball is to right => rotate right slightly
                double sign = Math.signum(delta);
                double turnDeg = Math.min(10, Math.abs(delta) / 10.0 * 6.0); // heuristic mapping
                rotateDegrees(turnDeg * sign);
                sleep(120);
            }
        }
        return pipeline.hasBall(); // best effort
    }

    // Read color sensor and map to 1 (green) or 2 (purple) or 0 unknown.
    private int readColorValueFromSensor() {
        // Sample for a short stable window
        long start = System.currentTimeMillis();
        int votesGreen = 0;
        int votesPurple = 0;

        while (System.currentTimeMillis() - start < 120 && opModeIsActive()) {
            int r = intakeColorSensor.red();
            int g = intakeColorSensor.green();
            int b = intakeColorSensor.blue();

            // rough heuristic:
            if (g > r && g > b && g > 30) votesGreen++;
            // purple often has both red and blue significant and green low
            if (r > 20 && b > 20 && g < Math.min(r, b) * 0.6) votesPurple++;

            sleep(25);
        }

        if (votesGreen > votesPurple) return 1;
        if (votesPurple > votesGreen) return 2;
        return 0;
    }

    // --------------------------
    // Navigation & drive primitives (encoder + imu)
    // --------------------------

    // Drive forward (inches) using RUN_TO_POSITION
    private void driveForwardInches(double inches) {
        encoderDrive(DRIVE_SPEED, inches, inches);
    }

    private void driveBackwardInches(double inches) {
        encoderDrive(DRIVE_SPEED, -inches, -inches);
    }

    private void encoderDrive(double speed, double leftInches, double rightInches) {
        int newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        int newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            telemetry.addData("Encoders", "L=%d R=%d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            telemetry.update();
            sleep(10);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Rotate using IMU: positive degrees rotates robot to right (clockwise)
    private void rotateDegrees(double degrees) {
        // Simple proportional turning to reach target heading
        double startHeading = getHeading();
        double target = normalizeAngle(startHeading + degrees);

        // small PID-like loop (P-only)
        while (opModeIsActive()) {
            double current = getHeading();
            double error = angleDiff(target, current);
            if (Math.abs(error) < 3.0) break;
            double power = TURN_SPEED * (Math.min(1.0, Math.abs(error) / 30.0));
            double dir = Math.signum(error);
            leftDrive.setPower(dir * power);
            rightDrive.setPower(-dir * power);
            telemetry.addData("Rotate", "target=%.1f cur=%.1f err=%.1f", target, current, error);
            telemetry.update();
            sleep(10);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(100);
    }

    private double getHeading() {
        Orientation o = imu.getAngularOrientation();
        // firstAngle is in degrees, range -180..180
        return o.firstAngle;
    }

    // normalize angle to -180..180
    private double normalizeAngle(double a) {
        while (a <= -180) a += 360;
        while (a > 180) a -= 360;
        return a;
    }

    // difference from target to current (-180..180), positive means need to turn right
    private double angleDiff(double target, double current) {
        double diff = target - current;
        while (diff <= -180) diff += 360;
        while (diff > 180) diff -= 360;
        return diff;
    }

    // Navigate to goal zone: simple placeholder
    private void navigateToGoal() {
        telemetry.addLine("Navigating to goal...");
        telemetry.update();
        // TODO: replace with your field-specific path using encoderDrive() and rotateDegrees()
        driveForwardInches(36); // example: drive forward 36 inches
        rotateDegrees(-45);     // example adjust
    }

    // Fire collected balls (count = how many verified balls you collected)
    private void fireCollectedBalls(int count) {
        if (count <= 0) return;
        telemetry.addData("Shooting", "Shots: " + count);
        telemetry.update();

        flywheel.setPower(FLYWHEEL_POWER_30IN);
        sleep(700); // spin up — tune/pid if you have velocity control

        for (int i = 0; i < count; i++) {
            // If using servo feeder:
            if (feederServo != null) {
                feederServo.setPosition(0.2);
                sleep(300);
                feederServo.setPosition(0.8);
            } else {
                // fallback: pulse intake to push ball into flywheel
                intake.setPower(1.0);
                sleep(350);
                intake.setPower(0.0);
            }
            sleep(650); // wait between shots for stability
        }

        flywheel.setPower(0.0);
    }

    // --------------------------
    // OpenCvPipeline inner class
    // - Detects obelisk multi-panel colors and ball blobs (green & purple)
    // - Provides thread-safe getters to OpMode
    // --------------------------
    public static class ObeliskAndBallPipeline extends OpenCvPipeline {

        private final int panelCount;
        private final Rect[] panelROIs;

        // pipeline outputs (volatile for thread-safety)
        private volatile int[] obeliskPanels; // values 0=unknown,1=green,2=purple
        private volatile boolean ballDetected = false;
        private volatile double ballCenterX = 0.0;
        private volatile double lastImageWidth = 320;

        // Temporary mats
        private Mat hsv = new Mat();
        private Mat blurred = new Mat();
        private Mat maskGreen = new Mat();
        private Mat maskPurple = new Mat();
        private Mat maskBoth = new Mat();
        private Mat hierarchy = new Mat();

        // HSV thresholds: tune these for your lighting!
        // Note: OpenCV Hue range 0..180 (half of degrees)
        private final Scalar GREEN_LOWER = new Scalar(35/2.0, 50, 50);   // approx hue 35°
        private final Scalar GREEN_UPPER = new Scalar(85/2.0, 255, 255);  // approx hue 85°
        private final Scalar PURPLE_LOWER = new Scalar(125/2.0, 40, 40);  // ~250°/2
        private final Scalar PURPLE_UPPER = new Scalar(155/2.0, 255, 255); // ~310°/2

        public ObeliskAndBallPipeline(int panels) {
            this.panelCount = Math.max(1, panels);
            panelROIs = new Rect[panelCount];
        }

        @Override
        public Mat processFrame(Mat input) {

            // Preprocess
            Imgproc.GaussianBlur(input, blurred, new Size(5, 5), 0);
            Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

            // Create masks
            Core.inRange(hsv, GREEN_LOWER, GREEN_UPPER, maskGreen);
            Core.inRange(hsv, PURPLE_LOWER, PURPLE_UPPER, maskPurple);
            Core.bitwise_or(maskGreen, maskPurple, maskBoth);

            // --- Obelisk detection (panels left->right) ---
            int width = input.width();
            int height = input.height();
            lastImageWidth = width;

            // Define ROIs across the top center area by default — change to match your obelisk location in camera view
            // This uses a horizontal strip centered vertically at 30% height with panelCount equal-width slices
            int roiTop = (int)(height * 0.12);
            int roiHeight = (int)(height * 0.18);
            int roiY = Math.max(0, roiTop);
            int roiW = width / panelCount;
            for (int i = 0; i < panelCount; i++) {
                int x = i * roiW;
                panelROIs[i] = new Rect(x, roiY, roiW, roiHeight);
            }

            int[] panels = new int[panelCount];
            for (int i = 0; i < panelCount; i++) {
                Rect r = panelROIs[i];
                Mat subGreen = new Mat(maskGreen, r);
                Mat subPurple = new Mat(maskPurple, r);
                double greenCount = Core.countNonZero(subGreen);
                double purpleCount = Core.countNonZero(subPurple);
                // choose whichever has more pixels and exceed threshold relative to ROI size
                double area = r.width * r.height;
                double threshold = Math.max(10, area * 0.03); // at least 3% of ROI or 10 pixels
                if (greenCount > purpleCount && greenCount > threshold) {
                    panels[i] = 1;
                    Imgproc.rectangle(input, r.tl(), r.br(), new Scalar(0,255,0), 2);
                } else if (purpleCount > greenCount && purpleCount > threshold) {
                    panels[i] = 2;
                    Imgproc.rectangle(input, r.tl(), r.br(), new Scalar(255,0,255), 2);
                } else {
                    panels[i] = 0;
                    Imgproc.rectangle(input, r.tl(), r.br(), new Scalar(0,0,255), 1);
                }
                subGreen.release();
                subPurple.release();
            }
            obeliskPanels = panels; // publish

            // --- Ball detection: find largest contour in maskBoth (green OR purple) ---
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(maskBoth.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double bestArea = 0;
            Point bestCenter = null;
            for (MatOfPoint cnt : contours) {
                double area = Imgproc.contourArea(cnt);
                if (area < 50) continue; // filter small noise
                Moments m = Imgproc.moments(cnt);
                if (m.get_m00() == 0) continue;
                double cx = m.get_m10() / m.get_m00();
                double cy = m.get_m01() / m.get_m00();

                if (area > bestArea) {
                    bestArea = area;
                    bestCenter = new Point(cx, cy);
                }
            }

            if (bestCenter != null) {
                ballDetected = true;
                ballCenterX = bestCenter.x;
                Imgproc.circle(input, bestCenter, 6, new Scalar(255,255,0), -1);
            } else {
                ballDetected = false;
            }

            // Optionally draw debugging visuals
            Imgproc.putText(input, "Panels: " + panelsToString(panels), new Point(8,20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255,255,255),1);
            if (ballDetected) Imgproc.putText(input, String.format("BallX: %.1f", ballCenterX), new Point(8,40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255,255,255),1);

            // Return annotated frame to display
            return input;
        }

        private String panelsToString(int[] panels) {
            StringBuilder sb = new StringBuilder();
            sb.append("[");
            for (int i = 0; i < panels.length; i++) {
                sb.append(panels[i]);
                if (i < panels.length - 1) sb.append(",");
            }
            sb.append("]");
            return sb.toString();
        }

        // ---- Thread-safe getters ----
        public int[] getObeliskPanels() {
            return obeliskPanels == null ? new int[0] : obeliskPanels.clone();
        }
        public boolean hasBall() {
            return ballDetected;
        }
        public double getLatestBallCenterX() {
            return ballCenterX;
        }
        public double getImageWidth() {
            return lastImageWidth;
        }
    }
}
