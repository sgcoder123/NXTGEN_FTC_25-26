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
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(name = "SequenceCollectorAuto_Enhanced", group = "Autonomous")
public class SequenceCollectorAuto_Enhanced extends LinearOpMode {

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
    // Configuration Constants
    // --------------------------
    private static final String WEBCAM_NAME = "Webcam 1";
    
    // Movement Configuration
    private static final double DRIVE_SPEED = 0.45;
    private static final double TURN_SPEED = 0.32;
    private static final double COUNTS_PER_MOTOR_REV = 1120; // NeveRest 40
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double WHEEL_BASE_WIDTH = 12.0; // inches between wheels
    
    // PID Control Constants
    private static final double TURN_KP = 0.02;
    private static final double TURN_KI = 0.001;
    private static final double TURN_KD = 0.005;
    private static final double DRIVE_KP = 0.015;
    private static final double DRIVE_KI = 0.001;
    private static final double DRIVE_KD = 0.003;
    
    // Shooting Configuration
    private static final double FLYWHEEL_POWER_30IN = 0.70;
    private static final long FLYWHEEL_SPINUP_MS = 700;
    private static final long SHOT_DELAY_MS = 650;
    
    // Intake Configuration
    private static final long INTAKE_TIME_MS = 900;
    private static final long REJECT_TIME_MS = 400;
    private static final double INTAKE_POWER = 1.0;
    private static final double REJECT_POWER = -1.0;
    
    // Vision Configuration
    private static final int MAX_RETRIES_PER_ITEM = 3;
    private static final int OBELISK_NUM_PANELS = 4;
    private static final long VISION_STABLE_MS = 500;
    private static final long VISION_TIMEOUT_MS = 4000;
    private static final double BALL_DETECTION_MIN_AREA = 50.0;
    private static final double BALL_ALIGNMENT_TOLERANCE = 20.0;
    
    // Color Detection Configuration
    private static final long COLOR_SAMPLE_TIME_MS = 150;
    private static final int COLOR_SAMPLE_INTERVAL_MS = 25;
    private static final int COLOR_GREEN_THRESHOLD = 30;
    private static final double COLOR_PURPLE_GREEN_RATIO = 0.6;
    
    // Navigation Configuration
    private static final double NAVIGATION_SPEED = 0.4;
    private static final double TURN_TOLERANCE_DEGREES = 2.0;
    private static final double DRIVE_TOLERANCE_INCHES = 0.5;
    
    // Safety Configuration
    private static final long MAX_OPMODE_TIME_MS = 30000; // 30 seconds max
    private static final long EMERGENCY_STOP_TIMEOUT_MS = 5000;

    // Timing and State Management
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime lastSuccessfulAction = new ElapsedTime();
    private AtomicBoolean emergencyStop = new AtomicBoolean(false);
    private AtomicInteger totalBallsCollected = new AtomicInteger(0);
    private AtomicInteger totalBallsShot = new AtomicInteger(0);
    
    // PID Controllers
    private PIDController turnController;
    private PIDController driveController;
    
    // State tracking
    private RobotState currentState = RobotState.INITIALIZING;
    private Queue<Integer> targetSequence = new LinkedList<>();
    private List<Double> navigationPath = new ArrayList<>();
    
    // Enhanced logging
    private StringBuilder operationLog = new StringBuilder();
    
    // Inner classes for better organization
    private enum RobotState {
        INITIALIZING, READING_OBELISK, SEARCHING_BALL, ALIGNING_BALL, 
        COLLECTING_BALL, VERIFYING_BALL, NAVIGATING, SHOOTING, COMPLETED, ERROR
    }
    
    private static class PIDController {
        private double kp, ki, kd;
        private double lastError = 0;
        private double integral = 0;
        private double derivative = 0;
        private ElapsedTime timer = new ElapsedTime();
        
        public PIDController(double kp, double ki, double kd) {
            this.kp = kp; this.ki = ki; this.kd = kd;
            timer.reset();
        }
        
        public double calculate(double target, double current) {
            double error = target - current;
            double dt = timer.seconds();
            timer.reset();
            
            integral += error * dt;
            derivative = (error - lastError) / dt;
            
            double output = kp * error + ki * integral + kd * derivative;
            lastError = error;
            
            return Math.max(-1.0, Math.min(1.0, output));
        }
        
        public void reset() {
            lastError = 0;
            integral = 0;
            derivative = 0;
            timer.reset();
        }
    }

    // --------------------------
    // OpMode lifecycle
    // --------------------------
    @Override
    public void runOpMode() {
        try {
            initializeHardware();
            initializeControllers();
            initializeVision();
            
            currentState = RobotState.INITIALIZING;
            logOperation("Hardware initialization complete");
            
            telemetry.addLine("Enhanced AutoBotzilla Ready!");
            telemetry.addData("Version", "2.0 Enhanced");
            telemetry.update();

            waitForStart();
            runtime.reset();
            lastSuccessfulAction.reset();
            
            currentState = RobotState.READING_OBELISK;
            logOperation("OpMode started");
            
            // Main autonomous sequence with enhanced error handling
            executeAutonomousSequence();
            
        } catch (Exception e) {
            handleError("Critical error in runOpMode", e);
        } finally {
            cleanup();
        }
    }
    
    private void initializeHardware() {
        // Hardware mapping with error checking
        leftDrive = getHardwareWithErrorCheck(DcMotor.class, "left_drive");
        rightDrive = getHardwareWithErrorCheck(DcMotor.class, "right_drive");
        intake = getHardwareWithErrorCheck(DcMotor.class, "intake");
        flywheel = getHardwareWithErrorCheck(DcMotor.class, "flywheel");
        intakeColorSensor = getHardwareWithErrorCheck(ColorSensor.class, "intake_color");
        imu = getHardwareWithErrorCheck(BNO055IMU.class, "imu");
        
        // Optional hardware
        try {
            feederServo = hardwareMap.get(Servo.class, "feeder");
        } catch (Exception e) {
            logOperation("Feeder servo not found - using intake fallback");
        }

        // Motor configuration
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // Encoder setup
        setupEncoders();
        
        // IMU initialization
        initializeIMU();
    }
    
    private <T> T getHardwareWithErrorCheck(Class<T> type, String name) {
        try {
            T hardware = hardwareMap.get(type, name);
            if (hardware == null) {
                throw new RuntimeException("Hardware device '" + name + "' not found");
            }
            logOperation("Hardware mapped: " + name);
            return hardware;
        } catch (Exception e) {
            handleError("Failed to map hardware: " + name, e);
            return null;
        }
    }
    
    private void setupEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set zero power behavior
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void initializeIMU() {
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.calibrationDataFile = "BNO055IMUCalibration.json";
        imuParams.loggingEnabled = true;
        imuParams.loggingTag = "IMU";
        imuParams.accelerationIntegrationAlgorithm = new BNO055IMU.AccelerationIntegrationAlgorithm() {
            @Override
            public void initialize(BNO055IMU.Parameters parameters, Orientation initialOrientation) {}
            @Override
            public Orientation update(Orientation orientation, Acceleration acceleration) {
                return orientation;
            }
        };
        
        if (!imu.initialize(imuParams)) {
            handleError("IMU initialization failed", null);
        }
        
        // Wait for IMU calibration
        while (!imu.isGyroCalibrated() && opModeIsActive()) {
            telemetry.addData("IMU", "Calibrating...");
            telemetry.update();
            sleep(50);
        }
        logOperation("IMU initialized and calibrated");
    }
    
    private void initializeControllers() {
        turnController = new PIDController(TURN_KP, TURN_KI, TURN_KD);
        driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
        logOperation("PID controllers initialized");
    }

    private void initializeVision() {
        try {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            pipeline = new EnhancedObeliskAndBallPipeline(OBELISK_NUM_PANELS);
            camera = OpenCvCameraFactory.getInstance()
                    .createWebcam(hardwareMap.get(com.qualcomm.robotcore.hardware.WebcamName.class, WEBCAM_NAME),
                            cameraMonitorViewId);
            camera.setPipeline(pipeline);

            // Enhanced camera initialization with better error handling
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override public void onOpened() {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                    logOperation("Camera streaming started at 640x480");
                }
                @Override public void onError(int errorCode) {
                    handleError("Camera failed to open. Error code: " + errorCode, null);
                }
            });
            
            // Wait for camera to initialize
            sleep(1000);
            logOperation("Vision system initialized");
            
        } catch (Exception e) {
            handleError("Vision initialization failed", e);
        }
    }
    
    private void executeAutonomousSequence() {
        // Wait for vision to stabilize
        sleep(VISION_STABLE_MS);
        
        // 1) Read obelisk sequence with enhanced error handling
        if (!readObeliskSequenceEnhanced()) {
            handleError("Failed to read obelisk sequence", null);
            return;
        }
        
        // 2) Collect balls with improved reliability
        collectBallsSequence();
        
        // 3) Navigate to goal with path planning
        navigateToGoalEnhanced();
        
        // 4) Shoot collected balls with improved accuracy
        shootCollectedBallsEnhanced();
        
        currentState = RobotState.COMPLETED;
        logOperation("Autonomous sequence completed successfully");
        
        telemetry.addLine("Enhanced Auto Complete!");
        telemetry.addData("Balls Collected", totalBallsCollected.get());
        telemetry.addData("Balls Shot", totalBallsShot.get());
        telemetry.addData("Total Time", runtime.seconds());
        telemetry.update();
    }
    }

    // Enhanced movement methods with PID control
    private void driveForwardInches(double inches) {
        encoderDriveEnhanced(DRIVE_SPEED, inches, inches);
    }

    private void driveBackwardInches(double inches) {
        encoderDriveEnhanced(DRIVE_SPEED, -inches, -inches);
    }
    
    private void strafeLeftInches(double inches) {
        encoderDriveEnhanced(DRIVE_SPEED, -inches, inches);
    }
    
    private void strafeRightInches(double inches) {
        encoderDriveEnhanced(DRIVE_SPEED, inches, -inches);
    }

    private void encoderDriveEnhanced(double speed, double leftInches, double rightInches) {
        int newLeftTarget = leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        int newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        // Enhanced monitoring with PID correction
        while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            // PID correction for straight driving
            double leftError = newLeftTarget - leftDrive.getCurrentPosition();
            double rightError = newRightTarget - rightDrive.getCurrentPosition();
            
            double leftPower = driveController.calculate(0, leftError);
            double rightPower = driveController.calculate(0, rightError);
            
            leftDrive.setPower(Math.abs(speed) + leftPower);
            rightDrive.setPower(Math.abs(speed) + rightPower);
            
            telemetry.addData("Drive", "L=%d R=%d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            telemetry.addData("Target", "L=%d R=%d", newLeftTarget, newRightTarget);
            telemetry.update();
            sleep(10);
        }

        stopMotors();
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Enhanced rotation with PID control
    private void rotateDegreesEnhanced(double degrees) {
        double startHeading = getHeading();
        double target = normalizeAngle(startHeading + degrees);
        
        turnController.reset();
        
        while (opModeIsActive()) {
            double current = getHeading();
            double error = angleDiff(target, current);
            
            if (Math.abs(error) < TURN_TOLERANCE_DEGREES) {
                break;
            }
            
            double power = turnController.calculate(target, current);
            
            leftDrive.setPower(power);
            rightDrive.setPower(-power);
            
            telemetry.addData("Rotate", "target=%.1f cur=%.1f err=%.1f power=%.2f", 
                    target, current, error, power);
            telemetry.update();
            sleep(10);
        }
        
        stopMotors();
        sleep(100);
    }
    
    private void stopMotors() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    
    private void stopCamera() {
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }

    // Enhanced vision and sequence reading
    private boolean readObeliskSequenceEnhanced() {
        currentState = RobotState.READING_OBELISK;
        
        // Wait for vision to stabilize with timeout
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < VISION_STABLE_MS && opModeIsActive()) {
            sleep(20);
        }

        // Multiple attempts to read sequence
        for (int attempt = 1; attempt <= 3; attempt++) {
            int[] panels = pipeline.getObeliskPanels();
            if (panels == null || panels.length == 0) {
                telemetry.addData("Vision", "No panels detected, attempt " + attempt);
                telemetry.update();
                sleep(500);
                continue;
            }

            targetSequence.clear();
            boolean validSequence = true;
            
            for (int i = 0; i < panels.length; i++) {
                int panel = panels[i];
                if (panel == 1 || panel == 2) {
                    targetSequence.add(panel);
                } else {
                    telemetry.addData("Panel " + i, "Unknown value: " + panel);
                    validSequence = false;
                }
            }
            
            if (validSequence && !targetSequence.isEmpty()) {
                logOperation("Obelisk sequence read: " + targetSequence.toString());
                telemetry.addData("Sequence", targetSequence.toString());
                telemetry.update();
                return true;
            }
            
            sleep(300); // Wait before retry
        }
        
        return false;
    }

    private void collectBallsSequence() {
        while (!targetSequence.isEmpty() && opModeIsActive() && !emergencyStop.get()) {
            int expected = targetSequence.peek();
            currentState = RobotState.SEARCHING_BALL;
            
            telemetry.addData("Expecting", expected);
            telemetry.addData("Remaining", targetSequence.size());
            telemetry.update();

            boolean success = collectAndVerifyEnhanced(expected);

            if (success) {
                targetSequence.poll();
                totalBallsCollected.incrementAndGet();
                lastSuccessfulAction.reset();
                logOperation("Successfully collected ball: " + expected);
            } else {
                logOperation("Failed to collect expected ball: " + expected);
                // Check for emergency stop conditions
                if (lastSuccessfulAction.seconds() > EMERGENCY_STOP_TIMEOUT_MS / 1000.0) {
                    emergencyStop.set(true);
                    handleError("Emergency stop triggered - no progress for " + EMERGENCY_STOP_TIMEOUT_MS + "ms", null);
                    break;
                }
            }
            
            // Safety check for maximum runtime
            if (runtime.seconds() > MAX_OPMODE_TIME_MS / 1000.0) {
                logOperation("Maximum runtime reached, stopping collection");
                break;
            }
        }
    }
    
    // Enhanced ball collection with better error handling
    private boolean collectAndVerifyEnhanced(int expectedValue) {
        for (int attempt = 1; attempt <= MAX_RETRIES_PER_ITEM && opModeIsActive(); attempt++) {
            currentState = RobotState.SEARCHING_BALL;
            telemetry.addData("Collect attempt", attempt + "/" + MAX_RETRIES_PER_ITEM);
            telemetry.update();

            // 1) Search for ball with enhanced timeout
            boolean found = searchForBallWithCameraEnhanced(VISION_TIMEOUT_MS);
            if (!found) {
                telemetry.addData("Search", "No ball found this attempt");
                telemetry.update();
                continue;
            }

            // 2) Enhanced alignment
            currentState = RobotState.ALIGNING_BALL;
            boolean aligned = alignToBallEnhanced();
            if (!aligned) {
                telemetry.addData("Align", "Alignment failed");
                telemetry.update();
                continue;
            }

            // 3) Approach and collect
            currentState = RobotState.COLLECTING_BALL;
            driveForwardInches(18); // Slightly reduced for better accuracy

            // Enhanced intake with feedback
            intake.setPower(INTAKE_POWER);
            sleep(INTAKE_TIME_MS);
            intake.setPower(0.0);

            // 4) Enhanced color verification
            currentState = RobotState.VERIFYING_BALL;
            int observed = readColorValueFromSensorEnhanced();
            telemetry.addData("Expected", expectedValue);
            telemetry.addData("Observed", observed);
            telemetry.update();

            if (observed == expectedValue) {
                telemetry.addLine("Ball verified successfully!");
                telemetry.update();
                return true;
            } else {
                telemetry.addLine("Wrong ball - rejecting");
                telemetry.update();
                
                // Enhanced rejection sequence
                intake.setPower(REJECT_POWER);
                sleep(REJECT_TIME_MS);
                intake.setPower(0.0);
                driveBackwardInches(8);
                rotateDegreesEnhanced(25); // Avoid same ball
            }
        }
        return false;
    }

    // Enhanced ball search with better scanning pattern
    private boolean searchForBallWithCameraEnhanced(long timeoutMs) {
        long start = System.currentTimeMillis();
        int searchDirection = 1; // 1 for right, -1 for left
        double totalRotation = 0;
        
        while (System.currentTimeMillis() - start < timeoutMs && opModeIsActive()) {
            if (pipeline.hasBall()) {
                telemetry.addData("Vision", "Ball found!");
                telemetry.update();
                return true;
            }
            
            // Adaptive search pattern
            double rotationStep = Math.min(15, 5 + totalRotation / 100); // Start small, increase over time
            rotateDegreesEnhanced(rotationStep * searchDirection);
            totalRotation += rotationStep;
            
            // Change direction if we've rotated too far
            if (totalRotation > 180) {
                searchDirection *= -1;
                totalRotation = 0;
            }
            
            sleep(100);
        }
        return false;
    }

    // Enhanced ball alignment with PID control
    private boolean alignToBallEnhanced() {
        final int MAX_ALIGN_TRIES = 8;
        
        for (int i = 0; i < MAX_ALIGN_TRIES && opModeIsActive(); i++) {
            if (!pipeline.hasBall()) {
                telemetry.addData("Align", "Ball lost during alignment");
                telemetry.update();
                return false;
            }

            double cx = pipeline.getLatestBallCenterX();
            double imgW = pipeline.getImageWidth();
            double targetX = imgW / 2.0;
            double delta = cx - targetX;

            telemetry.addData("Align", "cx=%.1f target=%.1f delta=%.1f", cx, targetX, delta);
            telemetry.update();

            if (Math.abs(delta) < BALL_ALIGNMENT_TOLERANCE) {
                telemetry.addData("Align", "Centered!");
                telemetry.update();
                return true;
            }
            
            // Proportional control for alignment
            double turnDeg = Math.min(8, Math.abs(delta) / 15.0 * 5.0);
            double sign = Math.signum(delta);
            rotateDegreesEnhanced(turnDeg * sign);
            sleep(150);
        }
        
        return pipeline.hasBall();
    }

    // Enhanced color detection with better algorithms
    private int readColorValueFromSensorEnhanced() {
        long start = System.currentTimeMillis();
        int votesGreen = 0;
        int votesPurple = 0;
        int totalSamples = 0;
        
        // Collect more samples for better accuracy
        while (System.currentTimeMillis() - start < COLOR_SAMPLE_TIME_MS && opModeIsActive()) {
            int r = intakeColorSensor.red();
            int g = intakeColorSensor.green();
            int b = intakeColorSensor.blue();
            
            totalSamples++;
            
            // Enhanced color detection algorithms
            // Green detection: green dominant and above threshold
            if (g > r && g > b && g > COLOR_GREEN_THRESHOLD) {
                votesGreen++;
            }
            
            // Purple detection: red and blue significant, green relatively low
            if (r > 20 && b > 20 && g < Math.min(r, b) * COLOR_PURPLE_GREEN_RATIO) {
                votesPurple++;
            }
            
            // Additional purple detection: check for magenta-like colors
            if (r > 30 && b > 30 && Math.abs(r - b) < 20 && g < (r + b) / 3) {
                votesPurple++;
            }
            
            sleep(COLOR_SAMPLE_INTERVAL_MS);
        }
        
        telemetry.addData("Color", "Samples: %d, Green: %d, Purple: %d", totalSamples, votesGreen, votesPurple);
        
        // Require clear majority for confidence
        if (votesGreen > votesPurple && votesGreen > totalSamples * 0.3) return 1;
        if (votesPurple > votesGreen && votesPurple > totalSamples * 0.3) return 2;
        return 0; // Unknown/uncertain
    }

    // Enhanced navigation and goal shooting
    private void navigateToGoalEnhanced() {
        currentState = RobotState.NAVIGATING;
        logOperation("Starting navigation to goal");
        
        // Create navigation path based on field layout
        // This is a placeholder - customize for your field
        navigationPath.clear();
        navigationPath.add(36.0);  // Drive forward 36 inches
        navigationPath.add(-45.0); // Turn left 45 degrees
        navigationPath.add(24.0);  // Drive forward 24 inches
        
        for (Double command : navigationPath) {
            if (!opModeIsActive() || emergencyStop.get()) break;
            
            if (command > 0) {
                driveForwardInches(command);
            } else {
                rotateDegreesEnhanced(command);
            }
            
            sleep(200); // Brief pause between movements
        }
        
        logOperation("Navigation to goal completed");
    }
    
    private void shootCollectedBallsEnhanced() {
        currentState = RobotState.SHOOTING;
        int ballsToShoot = totalBallsCollected.get();
        
        if (ballsToShoot <= 0) {
            logOperation("No balls to shoot");
            return;
        }
        
        logOperation("Starting to shoot " + ballsToShoot + " balls");
        telemetry.addData("Shooting", "Balls: " + ballsToShoot);
        telemetry.update();

        // Enhanced flywheel control
        flywheel.setPower(FLYWHEEL_POWER_30IN);
        sleep(FLYWHEEL_SPINUP_MS);
        
        for (int i = 0; i < ballsToShoot && opModeIsActive(); i++) {
            telemetry.addData("Shot", (i + 1) + "/" + ballsToShoot);
            telemetry.update();
            
            // Enhanced feeding mechanism
            if (feederServo != null) {
                // Servo-based feeding
                feederServo.setPosition(0.2);
                sleep(300);
                feederServo.setPosition(0.8);
            } else {
                // Intake-based feeding with better timing
                intake.setPower(INTAKE_POWER);
                sleep(400);
                intake.setPower(0.0);
            }
            
            totalBallsShot.incrementAndGet();
            sleep(SHOT_DELAY_MS);
        }

        flywheel.setPower(0.0);
        logOperation("Shooting completed: " + totalBallsShot.get() + " balls shot");
    }

    // Enhanced utility methods
    private double getHeading() {
        try {
            Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return o.firstAngle;
        } catch (Exception e) {
            logOperation("IMU error: " + e.getMessage());
            return 0.0;
        }
    }

    private double normalizeAngle(double a) {
        while (a <= -180) a += 360;
        while (a > 180) a -= 360;
        return a;
    }

    private double angleDiff(double target, double current) {
        double diff = target - current;
        while (diff <= -180) diff += 360;
        while (diff > 180) diff -= 360;
        return diff;
    }
    
    // Enhanced error handling and logging
    private void handleError(String message, Exception e) {
        currentState = RobotState.ERROR;
        String errorMsg = message + (e != null ? ": " + e.getMessage() : "");
        logOperation("ERROR: " + errorMsg);
        
        telemetry.addData("ERROR", errorMsg);
        telemetry.update();
        
        // Stop all motors for safety
        stopMotors();
        intake.setPower(0.0);
        flywheel.setPower(0.0);
    }
    
    private void logOperation(String operation) {
        String timestamp = String.format("%.2f", runtime.seconds());
        String logEntry = "[" + timestamp + "s] " + operation;
        operationLog.append(logEntry).append("\n");
        
        // Keep log size manageable
        if (operationLog.length() > 2000) {
            operationLog.delete(0, 500);
        }
    }
    
    private void cleanup() {
        try {
            stopMotors();
            if (intake != null) intake.setPower(0.0);
            if (flywheel != null) flywheel.setPower(0.0);
            stopCamera();
            
            // Final telemetry with operation log
            telemetry.addLine("=== OPERATION LOG ===");
            telemetry.addData("Log", operationLog.toString());
            telemetry.update();
            
        } catch (Exception e) {
            telemetry.addData("Cleanup Error", e.getMessage());
            telemetry.update();
        }
    }

    // Enhanced computer vision pipeline
    public static class EnhancedObeliskAndBallPipeline extends OpenCvPipeline {

        private final int panelCount;
        private final Rect[] panelROIs;

        // Enhanced pipeline outputs with better thread safety
        private volatile int[] obeliskPanels;
        private volatile boolean ballDetected = false;
        private volatile double ballCenterX = 0.0;
        private volatile double ballCenterY = 0.0;
        private volatile double ballArea = 0.0;
        private volatile double lastImageWidth = 640;
        private volatile double lastImageHeight = 480;
        
        // Enhanced detection confidence
        private volatile double detectionConfidence = 0.0;
        private volatile int stableFrames = 0;
        private static final int MIN_STABLE_FRAMES = 3;

        // Enhanced processing mats
        private Mat hsv = new Mat();
        private Mat blurred = new Mat();
        private Mat maskGreen = new Mat();
        private Mat maskPurple = new Mat();
        private Mat maskBoth = new Mat();
        private Mat hierarchy = new Mat();
        private Mat morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));

        // Enhanced HSV thresholds with better color detection
        private final Scalar GREEN_LOWER = new Scalar(35/2.0, 60, 60);
        private final Scalar GREEN_UPPER = new Scalar(85/2.0, 255, 255);
        private final Scalar PURPLE_LOWER = new Scalar(125/2.0, 50, 50);
        private final Scalar PURPLE_UPPER = new Scalar(155/2.0, 255, 255);
        
        // Additional purple detection for magenta tones
        private final Scalar PURPLE2_LOWER = new Scalar(160/2.0, 40, 40);
        private final Scalar PURPLE2_UPPER = new Scalar(180/2.0, 255, 255);

        public EnhancedObeliskAndBallPipeline(int panels) {
            this.panelCount = Math.max(1, panels);
            panelROIs = new Rect[panelCount];
        }

        @Override
        public Mat processFrame(Mat input) {
            try {
                // Enhanced preprocessing
                Imgproc.GaussianBlur(input, blurred, new Size(7, 7), 0);
                Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

                // Enhanced color masking
                Core.inRange(hsv, GREEN_LOWER, GREEN_UPPER, maskGreen);
                
                // Two purple masks for better detection
                Mat maskPurple1 = new Mat();
                Mat maskPurple2 = new Mat();
                Core.inRange(hsv, PURPLE_LOWER, PURPLE_UPPER, maskPurple1);
                Core.inRange(hsv, PURPLE2_LOWER, PURPLE2_UPPER, maskPurple2);
                Core.bitwise_or(maskPurple1, maskPurple2, maskPurple);
                
                // Morphological operations to clean up masks
                Imgproc.morphologyEx(maskGreen, maskGreen, Imgproc.MORPH_CLOSE, morphKernel);
                Imgproc.morphologyEx(maskPurple, maskPurple, Imgproc.MORPH_CLOSE, morphKernel);
                
                Core.bitwise_or(maskGreen, maskPurple, maskBoth);

                // Enhanced obelisk detection
                detectObeliskPanels(input);
                
                // Enhanced ball detection
                detectBalls(input);
                
                // Clean up temporary mats
                maskPurple1.release();
                maskPurple2.release();
                
            } catch (Exception e) {
                // Handle processing errors gracefully
                telemetry.addData("Pipeline Error", e.getMessage());
            }
            
            return input;
        }
        
        private void detectObeliskPanels(Mat input) {
            int width = input.width();
            int height = input.height();
            lastImageWidth = width;
            lastImageHeight = height;

            // Enhanced ROI calculation with better positioning
            int roiTop = (int)(height * 0.10);
            int roiHeight = (int)(height * 0.20);
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
                
                double area = r.width * r.height;
                double threshold = Math.max(15, area * 0.04); // Increased threshold for better reliability
                
                if (greenCount > purpleCount && greenCount > threshold) {
                    panels[i] = 1;
                    Imgproc.rectangle(input, r.tl(), r.br(), new Scalar(0,255,0), 3);
                } else if (purpleCount > greenCount && purpleCount > threshold) {
                    panels[i] = 2;
                    Imgproc.rectangle(input, r.tl(), r.br(), new Scalar(255,0,255), 3);
                } else {
                    panels[i] = 0;
                    Imgproc.rectangle(input, r.tl(), r.br(), new Scalar(0,0,255), 2);
                }
                
                subGreen.release();
                subPurple.release();
            }
            obeliskPanels = panels;
        }
        
        private void detectBalls(Mat input) {
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(maskBoth.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double bestArea = 0;
            Point bestCenter = null;
            
            for (MatOfPoint cnt : contours) {
                double area = Imgproc.contourArea(cnt);
                if (area < BALL_DETECTION_MIN_AREA) continue;
                
                // Enhanced circularity check
                double perimeter = Imgproc.arcLength(new MatOfPoint(cnt.toArray()), true);
                double circularity = 4 * Math.PI * area / (perimeter * perimeter);
                
                if (circularity < 0.3) continue; // Filter non-circular objects
                
                Moments m = Imgproc.moments(cnt);
                if (m.get_m00() == 0) continue;
                
                double cx = m.get_m10() / m.get_m00();
                double cy = m.get_m01() / m.get_m00();

                if (area > bestArea) {
                    bestArea = area;
                    bestCenter = new Point(cx, cy);
                }
            }

            if (bestCenter != null && bestArea > BALL_DETECTION_MIN_AREA) {
                // Enhanced stability check
                if (Math.abs(bestCenter.x - ballCenterX) < 10 && Math.abs(bestArea - ballArea) < 100) {
                    stableFrames++;
                } else {
                    stableFrames = 0;
                }
                
                if (stableFrames >= MIN_STABLE_FRAMES) {
                    ballDetected = true;
                    ballCenterX = bestCenter.x;
                    ballCenterY = bestCenter.y;
                    ballArea = bestArea;
                    detectionConfidence = Math.min(1.0, stableFrames / 10.0);
                    
                    // Enhanced visualization
                    Imgproc.circle(input, bestCenter, 8, new Scalar(255,255,0), -1);
                    Imgproc.circle(input, bestCenter, 12, new Scalar(255,255,0), 2);
                }
            } else {
                ballDetected = false;
                stableFrames = 0;
                detectionConfidence = 0.0;
            }
            
            // Enhanced debugging information
            Imgproc.putText(input, "Panels: " + panelsToString(obeliskPanels), 
                    new Point(8,25), Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255,255,255), 2);
            
            if (ballDetected) {
                Imgproc.putText(input, String.format("Ball: (%.0f,%.0f) A:%.0f C:%.2f", 
                        ballCenterX, ballCenterY, ballArea, detectionConfidence), 
                        new Point(8,50), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255,255,255), 2);
            }
        }

        private String panelsToString(int[] panels) {
            if (panels == null) return "[]";
            StringBuilder sb = new StringBuilder();
            sb.append("[");
            for (int i = 0; i < panels.length; i++) {
                sb.append(panels[i]);
                if (i < panels.length - 1) sb.append(",");
            }
            sb.append("]");
            return sb.toString();
        }

        // Enhanced thread-safe getters
        public int[] getObeliskPanels() {
            return obeliskPanels == null ? new int[0] : obeliskPanels.clone();
        }
        
        public boolean hasBall() {
            return ballDetected && detectionConfidence > 0.5;
        }
        
        public double getLatestBallCenterX() {
            return ballCenterX;
        }
        
        public double getLatestBallCenterY() {
            return ballCenterY;
        }
        
        public double getBallArea() {
            return ballArea;
        }
        
        public double getDetectionConfidence() {
            return detectionConfidence;
        }
        
        public double getImageWidth() {
            return lastImageWidth;
        }
        
        public double getImageHeight() {
            return lastImageHeight;
        }
    }

    // Configuration and field-specific settings
    private static class FieldConfiguration {
        // Field dimensions (customize for your field)
        public static final double FIELD_WIDTH_INCHES = 144.0;
        public static final double FIELD_LENGTH_INCHES = 144.0;
        
        // Starting positions
        public static final double START_X = 0.0;
        public static final double START_Y = 0.0;
        public static final double START_HEADING = 0.0;
        
        // Goal positions
        public static final double GOAL_X = 72.0;
        public static final double GOAL_Y = 36.0;
        
        // Navigation waypoints
        public static final double[] NAVIGATION_WAYPOINTS = {
            36.0, -45.0, 24.0  // Drive forward, turn left, drive forward
        };
        
        // Vision calibration
        public static final double CAMERA_HEIGHT_INCHES = 12.0;
        public static final double CAMERA_ANGLE_DEGREES = 15.0;
        public static final double BALL_DETECTION_DISTANCE_INCHES = 24.0;
    }
    
    // Performance monitoring
    private static class PerformanceMonitor {
        private long startTime;
        private int operationsCount;
        private double totalDistance;
        private int successfulOperations;
        private int failedOperations;
        
        public PerformanceMonitor() {
            startTime = System.currentTimeMillis();
            operationsCount = 0;
            totalDistance = 0.0;
            successfulOperations = 0;
            failedOperations = 0;
        }
        
        public void recordOperation(boolean success, double distance) {
            operationsCount++;
            totalDistance += distance;
            if (success) {
                successfulOperations++;
            } else {
                failedOperations++;
            }
        }
        
        public double getSuccessRate() {
            return operationsCount > 0 ? (double) successfulOperations / operationsCount : 0.0;
        }
        
        public double getAverageSpeed() {
            long elapsed = System.currentTimeMillis() - startTime;
            return elapsed > 0 ? totalDistance / (elapsed / 1000.0) : 0.0;
        }
        
        public String getSummary() {
            return String.format("Ops: %d, Success: %.1f%%, Speed: %.1f in/s", 
                    operationsCount, getSuccessRate() * 100, getAverageSpeed());
        }
    }
}
