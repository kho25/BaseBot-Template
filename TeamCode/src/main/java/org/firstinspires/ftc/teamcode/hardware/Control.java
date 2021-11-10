package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class Control extends Devices {


    // MOTORS

    public static class motor {

        // moves motor based on power
        // 1.0: forwards
        // 0.0: brake
        // -1.0 backwards
        public static void moveMotor(DcMotor motor, double power) {
            double speed = Range.clip(power, -1.0, 1.0);
            motor.setPower(speed);
        }

        public static void linearSlideSetPosition(DcMotor motor, int targetPosition) {
            double power = 1.0;
            if (Encoders.getMotorEnc(motor) > targetPosition) power = -1.0;

            motor.setTargetPosition(targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motor.setPower(power);
            while (motor.isBusy()) {
            }
            motor.setPower(0);

            return;
        }
    }

    public static class drive {

        public static void configureDriveMotors() {
            rightFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
            rightBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);

            leftFrontDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftBackDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightFrontDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightBackDriveMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            leftFrontDriveMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftBackDriveMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightFrontDriveMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightBackDriveMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        // drive with each side correlating to gamepad sticks
        public static void tankDrive(double leftPwr, double rightPwr) {
            double leftPower = Range.clip(leftPwr, -1.0, 1.0);
            double rightPower = Range.clip(rightPwr, -1.0, 1.0);

            leftFrontDriveMotor.setPower(leftPower);
            leftBackDriveMotor.setPower(leftPower);
            rightFrontDriveMotor.setPower(rightPower);
            rightBackDriveMotor.setPower(rightPower);
        }

        // tankDrive + mecanum drive
        public static void tankanumDrive(double rightPwr, double leftPwr, double lateralPwr) {
            double leftFrontPower = Range.clip(leftPwr - lateralPwr, -1.0, 1.0);
            double leftBackPower = Range.clip(leftPwr + lateralPwr, -1.0, 1.0);
            double rightFrontPower = Range.clip(rightPwr - lateralPwr, -1.0, 1.0);
            double rightBackPower = Range.clip(rightPwr + lateralPwr, -1.0, 1.0);

            leftFrontDriveMotor.setPower(leftFrontPower);
            leftBackDriveMotor.setPower(leftBackPower);
            rightFrontDriveMotor.setPower(rightFrontPower);
            rightBackDriveMotor.setPower(rightBackPower);
        }

        public static void turnWithEncoder(double power) {
            Encoders.driveRunUsingEncoder();

            leftFrontDriveMotor.setPower(power);
            leftBackDriveMotor.setPower(power);
            rightFrontDriveMotor.setPower(-power);
            rightBackDriveMotor.setPower(-power);
        }

        public static void stopPower() {
            leftFrontDriveMotor.setPower(0);
            leftBackDriveMotor.setPower(0);
            rightFrontDriveMotor.setPower(0);
            rightBackDriveMotor.setPower(0);
        }
    }

    public static class auto {
        public static double armEncoderToAngle(double encoderReading) {
            return encoderReading/ConstantVariables.K_ARM_ROTATE_PPR* 360*ConstantVariables.K_ARM_GEAR_RATIO;
        }

        public static void spinCarousel(DcMotor motor) {
            int shiftValue = 1000; // increase/decrease depending on how long you want the motor to spin
            int targetPosition = Encoders.getMotorEnc(motor) + shiftValue;

            motor.setTargetPosition(targetPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motor.setPower(1.0);
            while (motor.isBusy()) {
            }
            motor.setPower(0);

            Encoders.resetMotorEnc(motor);
        }

//        double power = 1.0;
//        int targetPosition = stageIndex * 500;
//            if (Encoders.getMotorEnc(motor) > targetPosition) power = -1.0;
//
//            motor.setTargetPosition(targetPosition);
//            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            motor.setPower(power);
//            while (motor.isBusy()) { }
//            motor.setPower(0);
//
//            return;

        // old method of autonomous drive, only use in iterative autos
//        public static boolean drive(double inches, double power) {
//            double TARGET_ENC = ConstantVariables.K_PPIN_DRIVE * inches;
//            double left_speed = power;
//            double right_speed = power;
//            double error = -Encoders.getMotorEnc(Devices.leftFrontDriveMotor) - Encoders.getMotorEnc(Devices.rightFrontDriveMotor);
//
//            error /= ConstantVariables.K_DRIVE_ERROR_P;
//            left_speed += error;
//            right_speed -= error;
//
//            left_speed = Range.clip(left_speed, -1, 1);
//            right_speed = Range.clip(right_speed, -1, 1);
//            leftFrontDriveMotor.setPower(left_speed);
//            leftBackDriveMotor.setPower(left_speed);
//            rightFrontDriveMotor.setPower(right_speed);
//            rightBackDriveMotor.setPower(right_speed);
//
//            if (Math.abs(Encoders.getMotorEnc(Devices.rightFrontDriveMotor)) >= TARGET_ENC) {
//                drive.stopPower();
//                return true;
//            } else return false;
//        }

        // turns a specific amount of degrees
        // power: the speed to move (1.0 to -1.0)
        // degrees: the amount of degees to turn
        // returns whether it has reached the target degrees
        // old method of autonomous turn, only use in iterative autos
//        public static boolean turn(double power, double degrees) {
//            double TARGET_ENC = Math.abs(ConstantVariables.K_PPDEG_DRIVE * degrees);
//
//            double speed = Range.clip(power, -1, 1);
//            leftFrontDriveMotor.setPower(-speed);
//            leftBackDriveMotor.setPower(-speed);
//            rightFrontDriveMotor.setPower(speed);
//            rightBackDriveMotor.setPower(speed);
//
//            if (Math.abs(Encoders.getMotorEnc(rightFrontDriveMotor)) >= TARGET_ENC) {
//                leftFrontDriveMotor.setPower(0);
//                leftBackDriveMotor.setPower(0);
//                rightFrontDriveMotor.setPower(0);
//                rightBackDriveMotor.setPower(0);
//                return true;
//            } else return false;
//        }

        public static void moveWithEncoder(double inches, double speed) {
            double conversion = ConstantVariables.COUNTS_PER_INCH * ConstantVariables.BIAS;
            int move = (int) (Math.round(inches * conversion));

            leftFrontDriveMotor.setTargetPosition(leftFrontDriveMotor.getCurrentPosition() + move);
            leftBackDriveMotor.setTargetPosition(leftBackDriveMotor.getCurrentPosition() + move);
            rightFrontDriveMotor.setTargetPosition(rightFrontDriveMotor.getCurrentPosition() + move);
            rightBackDriveMotor.setTargetPosition(rightBackDriveMotor.getCurrentPosition() + move);

            Encoders.driveRunToPosition();

            leftFrontDriveMotor.setPower(speed);
            leftBackDriveMotor.setPower(speed);
            rightFrontDriveMotor.setPower(speed);
            rightBackDriveMotor.setPower(speed);

            while (leftFrontDriveMotor.isBusy() && leftBackDriveMotor.isBusy() && rightFrontDriveMotor.isBusy() && rightBackDriveMotor.isBusy()) {
            }

            drive.stopPower();
            return;
        }

        // degrees always positive, speedDirection to speed and direction
        public static void turnWithGyro(double degrees, double speedDirection) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double yaw = -angles.firstAngle;

            double first;
            double second;

            if (speedDirection > 0) {
                if (degrees > 10) {
                    first = (degrees - 10) + devertDegrees(yaw);
                    second = degrees + devertDegrees(yaw);
                } else {
                    first = devertDegrees(yaw);
                    second = degrees + devertDegrees(yaw);
                }
            } else {
                if (degrees > 10) {
                    first = devertDegrees(-(degrees - 10) + devertDegrees(yaw));
                    second = devertDegrees(-degrees + devertDegrees(yaw));
                } else {
                    first = devertDegrees(yaw);
                    second = devertDegrees(-degrees + devertDegrees(yaw));
                }
            }

//            double firsta = convertDegrees(first - 5); // 175
//            double firstb = convertDegrees(first + 5); // -175

//            drive.turnWithEncoder(speedDirection);
//
//            if (Math.abs(firsta - firstb) < 11) {
//                while (!(firsta < yaw && yaw < firstb)) {//within range?
//                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////                    gravity = imu.getGravity();
//                    yaw = -angles.firstAngle;
//                }
//            } else {
//                while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb))) {//within range?
//                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
////                    gravity = imu.getGravity();
//                    yaw = -angles.firstAngle;
//                }
//            }

            double seconda = convertDegrees(second - 5); // 175
            double secondb = convertDegrees(second + 5); // -175

            drive.turnWithEncoder(speedDirection);

            if (Math.abs(seconda - secondb) < 11) {
                while (!(seconda < yaw && yaw < secondb)) {//within range?
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    yaw = -angles.firstAngle;
                }
                while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb))) {//within range?
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    yaw = -angles.firstAngle;
                }
                drive.stopPower();
            }

            Encoders.driveResetEncs();
        }


        /*
        This function uses the encoders to strafe left or right.
        Negative input for inches results in left strafing.
         */
        public static void strafeToPosition(double inches, double speed) {
            double meccyBias = 1.1;
            int move = (int) (Math.round(inches * ConstantVariables.COUNTS_PER_INCH * meccyBias));

            leftFrontDriveMotor.setTargetPosition(leftFrontDriveMotor.getCurrentPosition() - move);
            leftBackDriveMotor.setTargetPosition(leftBackDriveMotor.getCurrentPosition() + move);
            rightFrontDriveMotor.setTargetPosition(rightFrontDriveMotor.getCurrentPosition() + move);
            rightBackDriveMotor.setTargetPosition(rightBackDriveMotor.getCurrentPosition() - move);

            Encoders.driveRunToPosition();

            leftFrontDriveMotor.setPower(speed);
            leftBackDriveMotor.setPower(speed);
            rightFrontDriveMotor.setPower(-speed);
            rightBackDriveMotor.setPower(-speed);

            while (leftFrontDriveMotor.isBusy() && leftBackDriveMotor.isBusy() && rightFrontDriveMotor.isBusy() && rightBackDriveMotor.isBusy()) {

            }

            drive.stopPower();
            return;
        }

        /*
        These functions are used in the turnWithGyro function to ensure inputs
        are interpreted properly.
        */
        public static double devertDegrees(double degrees) {
            if (degrees < 0) {
                degrees = degrees + 360;
            }
            return degrees;
        }

        public static double convertDegrees(double degrees) {
            if (degrees > 179) {
                degrees = -(360 - degrees);
            } else if (degrees < -180) {
                degrees = 360 + degrees;
            } else if (degrees > 360) {
                degrees = degrees - 360;
            }
            return degrees;
        }


        // Tensorflow


        private static VuforiaLocalizer vuforia;

        private static TFObjectDetector tfod;

        public static void initTF(String asset, String[] labels, double magnification, HardwareMap hardwareMap) {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = ConstantVariables.VUFORIA_KEY;
            parameters.cameraName = Devices.webcam;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.

            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 320;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(asset, labels);
            tfod.activate();
            tfod.setZoom(magnification, 16.0 / 9.0);
        }

        public static List<Recognition> tfGetRecognitions() {
            return tfod.getUpdatedRecognitions();
        }

        public static Recognition getDuck(Telemetry telemetry) {
            ElapsedTime timer = new ElapsedTime();
            boolean duckFound = false;
            Recognition duck = null;
            while (!duckFound) {
                if (timer.seconds() <= 5) {
                    List<Recognition> updatedRecognitions = tfGetRecognitions();
                    if (updatedRecognitions != null) {
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            if (recognition.getLabel().equals("Duck")) {
                                duck = recognition;
                                duckFound = true;

                            }
                            i++;
                        }
                    }
                    telemetry.update();
                } else {
                    duckFound = true;
                }
            }
            return duck;
        }

        public static int getDuckPositionIndexThree(double angle) {
            if (angle >= 10.0) return 2;
            else if (angle <= -10.0) return 0;
            else return 1;
        }

        public static int getDuckPositionIndexTwo(double angle) {
            if (angle >= 10.0) return 1;
            else if (angle <= -10.0) return 0;
            else return 2;
        }
    }

    public static class servo {

        // sets servo position based on pos
        // 1.0: highest position
        // 0.0: lowest position
        public static void setServoPosition(Servo servo, double pos) {
            double position = Range.clip(pos, 0.0, 1.0);
            servo.setPosition(position);
        }
    }

    public static class sensor {

        public static void initGyro() {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu.initialize(parameters);
        }

        public static boolean checkBlackColor(int red, int blue) {
            return blue > (3.0 / 4) * red;
        }

        public static boolean checkGreenColor(int green, int blue, int red) {
            if (green > blue + red) {
                return true;
            } else return false;
        }

        public static boolean checkBlueColor(int green, int blue, int red) {
            if (blue > green + red) {
                return true;
            } else return false;
        }

        public static boolean checkRedColor(int green, int blue, int red) {
            if (red > blue + green) {
                return true;
            } else return false;
        }
    }

    public static class pid {
        private ElapsedTime runtime;
        private double oldError;
        private double oldIntegral;
        private double oldTime;

        private double p, i, d, integralReset;

        public pid(double kP, double kI, double kD, double integralResetThreshold) {
            runtime = new ElapsedTime();
            oldError = 0;
            oldIntegral = 0;
            oldTime = 0;
            p = kP;
            i=kI;
            d=kD;
            integralReset = integralResetThreshold;
        }

        public double getPower(double goalPosition, double currentPosition) {

            double integral;
            double derivative;


            double error = goalPosition-currentPosition;
            double dT = runtime.milliseconds()-oldTime;

            if(Math.abs(error)>integralReset)
                oldIntegral = 0;

            integral = oldIntegral + error * dT;
            derivative = (error-oldError);

            oldError = error;
            oldIntegral = integral;
            oldTime = runtime.milliseconds();

            return p*error + i*integral + d*derivative;
        }
    }
}
