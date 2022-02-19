package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.hardware.Control.motor.extendArm;
import static org.firstinspires.ftc.teamcode.hardware.Control.servo.setServoPosition;
import static org.firstinspires.ftc.teamcode.hardware.Devices.armOuttakeServo;
import static org.firstinspires.ftc.teamcode.hardware.Devices.linearSlideMotor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.CV.IndicatorPositionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.*;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;


/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class blueWarehouseAuto extends OpMode {
    SampleMecanumDrive drive;
    Trajectory trajStart, trajWarehouseStart, trajAlliShip, trajPark, trajWarehouse;
    ElapsedTime runtime;
    double timeMarker;
    /*
     * Traj1: to alliance shipping hub
     * Traj2: to warehouse
     * Traj3: to alliance shipping hub
     * Traj4: park
     * */
    boolean armExtension, intake;
    int slidePositon;

    OpenCvWebcam webcam;
    IndicatorPositionPipeline pipeline;

    public void initLoop(){
        slidePositon = pipeline.getAnalysis();
    }
    public void init(){
        Devices.initDevices(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        runtime = new ElapsedTime();
        slidePositon = 1;  //TODO: change based on duck location
        armExtension = false;
        intake = false;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new IndicatorPositionPipeline(telemetry);
        webcam.setPipeline(pipeline);
        //webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }

            //@Override
            public void onError(int errorCode) {
                telemetry.addLine("Yikes the camera couldn't open");
            }
        });

        Pose2d startPos = new Pose2d(10, 61, Math.toRadians(-90));
        drive.setPoseEstimate(startPos);

        //to alliance shipping hub from starting point
        Trajectory trajStart = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-11, 50), Math.toRadians(-90)) //TODO: tune positions
                .addDisplacementMarker(10, () -> { //displacement in inches
                    armExtension = true;
                })
                .build();
        //to warehouse from post preloaded starting point
        Trajectory trajWarehouseStart = drive.trajectoryBuilder(trajStart.end())
                .splineTo(new Vector2d(50, 50), Math.toRadians(0))
                .addDisplacementMarker(20, () -> {
                    intake = true;
                })
                .build();
        //to alliance shipping hub
        Trajectory trajAlliShip = drive.trajectoryBuilder(trajWarehouseStart.end())
                .splineTo(new Vector2d(40, 16), Math.toRadians(-90))
                .addDisplacementMarker(20, () -> {
                    armExtension = true;
                })
                .build();
        //park at warehouse
        Trajectory trajPark = drive.trajectoryBuilder(trajAlliShip.end())
                .splineTo(new Vector2d(55, 55), Math.toRadians(-90))
                .addDisplacementMarker(10, () -> {
                    intake = false;
                    armExtension = false;
                })
                .build();
        //to warehouse
        Trajectory trajWarehouse = drive.trajectoryBuilder(trajAlliShip.end())
                .splineTo(new Vector2d(20, -20), Math.toRadians(-90))
                .addDisplacementMarker(20, () -> {
                    intake = true;
                })
                .build();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

    }


    public void start() {

        drive.followTrajectoryAsync(trajStart);
        armExtension = true;
        if(runtime.seconds() > 5) {
            armExtension = false; //TODO: tune timing
        }

        return;
    }

    public void loop() {
        //testing with red warehouse side coordinates
        int num = 0;
        if (num==0) {
            drive.update();
            if (!drive.isBusy() && runtime.seconds()-timeMarker > 5) {
                drive.followTrajectoryAsync(trajWarehouseStart);
                armExtension = false;
                num++;
            }
        }
        else if (num == 1) {
            drive.update();
            if(!drive.isBusy()){
                drive.followTrajectoryAsync(trajAlliShip);
                timeMarker = runtime.seconds();
                num++;

            }
        }
        else if (num==2) {
            if (!drive.isBusy() && runtime.seconds()-timeMarker > 5) {
                drive.followTrajectoryAsync(trajWarehouse);
                intake = false;

                //TODO: find cutoff time
                if(runtime.seconds() > 22){
                    num++;
                }
                else num--;

            }
        }
        else if (num==3){
            if(!drive.isBusy()) {
                drive.followTrajectoryAsync(trajPark);
                num++;
            }
        }
        else{

        }

        // extend arm and dump ducks
        if(armExtension){
            switch(slidePositon){
                case 1:
                    extendArm(ConstantVariables.SLIDE_POS_ONE);
                    setServoPosition(armOuttakeServo, ConstantVariables.SERVO_OUTTAKE_POSITION);
                    break;
                case 2:
                    extendArm(ConstantVariables.SLIDE_POS_TWO);
                    setServoPosition(armOuttakeServo, ConstantVariables.SERVO_OUTTAKE_POSITION);
                    break;
                case 3:
                    extendArm(ConstantVariables.SLIDE_POS_THREE);
                    setServoPosition(armOuttakeServo, ConstantVariables.SERVO_OUTTAKE_POSITION);
                    break;
            }
        }
        else{
            //de-extend arm
            if(Encoders.getMotorEnc(linearSlideMotor) > 0 ) linearSlideMotor.setPower(-1.0);
            setServoPosition(armOuttakeServo, ConstantVariables.SERVO_INTAKE_POSITION); //reset servo position

        }

        if (intake){
            Control.motor.intake(1.0);
        } else {
            Control.motor.intake(0.0);
        }

    }
}
