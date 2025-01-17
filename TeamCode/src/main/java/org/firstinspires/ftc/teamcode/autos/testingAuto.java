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

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.*;

import java.util.List;


/**
 * To understand how Road Runner works and setting it up: https://learnroadrunner.com/
 */


@Autonomous

public class testingAuto extends OpMode {
    SampleMecanumDrive drive;
    Trajectory traj1, traj2, traj3;
    ElapsedTime timer;
    Trajectory trajStart, trajWarehouseStart, trajAlliShip, trajPark, trajWarehouse;
    ElapsedTime runtime;
    double timeMarker;
    /*
    * Traj1: to alliance shipping hub
    * Traj2: to warehouse
    * Traj3: to alliance shipping hub
    * Traj4: park
    * */
    boolean armExtension = false;
    boolean intake = false;
    int slidePositon;

    public void init(){
        Devices.initDevices(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        timer = new ElapsedTime();
        runtime = new ElapsedTime();
        slidePositon = 0;  //TODO: change based on duck location
        armExtension = false;
        intake = false;


        Pose2d startPos = new Pose2d(10, 10, Math.toRadians(90));
        drive.setPoseEstimate(startPos);

        //to alliance shipping hub from starting point
        Trajectory trajStart = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-11, -50), Math.toRadians(90)) //TODO: tune positions
                .addDisplacementMarker(10, () -> { //displacement in inches
                    armExtension = true;
                })
                .build();
        //to freight hub from post preloaded starting point
        Trajectory trajWarehouseStart = drive.trajectoryBuilder(trajStart.end())
                .splineTo(new Vector2d(50, -50), Math.toRadians(0))
                .addDisplacementMarker(20, () -> {
                    intake = true;
                })
                .build();
        //to alliance shipping hub
        Trajectory trajAlliShip = drive.trajectoryBuilder(trajWarehouseStart.end())
                .splineTo(new Vector2d(40, -16), Math.toRadians(90))
                .addDisplacementMarker(20, () -> {
                    armExtension = true;
                })
                .build();
        //park at freight hub
        Trajectory trajPark = drive.trajectoryBuilder(trajAlliShip.end())
                .splineTo(new Vector2d(55, -55), Math.toRadians(90))
                .addDisplacementMarker(10, () -> {
                    intake = false;
                    armExtension = false;
                })
                .build();
        //to freight hub
        Trajectory trajWarehouse = drive.trajectoryBuilder(trajAlliShip.end())
                .splineTo(new Vector2d(20, 20), Math.toRadians(90))
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
                timer.reset();
                drive.followTrajectoryAsync(traj3);
                drive.followTrajectoryAsync(trajAlliShip);
                timeMarker = runtime.seconds();
                num++;

            }
        }
        else if (num==2) {
            drive.update();
            if (timer.seconds() > 5) {

                if (!drive.isBusy() && runtime.seconds() - timeMarker > 5) {
                    drive.followTrajectoryAsync(trajWarehouse);
                    intake = false;

                    //TODO: find cutoff time
                    if (runtime.seconds() > 22) {
                        num++;
                    } else num--;

                }
            } else if (num == 3) {
                if (!drive.isBusy()) {
                    drive.followTrajectoryAsync(trajPark);
                    num++;
                }
            } else {

            }
        }

            // extend arm and dump ducks
            if (armExtension) {
                switch (slidePositon) {
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
            } else {
                //de-extend arm
                if (Encoders.getMotorEnc(linearSlideMotor) > 0) linearSlideMotor.setPower(-1.0);
                setServoPosition(armOuttakeServo, ConstantVariables.SERVO_INTAKE_POSITION); //reset servo position

            }

            if (intake) {
                Control.motor.intake(1.0);
            } else {
                Control.motor.intake(0.0);
            }

        


    }
}
