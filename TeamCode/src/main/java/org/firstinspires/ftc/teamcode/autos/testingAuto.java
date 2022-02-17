package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.hardware.Control.motor.extendArm;

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
    /*
    * Traj1: to alliance shipping hub
    * Traj2: to freight hub
    * Traj3: to alliance shipping hub
    * Traj4: to freight hub
    * */
    boolean armExtension = false;
    boolean intake = false;
    int slidePositon:

    public void init(){
        Devices.initDevices(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(10, 10, Math.toRadians(90));
        drive.setPoseEstimate(startPos);
        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(20, 20), Math.toRadians(90))
                .addDisplacementMarker(10, () -> { //displacement in inches
                    armExtension = true;
                })
                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(10)
                .addDisplacementMarker(20, () -> {
                    intake = true;
                })
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(10)
                .build();

        int num = 0;

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

    }

    public void start() {

        drive.followTrajectoryAsync(traj1);

        return;
    }
    public void loop() {
    int num = 0;
        if (num==0) {
            drive.update();
            if (!drive.isBusy()) {
                drive.followTrajectoryAsync(traj2);
                armExtension = false;
                num++;
            }
        }
        else if (num == 1) {
            drive.update();
            if(!drive.isBusy()){
                wait(5);
                drive.followTrajectoryAsync(traj3);
                intake = false;
                num++;
            }
        }

        if(armExtension){
            switch(slidePositon){
                case 1:
                    extendArm(100);
                    break;
                case 2:

            }
        } else{
            //de-extend arm
        }

        if (intake){
            Control.motor.intake(1.0);
        } else {
            Control.motor.intake(0.0);
        }

        if (motor.getCurrentPosition()<100)
            motor.setPower(1);
        else
            motor.setPower(0);



    }
}
