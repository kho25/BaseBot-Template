package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.hardware.Control.motor.extendArm;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

public class DuckAutoRed extends OpMode {
    SampleMecanumDrive drive;
    Trajectory traj1, traj2, traj3;
    ElapsedTime timer;
    /*
     * Traj1: to alliance shipping hub
     * Traj2: to freight hub
     * Traj3: to alliance shipping hub
     * Traj4: to freight hub
     * */
    boolean armExtension = false;
    boolean intake = false;
    int slidePositon;
    String parkLocation = "warehouse";
    public void init() {
        Devices.initDevices(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        timer = new ElapsedTime();

        Pose2d startPos = new Pose2d(-34, -65, Math.toRadians(90));
        drive.setPoseEstimate(startPos);
        //go to spin duck
        Trajectory traj1 = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(-64, -65), Math.toRadians(180)
                .build();
        //to warehouse
        Trajectory warehousePark = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d (48, -48), Math.toRadians(0))
                .addDisplacementMarker(20, () -> {
                    intake = true;
                .build();
        //to depot
        Trajectory depotPark = drive.trajectoryBuilder(traj1.end())
            .strafeLeft(40)
            .splineTo(new Vector2d(-70,-34), Math.toRadians(90))
            .build();

        Trajectory parkTraj = drive.trajectoryBuilder(traj1.end())
            if (parkLocation.equals("warehouse")) {
                parkTraj = warehousePark;
            } else
                parkTraj = depotPark;

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

    }

    public void start() {

        drive.followTrajectoryAsync(traj1);

        return;
    }

    public void loop() {
        int num = 0;
        if (num == 0) {
            drive.update();
            if (!drive.isBusy()) {
                boolean duckServo = true;
                Trajectory parkTraj;
                drive.followTrajectoryAsync(parkTraj);
                num++;
            }
        } else if (num == 1) {
            drive.update();
            if (timer.seconds() > 5) {
                intake = false;
                num++;
            }
        }

        if (armExtension) {
            switch (slidePositon) {
                case 1:
                    extendArm(100);
                    break;
                case 2:

            }
        } else {
            //de-extend arm
        }

        if (intake) {
            Control.motor.intake(1.0);
        } else {
            Control.motor.intake(0.0);
        }
    }
}

    private void build() {
    }

    private void build() {
    }

    private void build() {
    }

    private void build() {
    }