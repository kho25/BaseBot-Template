package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.hardware.Encoders.resetMotorEnc;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;

@TeleOp
public class teleOpBasic extends BaseRobot {

int armExtension = 0;

    public void init(){
    super.init();
        resetMotorEnc(Devices.linearSlideMotor);
    //Devices.linearSlideMotor.set;
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        /*
        * 1. Driving
        * 2. Intake
        * 3. Outtake
        * 4. Ducks
        * */


        Control.drive.tankanumDrive(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.left_stick_x); // how to move

        //intake
        if(gamepad1.left_bumper){
            Control.motor.intake(1.0);
        }
        else { Control.motor.intake(0);}

        //outtake
        if (gamepad1.x && Devices.linearSlideMotor.getCurrentPosition()<1000){
            Control.motor.moveMotor(Devices.linearSlideMotor, 0.2);
        }
        else if(gamepad1.y && Devices.linearSlideMotor.getCurrentPosition()> 20){ //negative?
            Control.motor.moveMotor(Devices.linearSlideMotor, -0.2);
        }
        else{ Devices.linearSlideMotor.setPower(0);}
        telemetry.addData("slide position: ", Devices.linearSlideMotor.getCurrentPosition());

        //dump freight
        if(gamepad1.a){
            Devices.armOuttakeServo.setPosition(0.5);
        } else {
            Devices.armOuttakeServo.setPosition(1);//tune positions
        }
        telemetry.addData("outtake servo pose: ", Devices.armOuttakeServo.getPosition());

        //ducks
        if(gamepad1.right_bumper){
            Devices.duckServo.setPower(1);
        }
        else {
            Devices.duckServo.setPower(0);
        }

    }
}
