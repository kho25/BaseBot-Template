package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.Encoders;

import static org.firstinspires.ftc.teamcode.hardware.Devices.armLiftMotor;

@TeleOp
public class PidSample extends BaseRobot {
    Control.pid armController;
    double currentAngle;
    double output;
    public void init() {
        super.init();
        armController  = new Control.pid(0.11, 0, 0.05, 0.01);
        Encoders.resetMotorEnc(armLiftMotor);
    }

    public void loop() {
        super.loop();
        if(gamepad1.b) {
            currentAngle = Control.auto.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor));
            output = armController.getPower(60, (currentAngle));
            Control.motor.moveMotor(armLiftMotor, output);
        } else {
            Control.motor.moveMotor(armLiftMotor, 0);
        }
        if(gamepad1.x) {
            Encoders.resetMotorEnc(armLiftMotor);
        }

        telemetry.addData("arm 1 angle: ", Control.auto.armEncoderToAngle(Encoders.getMotorEnc(armLiftMotor)));

    }
}
