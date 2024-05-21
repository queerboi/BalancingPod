package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "pod")
public class BalancingPod extends LinearOpMode {
    DcMotorEx motor;
    IMU gyro;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                90,
                                0,
                                -45,
                                0  // acquisitionTime, not used
                        )
                )
        );
        gyro.initialize(params);

        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(calcPower());

            telemetry.addLine();
            telemetry.addData("Power", motor.getPower());
            telemetry.addData("Pitch", gyro.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
            telemetry.addData("Pitch Velocity", gyro.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate);
            telemetry.update();

            sleep(10);
        }
    }

    //TODO perhaps speed up loop, check hardware wiggle room

    public double calcPower() {
        //tilt forward (positive pitch) = accelerate forwards
        //ACCELERATION is proportional to pitch

        //positive end tipping down = positive pitch
        double pitch = gyro.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS);

        //positive end tipping down = positive pitch velocity
        double pitchVel = gyro.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate;

        //LONGEST BALANCE SO FAR:
        //return motor.getPower() + k0 * pitch + k1 * pitchVel + km * motor.getPower();
        //k0 = 0.9, k1 = 0.1, km = -0.01

        //motor.setVelocity(1); //angularRate is ticks per second

        if (pitchVel < 0 && pitch > -0.05) {
            if (pitch > 0) {
                telemetry.addLine("pitching BACK, pitched FORWARD");
            } else {
                telemetry.addLine("pitching BACK, pitched BACK");
            }
            return motor.getPower() + 0.45 * pitch;
        } else if (pitchVel > 0 && pitch < -0.07) {
            telemetry.addLine("pitching FORWARD, pitched BACK");
            return motor.getPower() + 0.40 * pitch; //new code begins here
        } else if (Math.abs(pitch) < 0.10) {
            return motor.getPower();
        } else {
            return 0.70 * motor.getPower();
        }

        /*
        OLD CODE
        else if (Math.abs(pitchVel) < 0.005 && Math.abs(pitch) > 0.25) {
            return 0.70 * motor.getPower(); //decellerate
        } else {
            return motor.getPower();
        }*/

        //if power is max and still pitching down, set velocity to 0
    }
}
