package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Test")
public class Test extends LinearOpMode {
    DcMotorEx motor;
    IMU gyro;

    @Override
    public void runOpMode() {
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
            telemetry.addData("Power", motor.getPower());
            telemetry.addData("Pitch", gyro.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
            telemetry.addData("Pitch Velocity", gyro.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate);
            telemetry.update();

            sleep(10);
        }
    }
}
