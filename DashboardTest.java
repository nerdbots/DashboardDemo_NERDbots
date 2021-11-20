/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

// IMPORTANT FOR DASHBOARD!! Need these two imports!
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;




// The @Config is IMPORTANT FOR DASHBOARD! It lets dashboard recognize the program.
@Config
@TeleOp(name="DashboardTest", group="Linear Opmode")
//                              \/ This calculates our loop time, which we use for PID.
public class DashboardTest<loopTime> extends LinearOpMode {
    //Sets up FTC Dashboard. 
    FtcDashboard dashboard;


    // initialize (set up) motor variables
    private BNO055IMU imu;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    //Gyro variables
    Orientation lastAngles = new Orientation();    
    double robotAngleToField = 0;
    double robotAngle = 0;
    public static double robotTargetAngle = 0;

    //PID gains
    public static double Kp = 0.0075;
    public static double Ki = 0.0006;
    public static double Kd = 0.001;


    // PID variables. 
    public static double propError = 0;
    public static double intError = 0;
    public static double derError = 0;
    public static double prevDerError = 0;
    public static double angletolerance = 0.5;
//   /\  IMPORTANT FOR DASHBOARD!! All variables that are to be edited in dashboard MUST be "public" and "static."


    double currentTime = 0;
    double deltaTime = 0;
    double startTime = 0;
    double oldTime = 0;
    double loopTime = 0;
    double motorPower = 0;
    double motorPowerOutput = 0;
    private ElapsedTime elapsedTime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // hardwaremaps
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "Rear_Right_Motor");


        //gyro initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        resetAngle();


        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        // Reset motor powers
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //init period
        waitForStart();


        startTime = elapsedTime.seconds();



        //IMPORTANT DASHBOARD VARIABLES!! Put these in otherwise it won't work
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        while (opModeIsActive() && !isStopRequested()) {
            currentTime = elapsedTime.seconds();
            loopTime = currentTime - oldTime;
            oldTime = currentTime;
            deltaTime = currentTime - startTime;

            motorPowerOutput = turnPID();

            rearRightMotor.setPower(motorPowerOutput);
            frontLeftMotor.setPower(motorPowerOutput);
            frontRightMotor.setPower(motorPowerOutput);
            rearLeftMotor.setPower(motorPowerOutput);
            

            // telemetry is neccessary for dashboard; to show it on a graph, it must be printed in telemetry.
            telemetry.addData("robot Angle", getAngle());
            telemetry.addData("kp", Kp);
            telemetry.addData("ki", Ki);
            telemetry.addData("kd", Kd);
            telemetry.update();
        }
    }

    // Reset gyro angle
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robotAngleToField = 0;
    }


    //Function to get the angle of the Gyro sensor
    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        robotAngleToField += deltaAngle;

        lastAngles = angles;

        return robotAngleToField;
    }


    // PID function
    private double turnPID() {

        // P
        propError = (robotTargetAngle - getAngle());

        // I
        intError += (robotTargetAngle - getAngle()) * loopTime;

        // D
        derError = ((robotTargetAngle - getAngle()) - prevDerError) / loopTime;


        // More D
        prevDerError = robotTargetAngle - getAngle();


        // If we're close to the target, stop
        if (Math.abs(robotTargetAngle - getAngle()) < angletolerance) {

            motorPower = propError * Kp + intError * Ki;
            intError = 0;
        }
        else    {
            motorPower = propError * Kp + intError * Ki + derError * Kd;
        }


        return motorPower;

    }



}