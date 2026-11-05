package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@TeleOp(name = "R LOCO", group = "Drive")
public class REDLOCO extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake1, intake2;

    // ===== LAUNCHER =====
    private DcMotorEx launcher, launcher2;
    private Servo hood, stopper, ramp;

    // ===== TURRET =====
    private Servo tur1, tur2;
    private GoBildaPinpointDriver pinpoint;
    LimeLightHelper limelight;

    // ===== LAUNCHER PID =====
    public static double kP = 0.007;
    public static double kI = 0.0012;
    public static double kD = 0.000000001;

    private double integral = 0, lastError = 0;
    private long lastTime = 0;
    private double targetVelocity;

    public static double OPEN = 0;
    public static double CLOSE = 0;

    // ===== FIELD =====
    public static double StartX = 54.5;
    public static double StartY = -7.8;

    double parY = 3.717;
    double perpX = -5.82;

    public static double turretOffsetX = 0.18; // inches вперед
    public static double turretOffsetY = 0; // если по центру

    public static double turmin = 0.0;
    public static double turmax = 1.0;

    public static double lgoalX = 65;
    public static double lgoalY = 65;

    public static double rgoalX = 65;
    public static double rgoalY = 65;

    public static double goalX;
    public static double goalY;
    public static double hoodBasePos;
    public static double d1 = 40;
    public static double d2 = 50;
    public static double d3 = 60;
    public static double d4 = 70;
    public static double d5 = 80;


    public static double v1 = 1440;
    public static double v2 = 1540;
    public static double v3 = 1710;
    public static double v4 = 1980;
    public static double v5 = 2110;


    public static double Hpos1 = 0;
    public static double Hpos2 = 0.12;
    public static double Hpos3 = 0.3;
    public static double Hpos4 = 0.425;
    public static double Hpos5 = 0.51;
    public static double Kl = -0.15;
    public static double Kr = 0.08;
    public static int LEFT_TAG = 32;
    public static int RIGHT_TAG = 35;
    public int AIM;
    private boolean track = false;

    private double normalize(double a) {
        while (a > Math.PI)
            a -= 2 * Math.PI;

        while (a < -Math.PI)
            a += 2 * Math.PI;

        return a;
    }

    private double angleToServo(double angle) {
        angle = normalize(angle);

        double range = Math.toRadians(270); // servo range
        double servo = 0.5 + (angle / range);

        return Range.clip(servo, 0.0, 1.0);
    }

    public double getVelocityFromTable(double distance) {

        double[] distances = {
                d1, d2, d3, d4, d5
        };

        double[] velocities = {
                v1, v2, v3, v4, v5
        };

        // ниже минимума
        if (distance <= distances[0])
            return velocities[0];

        // выше максимума
        if (distance >= distances[distances.length - 1])
            return velocities[velocities.length - 1];

        // ищем промежуток
        for (int i = 0; i < distances.length - 1; i++) {

            if (distance >= distances[i] && distance <= distances[i + 1]) {

                double d1 = distances[i];
                double d2 = distances[i + 1];

                double v1 = velocities[i];
                double v2 = velocities[i + 1];

                // линейная интерполяция
                double t = (distance - d1) / (d2 - d1);

                return v1 + t * (v2 - v1);
            }
        }

        return velocities[0]; // fallback
    }

    public double getHoodPosition(double distance) {

        if (distance < d1) {
            return Hpos1;
        } else if (distance < d2) {
            return Hpos2;
        } else if (distance < d3) {
            return Hpos3;
        } else if (distance < d4) {
            return Hpos4;
        } else if (distance < d5) {
            return Hpos5;
        } else {
            return 1;
        }
    }

    public void updateTurret() {

        Pose2D pos = pinpoint.getPosition();

        double robotX = pos.getX(DistanceUnit.INCH);
        double robotY = pos.getY(DistanceUnit.INCH);
        double robotHeading = pos.getHeading(AngleUnit.RADIANS);

        // ===== APPLY TURRET OFFSET =====
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        double turretX =
                robotX + turretOffsetX * cos - turretOffsetY * sin;

        double turretY =
                robotY + turretOffsetX * sin + turretOffsetY * cos;

        // ===== TARGET VECTOR =====
        double dx = goalX - turretX;
        double dy = goalY - turretY;

        double LeftError = Math.abs(robotHeading) * Kl;
        double RightError = Math.abs(robotHeading) * Kr;

        double targetAngle;

        if (Math.toDegrees(robotHeading) > 27.5) {
            targetAngle = Math.atan2(dy, dx) + LeftError;
        } else if (Math.toDegrees(robotHeading) < -5) {
            targetAngle = Math.atan2(dy, dx) + RightError;
        } else {
            targetAngle = Math.atan2(dy, dx);
        }

        // ===== RELATIVE ANGLE =====
        double relativeAngle = normalize(targetAngle - robotHeading);

        // ===== SERVO MAPPING =====
        double servoPos = angleToServo(relativeAngle);

        // ===== SAFETY CLAMP =====
        servoPos = Range.clip(servoPos, turmin, turmax);

        tur1.setPosition(servoPos);
        tur2.setPosition(servoPos);
    }

    public double getTurretError() {

        Pose2D pos = pinpoint.getPosition();

        double robotX = pos.getX(DistanceUnit.INCH);
        double robotY = pos.getY(DistanceUnit.INCH);
        double robotHeading = pos.getHeading(AngleUnit.RADIANS);

        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        double turretX =
                robotX + turretOffsetX * cos - turretOffsetY * sin;

        double turretY =
                robotY + turretOffsetX * sin + turretOffsetY * cos;

        double dx = goalX - turretX;
        double dy = goalY - turretY;

        double targetAngle = Math.atan2(dy, dx);

        double relativeAngle =
                normalize(targetAngle - robotHeading);

        return relativeAngle; // В РАДИАНАХ!
    }

    public void moveRobot() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x * 0.85;

        double heading =
                pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double globalForward =
                forward * cos - strafe * sin;

        double globalStrafe =
                forward * sin + strafe * cos;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] =
                globalForward + globalStrafe + rotate;

        newWheelSpeeds[1] =
                globalForward - globalStrafe - rotate;

        newWheelSpeeds[2] =
                globalForward - globalStrafe + rotate;

        newWheelSpeeds[3] =
                globalForward + globalStrafe - rotate;

        frontLeft.setPower(newWheelSpeeds[0]);
        frontRight.setPower(newWheelSpeeds[1]);
        backLeft.setPower(newWheelSpeeds[2]);
        backRight.setPower(newWheelSpeeds[3]);
    }

    public void closeTrack(int tagId) {

        if (!limelight.hasTag(tagId)) {
            return;
        }

        double tx = limelight.getTxForTag(tagId);

        double deadzone = 0.5;

        if (Math.abs(tx) < deadzone)
            return;

        double k = 0.0006;

        double currentPos = tur1.getPosition();

        double newPos = currentPos - tx * k;

        newPos = Range.clip(newPos, turmin, turmax);

        tur1.setPosition(newPos);
        tur2.setPosition(newPos);
    }



    @Override
    public void runOpMode() {

        limelight =
                new LimeLightHelper(hardwareMap);

        tur1 = hardwareMap.get(Servo.class, "tur1");
        tur2 = hardwareMap.get(Servo.class, "tur2");
        ramp = hardwareMap.get(Servo.class, "ramp");

        pinpoint =
                hardwareMap.get(
                        GoBildaPinpointDriver.class,
                        "pinpoint"
                );

        frontLeft =
                hardwareMap.get(DcMotor.class, "leftFront");

        frontRight =
                hardwareMap.get(DcMotor.class, "rightFront");

        backLeft =
                hardwareMap.get(DcMotor.class, "leftBack");

        backRight =
                hardwareMap.get(DcMotor.class, "rightBack");

        intake1 =
                hardwareMap.get(DcMotor.class, "intake1");

        intake2 =
                hardwareMap.get(DcMotor.class, "intake2");

        launcher =
                hardwareMap.get(DcMotorEx.class, "launcher");

        launcher2 =
                hardwareMap.get(DcMotorEx.class, "launcher2");

        stopper =
                hardwareMap.get(Servo.class, "stopper");

        hood =
                hardwareMap.get(Servo.class, "hood");

        frontLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake1.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake2.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontLeft.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        backLeft.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        launcher2.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        pinpoint.setOffsets(
                parY,
                perpX,
                DistanceUnit.INCH
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        Pose2D startPose =
                new Pose2D(
                        DistanceUnit.INCH,
                        StartX,
                        StartY,
                        AngleUnit.DEGREES,
                        0
                );

        pinpoint.setPosition(startPose);

        waitForStart();

        while (opModeIsActive()) {

            pinpoint.update();

            // ===== DISTANCE =====
            Pose2D pos = pinpoint.getPosition();

            double RHead =
                    pos.getHeading(AngleUnit.RADIANS);

            double robotX =
                    pos.getX(DistanceUnit.INCH);

            double robotY =
                    pos.getY(DistanceUnit.INCH);

            double cos =
                    Math.cos(pos.getHeading(AngleUnit.RADIANS));

            double sin =
                    Math.sin(pos.getHeading(AngleUnit.RADIANS));

            double turretX =
                    robotX + turretOffsetX * cos - turretOffsetY * sin;

            double turretY =
                    robotY + turretOffsetX * sin + turretOffsetY * cos;

            double dx = goalX - turretX;
            double dy = goalY - turretY;

            double distance =
                    Math.sqrt(dx * dx + dy * dy);

            // ===== HOOD =====
            hoodBasePos =
                    getHoodPosition(distance);

            hood.setPosition(hoodBasePos);

            moveRobot();


            if (gamepad1.right_bumper) {
                ramp.setPosition(0.075);
            } else if (gamepad2.right_bumper) {
                ramp.setPosition(0.5);
            } else {
                ramp.setPosition(0.7);
            }

            if (gamepad2.right_stick_button) {
                track = true;
            } else if (gamepad2.left_stick_button) {
                track = false;
            }


            if (gamepad2.dpad_up) {
                Pose2D reset1 =
                        new Pose2D(
                                DistanceUnit.INCH,
                                37.3,
                                61.8,
                                AngleUnit.DEGREES,
                                0
                        );

                pinpoint.setPosition(reset1);
            }

            if (gamepad2.dpad_left) {

                Pose2D reset2 =
                        new Pose2D(
                                DistanceUnit.INCH,
                                54.5,
                                7,
                                AngleUnit.DEGREES,
                                0
                        );

                pinpoint.setPosition(reset2);
            }

            if (gamepad2.dpad_down) {

                Pose2D reset3 =
                        new Pose2D(
                                DistanceUnit.INCH,
                                -60.1,
                                -61.3,
                                AngleUnit.DEGREES,
                                180
                        );

                pinpoint.setPosition(reset3);
            }

            targetVelocity =
                    getVelocityFromTable(distance);

            if (robotY < 0) {
                AIM = LEFT_TAG;
                goalX = lgoalX;
                goalY = lgoalY;
            } else if (robotY > 0) {
                AIM = RIGHT_TAG;
                goalX = rgoalX;
                goalY = rgoalY;
            }

            if (gamepad2.right_bumper) {
                double turretError =
                        Math.abs(getTurretError());

                double threshold =
                        Math.toRadians(10);

                if (
                        limelight.hasTarget()
                                && turretError < threshold
                                && distance > d3
                ) {
                    closeTrack(AIM);

                } else {
                    if(track){
                        updateTurret();
                    } else {
                        tur1.setPosition(0.5);
                        tur2.setPosition(0.5);
                    }
                }

            } else {
                if(track){
                    updateTurret();
                } else {
                    tur1.setPosition(0.5);
                    tur2.setPosition(0.5);
                }
            }

            double currentVelocity =
                    launcher.getVelocity();

            double error =
                    targetVelocity - currentVelocity;

            long now =
                    System.nanoTime();

            double dt =
                    (lastTime == 0)
                            ? 0
                            : (now - lastTime) / 1e9;

            lastTime = now;

            integral += error * dt;

            integral =
                    Range.clip(
                            integral,
                            -500,
                            500
                    );

            double derivative =
                    (dt > 0)
                            ? (error - lastError) / dt
                            : 0;

            lastError = error;

            double output =
                    kP * error
                            + kI * integral
                            + kD * derivative;

            output =
                    Range.clip(
                            output,
                            -1.0,
                            1.0
                    );

            launcher.setPower(output);
            launcher2.setPower(output);

            // ===== INTAKE =====
            if (
                    Math.abs(
                            gamepad2.left_trigger
                                    - gamepad2.right_trigger
                    ) > 0.1
            ) {

                double p =
                        gamepad2.left_trigger
                                - gamepad2.right_trigger;

                intake1.setPower(p);
                intake2.setPower(-p);

                stopper.setPosition(OPEN);

            } else {

                double p =
                        gamepad1.left_trigger
                                - gamepad1.right_trigger;

                intake1.setPower(p);
                intake2.setPower(-p);

                stopper.setPosition(CLOSE);
            }

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData(
                    "H",
                    Math.toDegrees(RHead)
            );
            telemetry.addData(
                    "Distance",
                    distance
            );
            telemetry.addData(
                    "Output",
                    output
            );
            telemetry.addData(
                    "error",
                    error
            );
            telemetry.addData(
                    "curVel",
                    currentVelocity
            );
            telemetry.addData(
                    "tarVel",
                    targetVelocity
            );
            telemetry.addData(
                    "turer",
                    Math.toDegrees(getTurretError())
            );

            telemetry.update();
        }
    }
}
