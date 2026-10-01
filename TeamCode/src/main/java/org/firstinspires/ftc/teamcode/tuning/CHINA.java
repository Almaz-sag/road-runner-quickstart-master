package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name= "LIV AUTO")
public class CHINA extends LinearOpMode {

    private DcMotorEx launcher;
    private DcMotorEx launcher2;
    private DcMotor intake1;
    private DcMotor intake2;

    private Servo tur1, tur2, ramp2;
    private Servo stopper;

    private LauncherPID launcherPID;
    private LimeLightHelper limelight;
    volatile boolean limeEnabled = false;

    volatile boolean pos1 = false;
    volatile boolean pos2 = false;
    volatile boolean pos3 = false;
    volatile boolean pos4 = false;
    volatile boolean pos5 = false;

    public static double turPos1 = 0.707;
    public static double turPos2 = 0.835;
    public static double turPos3 = 1;
    public static double turPos4 = 0.67;
    public static double turPos5 = 0.73;

    private boolean shooting = false;

    public static double Lo = 0;

    private boolean Lo1 = false;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive =
                new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        launcher =
                hardwareMap.get(DcMotorEx.class, "launcher");

        launcher2 =
                hardwareMap.get(DcMotorEx.class, "launcher2");

        intake1 =
                hardwareMap.get(DcMotor.class, "intake1");

        intake2 =
                hardwareMap.get(DcMotor.class, "intake2");

        stopper =
                hardwareMap.servo.get("stopper");

        tur1 =
                hardwareMap.servo.get("tur1");

        tur2 =
                hardwareMap.servo.get("tur2");

        ramp2 =
                hardwareMap.servo.get("ramp2");

        limelight =
                new LimeLightHelper(hardwareMap);

        launcher2.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        intake2.setDirection(
                DcMotorSimple.Direction.REVERSE
        );
        stopper.setPosition(0.98);

        launcherPID =
                new LauncherPID(launcher, launcher2);

        tur1.setPosition(0.5);
        tur2.setPosition(0.5);
        ramp2.setPosition(0.7);

        waitForStart();

        if (Lo1) {
            Lo = 5;
        } else {
            Lo = 0;
        }

        Thread pidThread = new Thread(() -> {

            while (opModeIsActive()) {

                launcherPID.update();

                sleep(10);

            }
        });

        pidThread.start();

        Thread limeThread = new Thread(() -> {

            while (opModeIsActive()) {

                if (limeEnabled) {

                    turretTrackByLimelight();

                } else {

                    if (pos1) {

                        tur1.setPosition(turPos1);
                        tur2.setPosition(turPos1);

                    } else if (pos2) {

                        tur1.setPosition(turPos2);
                        tur2.setPosition(turPos2);

                    } else if (pos3) {

                        tur1.setPosition(turPos3);
                        tur2.setPosition(turPos3);

                    } else if(pos4){

                        tur1.setPosition(turPos4);
                        tur2.setPosition(turPos4);

                    } else if(pos5){

                        tur1.setPosition(turPos5);
                        tur2.setPosition(turPos5);

                    } else {

                        tur1.setPosition(0.5);
                        tur2.setPosition(0.5);
                    }
                }

                sleep(20);
            }
        });

        limeThread.start();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0, 0))
                        .stopAndAdd(new LauncherCmd(1525))
                        .waitSeconds(1.5)
                        .stopAndAdd(new Stopper(stopper, 0.8))
                        .waitSeconds(0.1)
                        .stopAndAdd(new Intake(intake1, intake2, 1))
                        .waitSeconds(1)
                        .stopAndAdd(new Stopper(stopper, 0.98))
                        .stopAndAdd(new Intake(intake1, intake2, 0))
                        .strafeToLinearHeading(new Vector2d(15,0), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(15, 36), Math.toRadians(180))
                        .stopAndAdd(new Intake(intake1, intake2, 1))
                        .strafeToLinearHeading(new Vector2d(0, 36), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(10, 36), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(0, 36), Math.toRadians(180))
                        .stopAndAdd(new Intake(intake1, intake2, 0))
                        .strafeToLinearHeading(new Vector2d(45, 30), Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(90, 39), Math.toRadians(-90))
                        .build()
        );
    }

    public class LauncherPID {

        DcMotorEx launcher, launcher2;

        double targetVelocity = 0;

        public double kP = 0.007;
        public double kI = 0.0012;
        public double kD = 0.000000001;

        double integral = 0;
        double lastError = 0;

        long lastTime = System.nanoTime();

        FtcDashboard dashboard =
                FtcDashboard.getInstance();

        public LauncherPID(
                DcMotorEx m,
                DcMotorEx m2
        ) {

            this.launcher = m;
            this.launcher2 = m2;

            launcher.setMode(
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER
            );

            launcher2.setMode(
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER
            );
        }

        public void setTarget(double vel) {
            this.targetVelocity = vel;
        }

        public void update() {

            long now = System.nanoTime();

            double dt =
                    (now - lastTime) / 1e9;

            lastTime = now;

            double currentVelocity =
                    -launcher.getVelocity();

            double error =
                    targetVelocity - currentVelocity;

            integral += error * dt;

            double derivative =
                    (dt > 0)
                            ? (error - lastError) / dt
                            : 0;

            lastError = error;

            double output =
                    kP * error +
                            kI * integral +
                            kD * derivative;

            output =
                    Range.clip(
                            output,
                            -1.0,
                            1.0
                    );

            launcher.setPower(output);
            launcher2.setPower(0);
        }
    }

    public class LauncherCmd implements Action {

        double vel;

        public LauncherCmd(double v) {
            this.vel = v;
        }

        @Override
        public boolean run(
                @NonNull TelemetryPacket telemetryPacket
        ) {

            launcherPID.setTarget(vel);

            return false;
        }
    }

    public void turretTrackByLimelight() {

        double tx =
                -limelight.getTx() +
                        3.5 -
                        Lo;

        double deadzone = 0.7;

        if (Math.abs(tx) < deadzone)
            return;

        double k = 0.0005;

        double currentPos =
                tur1.getPosition();

        double newPos =
                currentPos -
                        tx * k;

        newPos =
                Range.clip(
                        newPos,
                        0.0,
                        1.0
                );

        tur1.setPosition(newPos);
        tur2.setPosition(newPos);
    }

    public class LimeCmd implements Action {

        boolean enable;

        public LimeCmd(boolean enable) {
            this.enable = enable;
        }

        @Override
        public boolean run(
                @NonNull TelemetryPacket telemetryPacket
        ) {

            limeEnabled = enable;

            return false;
        }
    }

    public class Lo1 implements Action {

        boolean enable;

        public Lo1(boolean enable) {
            this.enable = enable;
        }

        @Override
        public boolean run(
                @NonNull TelemetryPacket telemetryPacket
        ) {

            Lo1 = enable;

            return false;
        }
    }

    public class TPos1 implements Action {

        boolean enable;

        public TPos1(boolean enable) {
            this.enable = enable;
        }

        @Override
        public boolean run(
                @NonNull TelemetryPacket telemetryPacket
        ) {

            pos1 = enable;

            return false;
        }
    }

    public class TPos2 implements Action {

        boolean enable;

        public TPos2(boolean enable) {
            this.enable = enable;
        }

        @Override
        public boolean run(
                @NonNull TelemetryPacket telemetryPacket
        ) {

            pos2 = enable;

            return false;
        }
    }

    public class TPos3 implements Action {

        boolean enable;

        public TPos3(boolean enable) {
            this.enable = enable;
        }

        @Override
        public boolean run(
                @NonNull TelemetryPacket telemetryPacket
        ) {

            pos3 = enable;

            return false;
        }
    }

    public class TPos4 implements Action {

        boolean enable;

        public TPos4(boolean enable) {
            this.enable = enable;
        }

        @Override
        public boolean run(
                @NonNull TelemetryPacket telemetryPacket
        ) {

            pos4 = enable;

            return false;
        }
    }

    public class TPos5 implements Action {

        boolean enable;

        public TPos5(boolean enable) {
            this.enable = enable;
        }

        @Override
        public boolean run(
                @NonNull TelemetryPacket telemetryPacket
        ) {

            pos5 = enable;

            return false;
        }
    }

    public class Stopper implements Action {

        Servo servo;
        double pos;

        public Stopper(
                Servo s,
                double p
        ) {

            this.servo = s;
            this.pos = p;
        }

        @Override
        public boolean run(
                @NonNull TelemetryPacket telemetryPacket
        ) {

            servo.setPosition(pos);

            return false;
        }
    }

    public class Intake implements Action {

        DcMotor motor, motor2;
        double pow;

        public Intake(
                DcMotor m,
                DcMotor m2,
                double p
        ) {

            this.motor = m;
            this.motor2 = m2;
            this.pow = p;
        }

        @Override
        public boolean run(
                @NonNull TelemetryPacket telemetryPacket
        ) {

            motor.setPower(-pow);
            motor2.setPower(-pow);

            return false;
        }
    }
}