package org.chillout1778.commands;

import static edu.wpi.first.units.Units.*;

import org.chillout1778.Constants;
import org.chillout1778.subsystems.SwerveNext;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * 遥控驱动命令类
 * 实现了两种控制模式的自动切换：
 * 1. 场地中心控制（FieldCentric）：当有旋转输入时使用，可以自由控制机器人的旋转
 * 2. 场地中心朝向控制（FieldCentricFacingAngle）：当无旋转输入时使用，自动保持机器人的朝向
 */
public class TeleopDriveNextCommand extends Command {
    private final SwerveNext m_swerveDrivetrain;
    private final CommandXboxController m_controller;
    private final SlewRateLimiter xfilter = new SlewRateLimiter(3);
    private final SlewRateLimiter yfilter = new SlewRateLimiter(3);
    private final SlewRateLimiter zfilter = new SlewRateLimiter(2);

    // 最大速度和角速度
    private final double MaxSpeed = Constants.SwerveNext.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // 场地中心控制请求：用于自由控制机器人的移动和旋转
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1) // 设置移动死区为最大速度的10%
            .withRotationalDeadband(MaxAngularRate * 0.1) // 设置旋转死区为最大角速度的10%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // 使用开环电压控制驱动电机
            .withSteerRequestType(SteerRequestType.MotionMagicExpo); // 使用MotionMagic控制转向电机

    // 场地中心朝向控制请求：用于保持机器人朝向
    private final SwerveRequest.FieldCentricFacingAngle facingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1) // 设置移动死区为最大速度的10%
            .withRotationalDeadband(MaxAngularRate * 0.1) // 设置旋转死区为最大角速度的10%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // 使用开环电压控制驱动电机
            .withSteerRequestType(SteerRequestType.MotionMagicExpo); // 使用MotionMagic控制转向电机

    // 用于角度保持的变量
    private Rotation2d lastAngle; // 记录最后的目标角度
    private boolean isRotating = true;

    /**
     * 构造函数
     * 
     * @param swerveDrivetrain 底盘子系统
     * @param controller       手柄控制器
     */
    public TeleopDriveNextCommand(SwerveNext swerveDrivetrain, CommandXboxController controller) {
        m_swerveDrivetrain = swerveDrivetrain;
        m_controller = controller;
        facingAngle.HeadingController.setPID(
                Constants.SwerveNext.HeadingController.getP(),
                Constants.SwerveNext.HeadingController.getI(),
                Constants.SwerveNext.HeadingController.getD());
        addRequirements(m_swerveDrivetrain);
    }

    @Override
    public void execute() {
        // 获取手柄输入并应用死区
        double xSpeed = -xfilter.calculate(m_controller.getLeftY());
        // -MathUtil.applyDeadband(m_controller.getLeftY(), 0.1);
        double ySpeed = -yfilter.calculate(m_controller.getLeftX());
        // -MathUtil.applyDeadband(m_controller.getLeftX(), 0.1);
        double rotationSpeed = -zfilter.calculate(m_controller.getRightX());
        // -MathUtil.applyDeadband(m_controller.getRightX(), 0.1);

        // 处理旋转控制
        boolean rotating = Math.abs(rotationSpeed) > 0.01;

        if (rotating) {
            isRotating = true;
        } // else {
          // // 如果刚停止旋转，更新目标角度为当前角度
          // lastAngle = m_swerveDrivetrain.getState().Pose.getRotation();
          // isRotating = false;
          // // if (correctionTimer.hasElapsed(CORRECTION_TIME_THRESHOLD)) {
          // // isRotating = false;
          // // }
          // }

        // 根据控制模式设置底盘运动
        m_swerveDrivetrain.setControl(
                isRotating ?
                // 旋转模式：使用场地中心控制
                        fieldCentric.withVelocityX(xSpeed * MaxSpeed)
                                .withVelocityY(ySpeed * MaxSpeed)
                                .withRotationalRate(rotationSpeed * MaxAngularRate)
                        :
                        // 保持朝向模式：使用场地中心朝向控制
                        facingAngle.withVelocityX(xSpeed * MaxSpeed)
                                .withVelocityY(ySpeed * MaxSpeed)
                                .withTargetDirection(lastAngle));
    }
}
