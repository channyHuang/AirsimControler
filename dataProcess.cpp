#include "dataProcess.h"

#include "messageStruct.h"
#include "osgManager.h"

#define SKEW_SYM_MATRIX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang)
{
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRIX(r_axis);
        /// Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

template<typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K << SKEW_SYM_MATRIX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001)
    {
        T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRIX(r_ang);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

/* Logrithm of a Rotation Matrix */
template<typename T>
Eigen::Matrix<T,3,1> SO3_LOG(const Eigen::Matrix<T, 3, 3> &R)
{
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T,3,1> K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}


#define DIM_OF_STATES (18) // For faster 

#define COV_OMEGA_NOISE_DIAG 1e-1
#define COV_ACC_NOISE_DIAG 0.4
#define COV_GYRO_NOISE_DIAG 0.2

#define COV_BIAS_ACC_NOISE_DIAG 0.05
#define COV_BIAS_GYRO_NOISE_DIAG 0.1

#define COV_START_ACC_DIAG 1e-1
#define COV_START_GYRO_DIAG 1e-1

static const Eigen::Matrix3d Eye3d(Eigen::Matrix3d::Identity());
static const Eigen::Matrix3f Eye3f(Eigen::Matrix3f::Identity());
static const Eigen::Vector3d Zero3d(0, 0, 0);
static const Eigen::Vector3f Zero3f(0, 0, 0);

double last_timestamp_imu = -1;
Eigen::Vector3d acc_imu( 0, 0, 0 ), angvel_avr( 0, 0, 0 ), acc_avr( 0, 0, 0 ), vel_imu( 0, 0, 0 ), pos_imu( 0, 0, 0 );
Eigen::Matrix3d R_imu;
Eigen::MatrixXd F_x( Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES >::Identity() );
Eigen::MatrixXd cov_w( Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES >::Zero() );
double          dt = 0;
int             if_first_imu = 1;
Eigen::Vector3d gravity = Eigen::Vector3d( 0, 0, 9.805 );

void DataProcess::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg) {
    double timestamp = msg->header.stamp.toSec();
    if (timestamp < last_timestamp_imu) return;

    angvel_avr << msg->angular_velocity.x(), msg->angular_velocity.y(), msg->angular_velocity.z();
    acc_avr << msg->linear_acceleration.x(), msg->linear_acceleration.y(), msg->linear_acceleration.z();
    
    // calc current vel/rot/pos
    if (!if_first_imu) {
        if_first_imu = 0;
        return;
    }
    dt = timestamp - last_timestamp_imu;
    last_timestamp_imu = timestamp;
    if (dt > 0.05) {
        dt = 0.05;
    }

    /* covariance propagation */
    Eigen::Matrix3d acc_avr_skew;
    Eigen::Matrix3d Exp_f = Exp( angvel_avr, dt );
    acc_avr_skew << SKEW_SYM_MATRIX( acc_avr );
    // Eigen::Matrix3d Jr_omega_dt = right_jacobian_of_rotion_matrix<double>(angvel_avr*dt);
    Eigen::Matrix3d Jr_omega_dt = Eigen::Matrix3d::Identity();
    F_x.block< 3, 3 >( 0, 0 ) = Exp_f.transpose();
    // F_x.block<3, 3>(0, 9) = -Eye3d * dt;
    F_x.block< 3, 3 >( 0, 9 ) = -Jr_omega_dt * dt;
    // F_x.block<3,3>(3,0)  = -R_imu * off_vel_skew * dt;
    F_x.block< 3, 3 >( 3, 3 ) = Eye3d; // Already the identity.
    F_x.block< 3, 3 >( 3, 6 ) = Eye3d * dt;
    F_x.block< 3, 3 >( 6, 0 ) = -R_imu * acc_avr_skew * dt;
    F_x.block< 3, 3 >( 6, 12 ) = -R_imu * dt;
    F_x.block< 3, 3 >( 6, 15 ) = Eye3d * dt;

    Eigen::Matrix3d cov_acc_diag, cov_gyr_diag, cov_omega_diag;
    cov_omega_diag = Eigen::Vector3d( COV_OMEGA_NOISE_DIAG, COV_OMEGA_NOISE_DIAG, COV_OMEGA_NOISE_DIAG ).asDiagonal();
    cov_acc_diag = Eigen::Vector3d( COV_ACC_NOISE_DIAG, COV_ACC_NOISE_DIAG, COV_ACC_NOISE_DIAG ).asDiagonal();
    cov_gyr_diag = Eigen::Vector3d( COV_GYRO_NOISE_DIAG, COV_GYRO_NOISE_DIAG, COV_GYRO_NOISE_DIAG ).asDiagonal();
    // cov_w.block<3, 3>(0, 0) = cov_omega_diag * dt * dt;
    cov_w.block< 3, 3 >( 0, 0 ) = Jr_omega_dt * cov_omega_diag * Jr_omega_dt * dt * dt;
    cov_w.block< 3, 3 >( 3, 3 ) = R_imu * cov_gyr_diag * R_imu.transpose() * dt * dt;
    cov_w.block< 3, 3 >( 6, 6 ) = cov_acc_diag * dt * dt;
    cov_w.block< 3, 3 >( 9, 9 ).diagonal() =
        Eigen::Vector3d( COV_BIAS_GYRO_NOISE_DIAG, COV_BIAS_GYRO_NOISE_DIAG, COV_BIAS_GYRO_NOISE_DIAG ) * dt * dt; // bias gyro covariance
    cov_w.block< 3, 3 >( 12, 12 ).diagonal() =
        Eigen::Vector3d( COV_BIAS_ACC_NOISE_DIAG, COV_BIAS_ACC_NOISE_DIAG, COV_BIAS_ACC_NOISE_DIAG ) * dt * dt; // bias acc covariance

    R_imu = R_imu * Exp_f;
    acc_imu = R_imu * acc_avr ;//- gravity;
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
    vel_imu = vel_imu + acc_imu * dt;
    //angvel_last = angvel_avr;
    //acc_s_last = acc_imu;

    // final result
    Eigen::Vector3d vel_end = vel_imu + acc_imu * dt;
    Eigen::Matrix3d rot_end = R_imu * Exp( angvel_avr, dt );
    Eigen::Vector3d pos_end = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

    Eigen::Quaterniond quat(rot_end);
    OsgManager::getInstance()->updatePosition(quat.x(), quat.y(), quat.z(), quat.w(), pos_end.x(), pos_end.y(), pos_end.z());
}