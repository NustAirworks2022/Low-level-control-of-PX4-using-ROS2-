#include "../include/px4_offboard_lowlevel/controller.h"


controller::controller(){

}

void controller::calculateControllerOutput(
        Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion) {
    assert(controller_torque_thrust);

    controller_torque_thrust->resize(4);
   // Initialize variables with current states
   // Position (x, y, z)
   double x = position_W_(0);
   double y = position_W_(1);
   double z = position_W_(2);

   // Orientation (roll, pitch, yaw)
   Eigen::Vector3d euler_angles = R_B_W_.eulerAngles(0, 1, 2);
   double ph = euler_angles(0);  // Roll
   double th = euler_angles(1);  // Pitch
   double ps = euler_angles(2);  // Yaw

   // Linear velocities (vx, vy, vz)
   double vx = velocity_W_(0);
   double vy = velocity_W_(1);
   double vz = velocity_W_(2);

   // Angular velocities (p, q, r)
   double p = angular_velocity_B_(0);
   double q = angular_velocity_B_(1);
   double r = angular_velocity_B_(2);


    // Control variables
    double x1 = 0.0, x2 = 0.0, x3 = 0.0, x4 = 0.0;
    double x5 = 0.0, x6 = 0.0, x7 = 0.0, x8 = 0.0;
    double x9 = 0.0, x10 = 0.0, x11 = 0.0, x12 = 0.0;
    double u1 = 0.0, u2 = 0.0, u3 = 0.0, u4 = 0.0; // Control inputs

    // Physical constants
    double g = 9.81;    // Gravity
    double m = 0.0;     // Mass
    double Ix = 0.0, Iy = 0.0, Iz = 0.0;  // Moments of inertia
    double kr = 0.0, kt = 0.0;  // Control gains


    // Distance variables
    double dist_x = 0.0, dist_y = 0.0, dist_z = 0.0;

    // Control variables for the new system
    double zeta1 = 0.0, zeta2 = 0.0, u_new = 0.0;

    // Desired states
    double des_height = 0.0, des_yaw = 0.0, des_radius = 0.0;
    // Control gains
    double k1 = 200.0;
    double k2 = 400.0;
    double k3 = 90.0;
    double k4 = 30.0;
    double k5 = 100.0;
    double k6 = 300.0;
    double k7 = 60.0;
    double k8 = 30.0;
    double k9 = 10.0;
    double k10 = 40.0;
    double k11 = 30.0;
    double k12 = 30.0;
    double k13 = 10.0;
    double k14 = 12.0;

    // Physical constants
    double L = 0.2656;
    double Ix = 0.01152;
    double Iy = 0.01152;
    double Iz = 0.0218;
    double g = 9.8;
    double m = 1.923;

    // Desired states
    double des_radius = 1.0;
    double des_height = 1.0;
    double des_yaw = 0.0;
    double d = 1.0;

    // Correct mapping from system identification point of view
    std::array<std::array<double, 4>, 4> M = { {
        {0.0, 0.0, -L,  L},
        { L, -L,  0.0, 0.0},
        {-d, -d,  d,   d},
        { 1.0, 1.0, 1.0, 1.0}
    } };
    // Distance variables
    double dist_x = 0.0;
    double dist_y = 0.0;
    double dist_z = 0.0;
    // Control gains
    double kt = 0.01;
    double kr = 0.01;

    double xi1_1 = pow(x, 2) - pow(des_radius, 2) + pow(y, 2);
    double xi1_2 = 2 * vx * x + 2 * vy * y;
    double xi1_3 = (2 * (dist_x * x + dist_y * y + m * pow(vx, 2) + m * pow(vy, 2) - kt * vx * x - kt * vy * y
        - y * zeta1 * cos(ps) * sin(ph) + x * zeta1 * sin(ph) * sin(ps) + x * zeta1 * cos(ph) * cos(ps) * sin(th)
        + y * zeta1 * cos(ph) * sin(ps) * sin(th))) / m;
    double xi1_4 = (2 * zeta2 * (x * sin(ph) * sin(ps) - y * cos(ps) * sin(ph) + x * cos(ph) * cos(ps) * sin(th)
        + y * cos(ph) * sin(ps) * sin(th))) / m
        - (2 * (kt * x - 2 * m * vx) * (dist_x - kt * vx + zeta1 * sin(ph) * sin(ps) + zeta1 * cos(ph) * cos(ps) * sin(th))) / pow(m, 2)
        + (2 * (2 * m * vy - kt * y) * (dist_y - kt * vy - zeta1 * cos(ps) * sin(ph) + zeta1 * cos(ph) * sin(ps) * sin(th))) / pow(m, 2)
        + (2 * vx * (dist_x - kt * vx + zeta1 * sin(ph) * sin(ps) + zeta1 * cos(ph) * cos(ps) * sin(th))) / m
        + (2 * vy * (dist_y - kt * vy - zeta1 * cos(ps) * sin(ph) + zeta1 * cos(ph) * sin(ps) * sin(th))) / m
        + (2 * zeta1 * (r * cos(ph) + q * sin(ph)) * (x * cos(ps) * sin(ph) + y * sin(ph) * sin(ps)
            + y * cos(ph) * cos(ps) * sin(th) - x * cos(ph) * sin(ps) * sin(th))) / (m * cos(th))
        - (2 * zeta1 * (p * cos(th) + r * cos(ph) * sin(th) + q * sin(ph) * sin(th)) * (y * cos(ph) * cos(ps)
            - x * cos(ph) * sin(ps) + x * cos(ps) * sin(ph) * sin(th) + y * sin(ph) * sin(ps) * sin(th))) / (m * cos(th))
        + (2 * zeta1 * cos(ph) * cos(th) * (q * cos(ph) - r * sin(ph)) * (x * cos(ps) + y * sin(ps))) / m;

    double Lg1Lf3S1 = (2 * (x * sin(ph) * sin(ps) - y * cos(ps) * sin(ph) + x * cos(ph) * cos(ps) * sin(th)
        + y * cos(ph) * sin(ps) * sin(th))) / m;

    double Lg2Lf3S1 = -(2 * zeta1 * (y * cos(ph) * cos(ps) - x * cos(ph) * sin(ps) + x * cos(ps) * sin(ph) * sin(th)
        + y * sin(ph) * sin(ps) * sin(th))) / (Ix * m);

    double Lg3Lf3S1 = ((2 * zeta1 * sin(ph) * (x * cos(ps) * sin(ph) + y * sin(ph) * sin(ps)
        + y * cos(ph) * cos(ps) * sin(th) - x * cos(ph) * sin(ps) * sin(th))) / (m * cos(th))
        + (2 * zeta1 * pow(cos(ph), 2) * cos(th) * (x * cos(ps) + y * sin(ps))) / m
        - (2 * zeta1 * sin(ph) * sin(th) * (y * cos(ph) * cos(ps) - x * cos(ph) * sin(ps)
            + x * cos(ps) * sin(ph) * sin(th) + y * sin(ph) * sin(ps) * sin(th))) / (m * cos(th))) / Iy;

    double Lg4Lf3S1 = -((2 * zeta1 * cos(ph) * sin(th) * (y * cos(ph) * cos(ps) - x * cos(ph) * sin(ps)
        + x * cos(ps) * sin(ph) * sin(th) + y * sin(ph) * sin(ps) * sin(th))) / (m * cos(th))
        - (2 * zeta1 * cos(ph) * (x * cos(ps) * sin(ph) + y * sin(ph) * sin(ps)
            + y * cos(ph) * cos(ps) * sin(th) - x * cos(ph) * sin(ps) * sin(th))) / (m * cos(th))
        + (2 * zeta1 * cos(ph) * cos(th) * sin(ph) * (x * cos(ps) + y * sin(ps))) / m) / Iz;

    double Lf4S1 = (q * cos(ph) - r * sin(ph)) * ((2 * zeta2 * (x * cos(ph) * cos(ps) * cos(th)
        + y * cos(ph) * cos(th) * sin(ps))) / m
        - (2 * zeta1 * (x * cos(ps) * cos(th) * sin(ph) + y * cos(th) * sin(ph) * sin(ps))
            * (p * cos(th) + r * cos(ph) * sin(th) + q * sin(ph) * sin(th))) / (m * cos(th))
        - (2 * zeta1 * (r * cos(ph) * cos(th) - p * sin(th) + q * cos(th) * sin(ph))
            * (y * cos(ph) * cos(ps) - x * cos(ph) * sin(ps) + x * cos(ps) * sin(ph) * sin(th)
                + y * sin(ph) * sin(ps) * sin(th))) / (m * cos(th))
        + (2 * zeta1 * (y * cos(ph) * cos(ps) * cos(th) - x * cos(ph) * cos(th) * sin(ps))
            * (r * cos(ph) + q * sin(ph))) / (m * cos(th))
        - (2 * zeta1 * cos(ph) * cos(ps) * cos(th) * (kt * x - 2 * m * vx)) / pow(m, 2)
        + (2 * zeta1 * cos(ph) * cos(th) * sin(ps) * (2 * m * vy - kt * y)) / pow(m, 2)
        - (2 * zeta1 * sin(th) * (p * cos(th) + r * cos(ph) * sin(th) + q * sin(ph) * sin(th))
            * (y * cos(ph) * cos(ps) - x * cos(ph) * sin(ps) + x * cos(ps) * sin(ph) * sin(th)
                + y * sin(ph) * sin(ps) * sin(th))) / pow(m * cos(th), 2)
        + (2 * vx * zeta1 * cos(ph) * cos(ps) * cos(th)) / m
        + (2 * vy * zeta1 * cos(ph) * cos(th) * sin(ps)) / m
        - (2 * zeta1 * cos(ph) * sin(th) * (q * cos(ph) - r * sin(ph)) * (x * cos(ps) + y * sin(ps))) / m
        + (2 * zeta1 * sin(th) * (r * cos(ph) + q * sin(ph)) * (x * cos(ps) * sin(ph)
            + y * sin(ph) * sin(ps) + y * cos(ph) * cos(ps) * sin(th)
            - x * cos(ph) * sin(ps) * sin(th))) / pow(m * cos(th), 2))
        - (p + (r * cos(ph) * sin(th)) / cos(th) + (q * sin(ph) * sin(th)) / cos(th))
        * ((2 * vy * (zeta1 * cos(ph) * cos(ps) + zeta1 * sin(ph) * sin(ps) * sin(th))) / m
            - (2 * vx * (zeta1 * cos(ph) * sin(ps) - zeta1 * cos(ps) * sin(ph) * sin(th))) / m
            + (2 * zeta2 * (y * cos(ph) * cos(ps) - x * cos(ph) * sin(ps)
                + x * cos(ps) * sin(ph) * sin(th) + y * sin(ph) * sin(ps) * sin(th))) / m
            - (2 * zeta1 * sin(th) * (q * cos(ph) - r * sin(ph)) * (x * cos(ps) + y * sin(ps))) / m
            - (2 * zeta1 * (r * cos(ph) + q * sin(ph)) * (x * cos(ps) * sin(ph)
                + y * sin(ph) * sin(ps) + y * cos(ph) * cos(ps) * sin(th)
                - x * cos(ph) * sin(ps) * sin(th))) / (m * cos(th))
            - (2 * zeta1 * cos(ph) * cos(th) * (x * cos(ph) * cos(ps) * cos(th)
                + y * cos(ph) * cos(th) * sin(ps))) / pow(m * cos(th), 2))
        + (p * (r * cos(ph) + q * sin(ph)) * (2 * zeta2 * (x * cos(ps) * sin(ph)
            + y * sin(ph) * sin(ps) + y * cos(ph) * cos(ps) * sin(th)
            - x * cos(ph) * sin(ps) * sin(th))) / m) / pow(cos(th), 2)
        + (2 * p * (q * cos(ph) - r * sin(ph)) * (zeta1 * (y * cos(ph) * cos(ps) * cos(th)
            - x * cos(ph) * cos(th) * sin(ps))) / m) / pow(cos(th), 2);

    double xi2_1 = z - des_height;
    double xi2_2 = vz;
    double xi2_3 = (dist_z + m * g - kt * vz + zeta1 * cos(th) * cos(ph)) / m;
    double xi2_4 = (q * cos(ph) - r * sin(ph)) * (zeta2 * cos(th) * cos(ph)) / m
        + (2 * zeta1 * sin(th) * (p * cos(th) + r * cos(ph) * sin(th) + q * sin(ph) * sin(th))) / (m * cos(th))
        + (zeta1 * sin(ph) * (p + (r * cos(ph) * sin(th)) / cos(th) + (q * sin(ph) * sin(th)) / cos(th))
            * cos(th) * cos(ph)) / (m * cos(th)) - (zeta1 * cos(th) * cos(ph) * (q * cos(ph) - r * sin(ph))) / m
        + (2 * zeta2 * vz * cos(th) * cos(ph)) / m + (2 * zeta1 * cos(ph) * sin(th) * vz) / m;

    double Lg1Lf3S2 = (cos(th) * cos(ph)) / m;
    double Lg2Lf3S2 = -(zeta1 * (r * cos(ph) + q * sin(ph)) * sin(ph) * sin(th)) / (Ix * m * cos(th));
    double Lg3Lf3S2 = ((zeta1 * sin(ph) * cos(th) * cos(ph)) / (Iy * m)
        + (zeta1 * pow(cos(ph), 2) * sin(th) * (r * cos(ph) + q * sin(ph))) / (Iy * m * cos(th))) / Iy;
    double Lg4Lf3S2 = (zeta1 * sin(ph) * (r * cos(ph) + q * sin(ph)) * sin(th) * sin(ph)) / (Iz * m * cos(th));

    double Lf4S2 = (p * (r * cos(ph) + q * sin(ph)) * (cos(ph) * cos(th)) / m) / pow(cos(th), 2)
        - (2 * vz * sin(th) * cos(ph) * (p * cos(th) + r * cos(ph) * sin(th)
            + q * sin(ph) * sin(th))) / pow(m * cos(th), 2) + (vz * cos(ph) * (p + (r * cos(ph) * sin(th)) / cos(th)
                + (q * sin(ph) * sin(th)) / cos(th)) * (cos(ph) * cos(th))) / (m * cos(th))
        - (vz * cos(ph) * cos(th) * (q * cos(ph) - r * sin(ph))) / m;

    // Define the D matrix
    Eigen::Matrix4d D;
    D << Lg1Lf3S1, Lg2Lf3S1, Lg3Lf3S1, Lg4Lf3S1,
        Lg1Lf3S2, Lg2Lf3S2, Lg3Lf3S2, Lg4Lf3S2,
        Lg1Lf3P1, Lg2Lf3P1, Lg3Lf3P1, Lg4Lf3P1,
        Lg1LfP2, Lg2LfP2, Lg3LfP2, Lg4LfP2;

    // Compute inverse and determinant of D
    Eigen::Matrix4d inv_D = D.inverse();
    double det_d = D.determinant();

    // Calculate v1_tran, v2_tran, v1_tang, and v2_tang
    double v1_tran = -Lf4S1 - k1 * xi1_1 - k2 * xi1_2 - k3 * xi1_3 - k4 * xi1_4;
    double v2_tran = -Lf4S2 - k5 * xi2_1 - k6 * xi2_2 - k7 * xi2_3 - k8 * xi2_4;

    double v1_tang = -Lf4P1 - k10 * (eta1_2 + 0.3) - k11 * eta1_3 - k12 * eta1_4;
    double v2_tang = -Lf2P2 - k13 * (eta2_1 - M_PI / 4) - k14 * eta2_2;

    // Compute control inputs
    Eigen:: Vector4d v(v1_tran, v2_tran, v1_tang, v2_tang);
    Eigen::Vector4d u = inv_D * v;

    double u_new = u(0);
    tau(0) = u(1);
    tau(1) = u(2);
    tau(3) = u(3);
    dt = 0.01; 
    // Update zeta
    zeta2 += dt * u_new;
    zeta1 += dt * zeta2;
    thrust=zeta1;

    *controller_torque_thrust << tau, thrust;
    /*// Geometric controller based on: 
    // T. Lee, M. Leok and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3),
    // " 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, USA, 2010.


    // Trajectory tracking.
    double thrust;
    Eigen::Matrix3d R_d_w;

    // Compute translational tracking errors.
    const Eigen::Vector3d e_p =
                position_W_ - r_position_W_;
    
    const Eigen::Vector3d e_v = 
                velocity_W_ - r_velocity_W_;
    
    const Eigen::Vector3d I_a_d = -position_gain_.cwiseProduct(e_p)
                                -velocity_gain_.cwiseProduct(e_v)
                                +_uav_mass * _gravity * Eigen::Vector3d::UnitZ() + _uav_mass * r_acceleration_W_;
    thrust = I_a_d.dot(R_B_W_.col(2));
    Eigen::Vector3d B_z_d;
    B_z_d = I_a_d;
    B_z_d.normalize();

    // Calculate Desired Rotational Matrix
    const Eigen::Vector3d B_x_d(std::cos(r_yaw), std::sin(r_yaw), 0.0);
    Eigen::Vector3d B_y_d = B_z_d.cross(B_x_d);
    B_y_d.normalize();
    R_d_w.col(0) = B_y_d.cross(B_z_d);
    R_d_w.col(1) = B_y_d;
    R_d_w.col(2) = B_z_d;
    
    Eigen::Quaterniond q_temp(R_d_w);
    *desired_quaternion = q_temp;
    
    // Attitude tracking.
    Eigen::Vector3d tau;

    const Eigen::Matrix3d e_R_matrix =
            0.5 * (R_d_w.transpose() * R_B_W_ - R_B_W_.transpose() * R_d_w)   ;
    Eigen::Vector3d e_R;
    e_R << e_R_matrix(2, 1), e_R_matrix(0, 2), e_R_matrix(1, 0);
    const Eigen::Vector3d omega_ref =
            r_yaw_rate * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d e_omega = angular_velocity_B_ - R_B_W_.transpose() * R_d_w * omega_ref;
    tau = -attitude_gain_.cwiseProduct(e_R)
           - angular_rate_gain_.cwiseProduct(e_omega)
           + angular_velocity_B_.cross(_inertia_matrix.asDiagonal() * angular_velocity_B_);*/

    // Output the wrench
   
}
