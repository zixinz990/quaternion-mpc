#include "Utils.h"

Eigen::Matrix3d Utils::skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d skew;
    skew << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
    return skew;
}

Eigen::Vector3d Utils::quat_to_euler(Eigen::Quaterniond quat) {
    Eigen::Vector3d rst;
    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();

    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y * y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);

    return rst;
}

Eigen::Vector4d Utils::quat_rk4(Eigen::Vector4d q, Eigen::Vector3d w, double dt) {
    Eigen::Vector4d k1, k2, k3, k4;
    k1 = dt * 0.5 * G(q) * w;
    k2 = dt * 0.5 * G(q + 0.5 * k1) * w;
    k3 = dt * 0.5 * G(q + 0.5 * k2) * w;
    k4 = dt * 0.5 * G(q + k3) * w;
    return q + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
}

Eigen::Vector4d Utils::cayley_map(Eigen::Vector3d phi) {
    Eigen::Vector4d phi_quat;
    phi_quat << 1, phi[0], phi[1], phi[2];
    return 1 / (sqrt(1 + phi.norm() * phi.norm())) * phi_quat;
}

Eigen::Vector3d Utils::inv_cayley_map(Eigen::Vector4d q) {
    return q.tail(3) / q[0];
}

Eigen::Vector4d Utils::quat_mult(Eigen::Vector4d q1, Eigen::Vector4d q2) {
    return L(q1) * q2;
}

Eigen::Vector4d Utils::quat_conj(Eigen::Vector4d q) {
    Eigen::Matrix4d T;
    T << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1;
    return T * q;
}

Eigen::Matrix4d Utils::L(Eigen::Vector4d q) {
    Eigen::Matrix4d L;
    L(0, 0) = q[0];
    L.block<1, 3>(0, 1) = -q.tail(3).transpose();
    L.block<3, 1>(1, 0) = q.tail(3);
    L.block<3, 3>(1, 1) = q[0] * Eigen::Matrix3d::Identity() + Utils::skew(q.tail(3));
    return L;
}

Eigen::Matrix4d Utils::R(Eigen::Vector4d q) {
    Eigen::Matrix4d R;
    R(0, 0) = q[0];
    R.block<1, 3>(0, 1) = -q.tail(3).transpose();
    R.block<3, 1>(1, 0) = q.tail(3);
    R.block<3, 3>(1, 1) = q[0] * Eigen::Matrix3d::Identity() - Utils::skew(q.tail(3));
    return R;
}

Eigen::MatrixXd Utils::G(Eigen::Vector4d q) {
    Eigen::MatrixXd H(4, 3);
    H << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    return L(q) * H;
}

ExplicitDynamicsFunction Utils::midpoint_dyn(int n, int m, ContinuousDynamicsFunction f) {
    auto fd = [n, m, f](double *xn, const double *x, const double *u, float h) {
        static Eigen::VectorXd xm(n);
        Eigen::Map<Eigen::VectorXd> xn_vec(xn, n);
        Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
        Eigen::Map<const Eigen::VectorXd> u_vec(u, n);
        f(xm.data(), x, u);
        xm *= h / 2;
        xm.noalias() += x_vec;
        f(xn, xm.data(), u);
        xn_vec = x_vec + h * xn_vec;
    };
    return fd;
}

ExplicitDynamicsJacobian Utils::midpoint_jac(int n, int m, ContinuousDynamicsFunction f, ContinuousDynamicsJacobian df) {
    auto fd = [n, m, f, df](double *jac, const double *x, const double *u, float h) {
        static Eigen::MatrixXd A(n, n);
        static Eigen::MatrixXd B(n, m);
        static Eigen::MatrixXd Am(n, n);
        static Eigen::MatrixXd Bm(n, m);
        static Eigen::VectorXd xm(n);
        static Eigen::MatrixXd In = Eigen::MatrixXd::Identity(n, n);

        Eigen::Map<Eigen::MatrixXd> J(jac, n, n + m);
        Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
        Eigen::Map<const Eigen::VectorXd> u_vec(u, n);

        // Evaluate the midpoint
        f(xm.data(), x, u);
        xm = x_vec + h / 2 * xm;

        // Evaluate the Jacobian
        df(J.data(), x, u);
        A = J.leftCols(n);
        B = J.rightCols(m);

        // Evaluate the Jacobian at the midpoint
        df(J.data(), xm.data(), u);
        Am = J.leftCols(n);
        Bm = J.rightCols(m);

        // Apply the chain rule
        J.leftCols(n) = In + h * Am * (In + h / 2 * A);
        J.rightCols(m) = h * (Am * h / 2 * B + Bm);
    };
    return fd;
}
