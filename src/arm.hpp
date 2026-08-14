#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include "vec2.hpp"
#include "mat.hpp"
#include "obstacle.hpp"

enum class IKMethod { Transpose, PseudoInverse, DLS };

struct Mat3 {
    double m[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    static Mat3 translation(double x, double y) { Mat3 r; r.m[0][2] = x; r.m[1][2] = y; return r; }
    static Mat3 rotTrans(double theta, double len) {
        Mat3 r; double c = std::cos(theta), s = std::sin(theta);
        r.m[0][0] = c; r.m[0][1] = -s; r.m[0][2] = len * c;
        r.m[1][0] = s; r.m[1][1] = c; r.m[1][2] = len * s;
        r.m[2][0] = 0; r.m[2][1] = 0; r.m[2][2] = 1;
        return r;
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 out;
        for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { double s = 0; for (int k = 0; k < 3; k++) s += m[i][k] * o.m[k][j]; out.m[i][j] = s; }
        return out;
    }
    Vec2 origin() const { return Vec2(m[0][2], m[1][2]); }
};

struct StepResult {
    bool converged = false;
    bool singular = false;
    bool unreachable = false;
    bool angleLimited = false;
    double errorNorm = 0.0;
    double manipulability = 0.0;
};

struct RobotArm {
    int n;
    std::vector<double> lengths;
    std::vector<double> angles;
    Vec2 base;
    double angleLimitDeg = 165.0;
    double singularEps = 0.02;
    double maxStep = 0.12;
    double dlsLambda0 = 0.6;
    double obstacleGain = 2200.0;
    double obstacleInfluence = 55.0;

    bool stallHasTarget = false;
    Vec2 stallTarget;
    double stallBestError = 1e18;
    int stallCounter = 0;
    static constexpr int STALL_LIMIT = 180;
    static constexpr double STALL_EPS = 0.5;

    RobotArm(int n_, std::vector<double> lens, Vec2 base_) : n(n_), lengths(std::move(lens)), angles(n_, 0.0), base(base_) {}

    void setJointCount(int n_) {
        n = n_;
        lengths.resize(n, lengths.empty() ? 80.0 : lengths.back());
        angles.resize(n, 0.0);
    }

    std::vector<Vec2> jointPositions() const {
        std::vector<Vec2> pts;
        pts.reserve(n + 1);
        Mat3 g = Mat3::translation(base.x, base.y);
        pts.push_back(g.origin());
        for (int i = 0; i < n; i++) { g = g * Mat3::rotTrans(angles[i], lengths[i]); pts.push_back(g.origin()); }
        return pts;
    }

    Vec2 endEffector() const { return jointPositions().back(); }

    Mat jacobian(const std::vector<Vec2>& joints) const {
        Vec2 ee = joints.back();
        Mat J = matCreate(2, n);
        for (int i = 0; i < n; i++) {
            Vec2 piv = joints[i];
            J[0][i] = -(ee.y - piv.y);
            J[1][i] = (ee.x - piv.x);
        }
        return J;
    }

    double totalLength() const { double s = 0; for (double l : lengths) s += l; return s; }

    bool targetOutOfReach(const Vec2& target) const { return (target - base).length() > totalLength() + 1e-6; }

    void clampAngles() {
        double lim = angleLimitDeg * M_PI / 180.0;
        for (int i = 0; i < n; i++) angles[i] = std::max(-lim, std::min(lim, angles[i]));
    }

    int countClampedJoints() const {
        double lim = angleLimitDeg * M_PI / 180.0;
        int c = 0;
        for (int i = 0; i < n; i++) if (std::fabs(std::fabs(angles[i]) - lim) < 1e-6) c++;
        return c;
    }

    Row obstacleAvoidTheta(const std::vector<Vec2>& joints, const std::vector<Obstacle>& obstacles) const {
        Row dtheta(n, 0.0);
        for (int k = 0; k < n; k++) {
            Vec2 a = joints[k], b = joints[k + 1];
            for (const auto& o : obstacles) {
                Vec2 cp = closestPointOnSegment(a, b, o.pos);
                double d = (cp - o.pos).length() - o.radius;
                if (d < obstacleInfluence && d > 1e-6) {
                    Vec2 dir = (cp - o.pos).normalized();
                    double mag = obstacleGain * (1.0 / d - 1.0 / obstacleInfluence) * (1.0 / (d * d));
                    Vec2 frep = dir * mag;
                    for (int j = 0; j <= k; j++) {
                        Vec2 piv = joints[j];
                        Vec2 r = cp - piv;
                        dtheta[j] += r.x * frep.y - r.y * frep.x;
                    }
                }
            }
        }
        return dtheta;
    }

    StepResult step(const Vec2& target, IKMethod method, const std::vector<Obstacle>& obstacles, bool avoidObstacles) {
        StepResult res;
        if (targetOutOfReach(target)) { res.unreachable = true; stallHasTarget = false; return res; }

        if (!stallHasTarget || (target - stallTarget).length() > 1e-6) {
            stallTarget = target;
            stallHasTarget = true;
            stallBestError = 1e18;
            stallCounter = 0;
        }

        std::vector<Vec2> joints = jointPositions();
        Vec2 ee = joints.back();
        Vec2 errV = target - ee;
        res.errorNorm = errV.length();
        Mat J = jacobian(joints);
        Mat Jt = matTranspose(J);
        Mat JJt = matMul(J, Jt);
        double det = mat2Det(JJt);
        res.manipulability = det > 0 ? std::sqrt(det) : 0.0;
        res.singular = res.manipulability < singularEps;
        if (res.errorNorm < 1.0) { res.converged = true; stallHasTarget = false; return res; }

        if (res.errorNorm < stallBestError - STALL_EPS) {
            stallBestError = res.errorNorm;
            stallCounter = 0;
        } else {
            stallCounter++;
        }
        if (stallCounter > STALL_LIMIT && countClampedJoints() > 0) {
            res.unreachable = true;
            res.angleLimited = true;
            return res;
        }

        Row e = { errV.x, errV.y };
        Row dtheta(n, 0.0);
        Mat JJtInv;
        bool invertible = mat2Inverse(JJt, JJtInv);

        if (method == IKMethod::Transpose || (method == IKMethod::PseudoInverse && !invertible)) {
            Row Jte = matVec(Jt, e);
            Row JJte = matVec(J, Jte);
            double num = e[0] * JJte[0] + e[1] * JJte[1];
            double den = JJte[0] * JJte[0] + JJte[1] * JJte[1];
            double alpha = den > 1e-12 ? num / den : 0.0;
            for (int i = 0; i < n; i++) dtheta[i] = alpha * Jte[i];
        } else if (method == IKMethod::PseudoInverse) {
            Row t = matVec(JJtInv, e);
            dtheta = matVec(Jt, t);
        } else {
            double lam2 = 0.0001;
            if (res.manipulability < 1.5) {
                double ratio = res.manipulability / 1.5;
                lam2 = dlsLambda0 * dlsLambda0 * (1.0 - ratio * ratio);
            }
            Mat damped = matCreate(2, 2);
            damped[0][0] = JJt[0][0] + lam2; damped[0][1] = JJt[0][1];
            damped[1][0] = JJt[1][0]; damped[1][1] = JJt[1][1] + lam2;
            Mat dampedInv;
            mat2Inverse(damped, dampedInv);
            Row t = matVec(dampedInv, e);
            dtheta = matVec(Jt, t);
        }

        if (avoidObstacles && !obstacles.empty()) {
            Row avoid = obstacleAvoidTheta(joints, obstacles);
            Mat Jplus;
            if (invertible) {
                Mat t = matMul(Jt, JJtInv);
                Mat proj = matSub(matIdentity(n), matMul(t, J));
                Row projected = matVec(proj, avoid);
                for (int i = 0; i < n; i++) dtheta[i] += projected[i] * 0.02;
            }
        }

        double stepNorm = 0.0;
        for (double v : dtheta) stepNorm += v * v;
        stepNorm = std::sqrt(stepNorm);
        if (stepNorm > maxStep) { double s = maxStep / stepNorm; for (double& v : dtheta) v *= s; }

        for (int i = 0; i < n; i++) angles[i] += dtheta[i];
        clampAngles();
        return res;
    }
};