#include "../src/arm.hpp"
#include <cstdio>

void run(IKMethod method, const char* name) {
    RobotArm arm(3, { 100, 100, 100 }, Vec2(0, 0));
    Vec2 target(150, 120);
    StepResult r;
    int iter = 0;
    for (; iter < 500; iter++) {
        r = arm.step(target, method, {}, false);
        if (r.converged || r.unreachable) break;
    }
    Vec2 ee = arm.endEffector();
    printf("%s -> iter=%d converged=%d unreachable=%d err=%.3f ee=(%.2f,%.2f)\n", name, iter, r.converged, r.unreachable, r.errorNorm, ee.x, ee.y);
}

void runSingular() {
    RobotArm arm(3, { 100, 100, 100 }, Vec2(0, 0));
    arm.angles = { 0, 0, 0 };
    Vec2 target(299, 0.001);
    for (int i = 0; i < 300; i++) {
        StepResult r = arm.step(target, IKMethod::DLS, {}, false);
        if (i % 50 == 0) printf("dls near-singular iter=%d manip=%.5f singular=%d err=%.3f\n", i, r.manipulability, r.singular, r.errorNorm);
        if (r.converged) { printf("dls near-singular converged at iter=%d\n", i); break; }
    }
    Vec2 ee = arm.endEffector();
    printf("final ee=(%.3f,%.3f) NaN-check x=%d y=%d\n", ee.x, ee.y, std::isnan(ee.x), std::isnan(ee.y));
}

void runUnreachable() {
    RobotArm arm(2, { 50, 50 }, Vec2(0, 0));
    StepResult r = arm.step(Vec2(500, 500), IKMethod::PseudoInverse, {}, false);
    printf("unreachable-test unreachable=%d\n", r.unreachable);
}

int main() {
    run(IKMethod::Transpose, "Transpose");
    run(IKMethod::PseudoInverse, "PseudoInverse");
    run(IKMethod::DLS, "DLS");
    runSingular();
    runUnreachable();
    return 0;
}
