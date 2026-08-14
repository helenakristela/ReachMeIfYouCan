#pragma once
#include <vector>
#include <cmath>
using Row = std::vector<double>;
using Mat = std::vector<Row>;
inline Mat matCreate(int r, int c, double v = 0.0) { return Mat(r, Row(c, v)); }
inline Mat matIdentity(int n) { Mat m = matCreate(n, n); for (int i = 0; i < n; i++) m[i][i] = 1.0; return m; }
inline Mat matTranspose(const Mat& a) {
    int r = (int)a.size(), c = (int)a[0].size();
    Mat t = matCreate(c, r);
    for (int i = 0; i < r; i++) for (int j = 0; j < c; j++) t[j][i] = a[i][j];
    return t;
}
inline Mat matMul(const Mat& a, const Mat& b) {
    int r = (int)a.size(), k = (int)a[0].size(), c = (int)b[0].size();
    Mat out = matCreate(r, c);
    for (int i = 0; i < r; i++) for (int j = 0; j < c; j++) { double s = 0; for (int t = 0; t < k; t++) s += a[i][t] * b[t][j]; out[i][j] = s; }
    return out;
}
inline Mat matSub(const Mat& a, const Mat& b) {
    int r = (int)a.size(), c = (int)a[0].size();
    Mat out = matCreate(r, c);
    for (int i = 0; i < r; i++) for (int j = 0; j < c; j++) out[i][j] = a[i][j] - b[i][j];
    return out;
}
inline Row matVec(const Mat& a, const Row& v) {
    int r = (int)a.size(), c = (int)a[0].size();
    Row out(r, 0.0);
    for (int i = 0; i < r; i++) { double s = 0; for (int j = 0; j < c; j++) s += a[i][j] * v[j]; out[i] = s; }
    return out;
}
inline bool mat2Inverse(const Mat& a, Mat& out) {
    double det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if (std::fabs(det) < 1e-12) return false;
    double id = 1.0 / det;
    out = matCreate(2, 2);
    out[0][0] = a[1][1] * id; out[0][1] = -a[0][1] * id;
    out[1][0] = -a[1][0] * id; out[1][1] = a[0][0] * id;
    return true;
}
inline double mat2Det(const Mat& a) { return a[0][0] * a[1][1] - a[0][1] * a[1][0]; }
