#include <chrono>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <iostream>
#include <random>

using namespace std;


class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    CurveFittingVertex() {}
    virtual bool read(istream&) { return false; }
    virtual bool write(ostream&) const { return false; }
    virtual void setToOriginImpl() { _estimate << 0, 0, 0; }
    virtual void oplusImpl(const double* update)
    {
        Eigen::Vector3d::ConstMapType v(update);
        _estimate += v;
    }
};

class CurveFittingEdge : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    CurveFittingEdge() {}
    virtual bool read(istream&) { return false; }
    virtual bool write(ostream&) const { return false; }

    virtual void computeError() override
    {
        const CurveFittingVertex* v = static_cast<CurveFittingVertex*>(vertex(0));
        const Eigen::Vector3d& abc = v->estimate();
        const double x = _measurement(0);
        _error(0) = exp(abc(0) * x * x + abc(1) * x + abc(2)) - _measurement(1);
    }

    virtual void linearizeOplus() override
    {
        const CurveFittingVertex* v = static_cast<CurveFittingVertex*>(vertex(0));
        const Eigen::Vector3d abc = v->estimate();
        const double x = _measurement(0);
        const double exp_y = exp(abc(0) * x * x + abc(1) * x + abc(2));
        _jacobianOplusXi << x * x * exp_y, x * exp_y, 1 * exp_y;
    }
};

/*
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    void computeError()
    {
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
    }

    virtual bool read(istream& in) {}
    virtual bool write(ostream& out) const {}

public:
    double _x;  // x 值， y 值为 _measurement
};
*/

int main()
{
    double a = 1.0, b = 2.0, c = 1.0;  // 真实参数值
    int N = 300;  // 数据点
    double w_sigma = 1.;  // 噪声Sigma值

    default_random_engine generator;
    normal_distribution<double> noise(0., w_sigma);

    // 构建 problem
    //! NOTE 由于这个例子中只有pose vertex(3自由度), 没有landmark, 所以第二个模板参数是一个非零值即可
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 99>> BlockSolverType;  // 3x1 3x2 3x3 都可以, Why?
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    g2o::SparseOptimizer optimizer;
    LinearSolverType* linearSolver = new LinearSolverType();
    BlockSolverType* blockSolver = new BlockSolverType(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* algo = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    //g2o::OptimizationAlgorithmGaussNewton* algo = new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
    optimizer.setAlgorithm(algo);

    // 构建顶点
    CurveFittingVertex* params = new CurveFittingVertex();
    params->setId(0);
    params->setEstimate(Eigen::Vector3d(2, -1, -5));
    optimizer.addVertex(params);

    // 构建边
    for (int i = 0; i < N; ++i) {
        const double x = i * 1. / N;
        const double n = noise(generator);
        const double y = exp(a * x * x + b * x + c) + n;

        CurveFittingEdge* e = new CurveFittingEdge();
        e->setId(i);
        e->setVertex(0, params);
        e->setMeasurement(Eigen::Vector2d(x, y));
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
        optimizer.addEdge(e);
    }

    cout << "\nTest CurveFitting start..." << endl;

    auto edges = optimizer.activeEdges();
    for (size_t i = 0; i < 4; ++i) {
        int outliers = 0;
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        optimizer.initializeOptimization();
        optimizer.setVerbose(true);
        optimizer.optimize(15);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "Solve time cost = " << time_used.count() * 1000 << "ms. " << endl;
        for (auto& e : edges) {
            if (e->level() > 0)
                continue;
            if (e->chi2() > 2 * w_sigma) {
                e->setLevel(1);
                outliers++;
            }
        }
        cout << " Result after Iteration " << i << " is: " << params->estimate().transpose()
             << ", tatal chi2 = " << optimizer.chi2() << ", outliers = " << outliers << endl;
    }

    cout << "The result is: " << params->estimate().transpose() << endl;
    cout << "The ground truth is: 1, 2, 1" << endl;

    return 0;
}
