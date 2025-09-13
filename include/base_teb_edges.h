/**
 * @Author:juchunyu@qq.com
*/

#pragma once 
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/factory.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <boost/thread/once.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <boost/make_shared.hpp>
#include <memory>


namespace teb_local_planner
{
// -----------------------------------------------------------------------------
// 存储全局配置：权重、目标位置等
// -----------------------------------------------------------------------------

struct TebConfig
{
    // 软约束配置：目标位置 + 权重
    double min_obstacle_dist = 0.5;
    double penalty_epsilon   = 0.05;
    double obstacle_weight   = 10;
    double weight_viapoint   = 10;

    // 优化器配置
    bool optimization_verbose = true;
    int no_inner_iterations = 5;
    int no_outer_iterations = 4;
};

// 一元边基类（连接1个顶点）
template <int D, typename E, typename VertexXi>
class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi>
{
public:
    using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

    // 构造函数：初始化顶点指针为NULL
    BaseTebUnaryEdge() { this->_vertices[0] = nullptr; }

    // 析构函数：从顶点的edges列表中删除当前边（避免野指针）
    virtual ~BaseTebUnaryEdge()
    {
        if (this->_vertices[0])
            this->_vertices[0]->edges().erase(this);
    }

    // 统一误差获取接口：计算误差后返回
    ErrorVector& getError()
    {
        computeError();
        return this->_error;
    }

    // 序列化默认实现（满足g2o接口要求，子类可重写）
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    // 统一配置传递接口：设置TebConfig
    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    // 共享配置指针（所有子类边可直接访问）
    const TebConfig* cfg_ = nullptr;
    // 继承g2o的误差和顶点成员
    using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐
};

// 2.2 二元边基类（连接2个顶点）
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
{
public:
    using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

    // 构造函数：初始化顶点指针为NULL
    BaseTebBinaryEdge()
    {
        this->_vertices[0] = nullptr;
        this->_vertices[1] = nullptr;
    }

    // 析构函数：从两个顶点的edges列表中删除当前边
    virtual ~BaseTebBinaryEdge()
    {
        if (this->_vertices[0])
            this->_vertices[0]->edges().erase(this);
        if (this->_vertices[1])
            this->_vertices[1]->edges().erase(this);
    }

    // 统一误差获取接口
    ErrorVector& getError()
    {
        computeError();
        return this->_error;
    }

    // 序列化默认实现
    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    // 统一配置传递接口
    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    // 共享配置指针
    const TebConfig* cfg_ = nullptr;
    // 继承g2o的误差和顶点成员
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace teb_local_planner
