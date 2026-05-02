#pragma once

#include <QVector>
#include <QMap>
#include <QSet>
#include <QPair>
#include <QString>
#include <QTextStream>
#include <optional>
#include <functional>
#include "graph.h"


/// EECN 图构建的结果
struct EecnGraph {
    /// Vc：边缘服务器节点集合
    QVector<quint32> servers;
    /// Gc 中保留的链路集合 Ec（包含完整链路信息）
    QVector<QSharedPointer<NetworkEdge>> edges;
    /// cost_Gc：Gc 的网络成本（Ec 中所有链路成本权重 We 之和）
    double costGc;
    /// E_abs：原始图 G 的链路总条数
    int eAbs;
    
    EecnGraph() : costGc(0.0), eAbs(0) {}
};

/// EECN 图构建算法——输入、过程与结果的全量容器
struct EecnBuild {
    // ----------------------------------------------------------------
    // 输入参数
    // ----------------------------------------------------------------
    /// 有向图数据文件路径
    QString filePath;
    /// EdgeServerNum：边缘服务器数量
    int edgeServerNum;
    /// αmin：跳数减少值下界
    double alphaMin;
    /// αmax：跳数减少值上界（限制服务器间最大跳数）
    double alphaMax;
    /// β：跳数减少平均参数（αmax > β > 1）
    double beta;
    
    // ----------------------------------------------------------------
    // 图构建过程数据
    // ----------------------------------------------------------------
    /// 原始图 G
    Graph g;

    /// V：所有节点集合（去重后的全部顶点）
    QVector<quint32> v;
    /// E：原始图 G 的全部链路（含完整 NetworkEdge 信息）
    QVector<QSharedPointer<NetworkEdge>> eSet;
    /// E_abs：E 的总条数
    int eAbs;
    /// Vc：边缘服务器节点集合
    QVector<quint32> vc;
    /// MINA = 0.01
    double mina;
    /// MAXW：E 中最大的链路成本权重 We
    double maxw;
    /// 生成的Eecn图
    Graph gc;
    /// 生成的初始 Gc
    QVector<QSharedPointer<NetworkEdge>> gcInitial;
    /// 初始 Gc 中不满足跳数约束的服务器对数量
    int hopViolations;
    
    // ----------------------------------------------------------------
    // 算法结果
    // ----------------------------------------------------------------
    /// 最终构建结果
    std::optional<EecnGraph> result;
    
    EecnBuild() 
        : edgeServerNum(0), alphaMin(0.0), alphaMax(0.0), beta(0.0),
          eAbs(0), mina(0.01), maxw(0.0), hopViolations(0) {}
    
    EecnBuild(const QString& path, int serverNum, double aMin, double aMax, double b)
        : filePath(path), edgeServerNum(serverNum), alphaMin(aMin), alphaMax(aMax), beta(b),
          eAbs(0), mina(0.01), maxw(0.0), hopViolations(0) {}
};

namespace network_resource_allocation {

/// EECN 图构建算法入口
std::optional<EecnBuild> eecnGraphBuild(
    const QString& filePath,
    int edgeServerNum,
    double alphaMin,
    double alphaMax,
    double beta);

/// 主入口：批量处理
void networkAllocation();

/// 辅助函数：同时输出到 stdout 和日志文件
void teeWriteln(QTextStream& logStream, const QString& msg);

} // namespace network_resource_allocation
