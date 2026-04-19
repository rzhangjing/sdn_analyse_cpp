#pragma once

#include <QVector>
#include <QMap>
#include <QSet>
#include <QPair>
#include <QString>
#include <QTextStream>
#include <optional>
#include <functional>
#include "floyd_warshall.h"

/// 一条链路的完整信息
///
/// 字段对应算法标识：
/// - `source`       : st_e  — 链路起始节点
/// - `target`       : en_e  — 链路终止节点
/// - `bandwidth`    : Be    — 链路带宽（Gbps）
/// - `bec`          : bec   — Ec 中链路的专用网络带宽（≤ Be）
/// - `delayMs`      :       — 原始延迟值（ms）
/// - `weight`       : We    — 链路成本权重（秒）= delayMs / 1000
/// - `hsEG`         : hs_e_G — 总回避度，对所有 (Ci, Cj) 对的 YG_e_ci_cj 累加和
/// - `weightedCost` : Wep   — 加权成本 = We × hs_e_G
struct NetworkEdge {
    /// st_e：链路起始节点
    quint32 source;
    /// en_e：链路终止节点
    quint32 target;
    /// Be：链路带宽（单位 Gbps）
    double bandwidth;
    /// bec：Ec 中该链路的专用网络带宽（单位 Gbps，必须 ≤ Be）
    double bec;
    /// 原始延迟值（单位 ms）
    double delayMs;
    /// We：链路成本权重（秒）= delayMs / 1000
    double weight;
    /// hs_e_G：链路总回避度
    double hsEG;
    /// Wep：链路加权成本 = We × hs_e_G
    double weightedCost;
    // 最短路径
    QVector<quint32> paths;

    NetworkEdge() 
        : source(0), target(0), bandwidth(0.0), bec(0.0), 
          delayMs(0.0), weight(0.0), hsEG(0.0), weightedCost(0.0) {}
    
    /// 从原始边属性创建链路实例
    NetworkEdge(quint32 src, quint32 tgt, double bw, double delay)
        : source(src), target(tgt), bandwidth(bw), bec(bw),
          delayMs(delay), weight(delay / 1000.0), hsEG(0.0), weightedCost(0.0) {}
};

/// EECN 图构建的结果
struct EecnGraph {
    /// Vc：边缘服务器节点集合
    QVector<quint32> servers;
    /// Gc 中保留的链路集合 Ec（包含完整链路信息）
    QVector<NetworkEdge> edges;
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
    /// FloydWarshallResult
    FloydWarshallResult floydWarshallRes;
    /// V：所有节点集合（去重后的全部顶点）
    QVector<quint32> v;
    /// E：原始图 G 的全部链路（含完整 NetworkEdge 信息）
    QVector<NetworkEdge> eSet;
    /// E_abs：E 的总条数
    int eAbs;
    /// Vc：边缘服务器节点集合
    QVector<quint32> vc;
    /// MINA = 0.01
    double mina;
    /// MAXW：E 中最大的链路成本权重 We
    double maxw;
    /// hG_vi_vj：G 上所有节点对的最短跳数矩阵
    QMap<QPair<quint32, quint32>, quint32> gHopMap;
    /// 生成的Eecn图
    Graph gc;
    /// 所有边缘服务器对最短路径上的边
    QSet<QPair<quint32, quint32>> gcSet;
    // 节点对快速查找表
    QMap<QPair<quint32, quint32>, NetworkEdge> edgeMap;
    /// 生成的初始 Gc
    QVector<NetworkEdge> gcInitial;
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
