#pragma once

#include <QVector>
#include <QMap>
#include <QSet>
#include <QPair>
#include <QString>
#include <QTextStream>
#include <optional>
#include <tuple>

#include "dijkstra.h"

/// K-中心算法结果
struct KCenterResult {
    /// 选中的k个中心节点
    QVector<quint32> centers;
    /// 每个非中心节点关联的中心节点 (节点 -> 关联的中心节点, 延迟)
    QVector<std::tuple<quint32, quint32, double>> nodeAssignments;
    /// 每个中心节点的统计信息 (中心节点, 关联节点数, max_delay_k, ave_delay_k)
    QVector<std::tuple<quint32, int, double, double>> centerStats;
    /// 所有中心节点中最大的max_delay_k
    double maxDelay;
    /// 所有中心节点的ave_delay_k的平均
    double aveDelay;
    
    KCenterResult() : maxDelay(0.0), aveDelay(0.0) {}
};

namespace network_deployment {

/// 计算所有节点对的最短延迟
/// 返回: QPair<bool, GraphShortestPath>
///   - first: true 表示成功
///   - second: GraphShortestPath 包含图数据和所有节点对的最短路径信息
QPair<bool, GraphShortestPath> 
networkDeploymentDelay(const QString& csvFile, QString* errorMsg = nullptr);

/// K-中心算法
/// 1. 找到k个中心节点
/// 2. 将其他节点关联到延迟最小的中心节点
/// 3. 计算每个中心节点的max_delay_k和ave_delay_k
/// 4. 计算所有中心节点中最大的max_delay和平均的ave_delay
std::optional<KCenterResult> kCenterAlgorithm(
    const QVector<std::tuple<quint32, quint32, double>>& delays, 
    int k);

/// 找到控制节点（与其他节点平均delay最小的节点）
/// 返回: (控制节点, 最大距离, 平均距离)
std::optional<std::tuple<quint32, double, double>> findControlNode(
    const QVector<std::tuple<quint32, quint32, double>>& delays);

/// 将 delays 数据写入 CSV 文件
bool writeDelaysToCsv(const QString& csvFile, 
    const QVector<std::tuple<quint32, quint32, double>>& delays,
    QString* errorMsg = nullptr);

/// 辅助函数：同时输出到 stdout 和日志文件
void teeWriteln(QTextStream& logStream, const QString& msg);

/// 打印 K-中心算法结果
void printKCenterResult(QTextStream& logStream, 
    const KCenterResult& result, int runId, int k, const QString& percentage);

/// 计算并输出延迟优化比
void printDelayRatio(QTextStream& logStream, 
    double maxDelay, double aveDelay, 
    double controlMax, double controlAvg, 
    int k, const QString& percentage);

/// 处理单个网络部署文件
void networkDeploymentFile(const QString& csvFile);

/// 批量处理
void networkDeployment();

} // namespace network_deployment
