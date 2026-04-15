#include "network_deployment.h"
#include "read_netdata.h"
#include "graph.h"
#include "dijkstra.h"
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QTextStream>
#include <QDateTime>
#include <QDebug>
#include <QtMath>
#include <cmath>
#include <limits>
#include <algorithm>

namespace network_deployment {

void teeWriteln(QTextStream& logStream, const QString& msg) {
    //qDebug().noquote() << msg;
    logStream << msg << Qt::endl;
}

QPair<bool, GraphShortestPath> 
networkDeploymentDelay(const QString& csvFile, QString* errorMsg) {
    GraphShortestPath graphResult;
    
    // 读取边数据
    QString readError;
    auto [success, edges] = read_netdata::readGraph(csvFile, &readError);
    if (!success) {
        if (errorMsg) {
            *errorMsg = QString("读取图数据失败: %1").arg(readError);
        }
        return qMakePair(false, graphResult);
    }
    
    // 构建图
    if (!Graph::fromEdges(edges, graphResult.graph, errorMsg)) {
        return qMakePair(false, graphResult);
    }
    
    // 获取图中所有节点
    QList<quint32> nodeList = graphResult.graph.nodes();
    
    // 计算每对节点之间的最小延时
    for (quint32 source : nodeList) {
        // 执行 Dijkstra 算法
        ShortestPathResult result;
        QString dijkstraError;
        if (!dijkstra(graphResult.graph, source, result, &dijkstraError)) {
            if (errorMsg) {
                *errorMsg = QString("Dijkstra 算法失败: %1").arg(dijkstraError);
            }
            continue;
        }
        
        // 获取到所有其他节点的最短距离（延时）和路径
        for (quint32 target : nodeList) {
            if (source != target) {
                double delay = result.distance(target);
                if (!qIsInf(delay)) {
                    NodeShortestPath nodePath;
                    nodePath.source = source;
                    nodePath.target = target;
                    nodePath.distance = delay;
                    nodePath.path = result.path(target);
                    graphResult.pathMap.insert(qMakePair(source, target), nodePath);
                }
            }
        }
    }
    
    return qMakePair(true, graphResult);
}

std::optional<KCenterResult> kCenterAlgorithm(
    const QVector<std::tuple<quint32, quint32, double>>& delays, 
    int k) {
    
    if (k <= 0 || delays.isEmpty()) {
        return std::nullopt;
    }
    
    // 构建距离矩阵和节点集合
    QMap<QPair<quint32, quint32>, double> distanceMap;
    QSet<quint32> nodeSet;
    
    for (const auto& [source, target, delay] : delays) {
        distanceMap.insert(qMakePair(source, target), delay);
        nodeSet.insert(source);
        nodeSet.insert(target);
    }
    
    QVector<quint32> nodeList;
    for (quint32 node : nodeSet) {
        nodeList.append(node);
    }
    int n = nodeList.size();
    
    if (k >= n) {
        // 如果k大于等于节点数，所有节点都是中心
        KCenterResult result;
        for (quint32 node : nodeList) {
            result.centers.append(node);
            result.centerStats.append(std::make_tuple(node, 0, 0.0, 0.0));
        }
        return result;
    }
    
    // 辅助函数：获取两个节点之间的距离
    auto getDistance = [&distanceMap](quint32 a, quint32 b) -> double {
        if (a == b) {
            return 0.0;
        }
        return distanceMap.value(qMakePair(a, b), std::numeric_limits<double>::infinity());
    };
    
    // 贪心算法选择k个中心
    // 1. 选择第一个中心（选择与其他节点平均距离最小的节点）
    quint32 bestFirstCenter = nodeList[0];
    double minAvgDist = std::numeric_limits<double>::infinity();
    
    for (quint32 candidate : nodeList) {
        double totalDist = 0.0;
        for (quint32 node : nodeList) {
            totalDist += getDistance(candidate, node);
        }
        double avgDist = totalDist / n;
        if (avgDist < minAvgDist) {
            minAvgDist = avgDist;
            bestFirstCenter = candidate;
        }
    }
    
    QVector<quint32> centers;
    centers.append(bestFirstCenter);
    QSet<quint32> centerSet;
    centerSet.insert(bestFirstCenter);
    
    // 2. 迭代选择剩余的k-1个中心
    while (centers.size() < k) {
        double maxMinDistance = -1.0;
        quint32 nextCenter = nodeList[0];
        
        // 对于每个非中心节点，计算它到最近中心的距离
        for (quint32 node : nodeList) {
            if (centerSet.contains(node)) {
                continue;
            }
            
            // 计算该节点到所有中心的最小距离
            double minDistToCenter = std::numeric_limits<double>::infinity();
            for (quint32 center : centers) {
                double dist = getDistance(node, center);
                if (dist < minDistToCenter) {
                    minDistToCenter = dist;
                }
            }
            
            // 选择距离当前中心集合最远的节点作为下一个中心
            if (minDistToCenter > maxMinDistance) {
                maxMinDistance = minDistToCenter;
                nextCenter = node;
            }
        }
        
        centers.append(nextCenter);
        centerSet.insert(nextCenter);
    }
    
    // 3. 将每个非中心节点关联到延迟最小的中心节点
    QVector<std::tuple<quint32, quint32, double>> nodeAssignments;
    QMap<quint32, QVector<QPair<quint32, double>>> centerToNodes;
    
    for (quint32 node : nodeList) {
        if (centerSet.contains(node)) {
            continue; // 跳过中心节点本身
        }
        
        // 找到距离该节点最近的中心
        double minDist = std::numeric_limits<double>::infinity();
        quint32 nearestCenter = centers[0];
        
        for (quint32 center : centers) {
            double dist = getDistance(node, center);
            if (dist < minDist) {
                minDist = dist;
                nearestCenter = center;
            }
        }
        
        nodeAssignments.append(std::make_tuple(node, nearestCenter, minDist));
        centerToNodes[nearestCenter].append(qMakePair(node, minDist));
    }
    
    // 4. 计算每个中心节点的统计信息
    QVector<std::tuple<quint32, int, double, double>> centerStats;
    double globalMaxDelay = 0.0;
    double totalAveDelay = 0.0;
    
    for (quint32 center : centers) {
        QVector<QPair<quint32, double>> associatedNodes = centerToNodes.value(center);
        int nodeCount = associatedNodes.size();
        
        if (nodeCount == 0) {
            // 该中心没有关联的节点
            centerStats.append(std::make_tuple(center, 0, 0.0, 0.0));
            continue;
        }
        
        // 计算该中心的max_delay_k和ave_delay_k
        double maxDelayK = 0.0;
        double totalDelay = 0.0;
        for (const auto& [node, dist] : associatedNodes) {
            Q_UNUSED(node);
            maxDelayK = qMax(maxDelayK, dist);
            totalDelay += dist;
        }
        double aveDelayK = totalDelay / nodeCount;
        
        centerStats.append(std::make_tuple(center, nodeCount, maxDelayK, aveDelayK));
        
        // 更新全局统计
        if (maxDelayK > globalMaxDelay) {
            globalMaxDelay = maxDelayK;
        }
        totalAveDelay += aveDelayK;
    }
    
    double aveDelay = totalAveDelay / k;
    
    KCenterResult result;
    result.centers = centers;
    result.nodeAssignments = nodeAssignments;
    result.centerStats = centerStats;
    result.maxDelay = globalMaxDelay;
    result.aveDelay = aveDelay;
    
    return result;
}

std::optional<std::tuple<quint32, double, double>> findControlNode(
    const QVector<std::tuple<quint32, quint32, double>>& delays) {
    
    // 统计每个节点作为source时的所有delay
    QMap<quint32, QVector<double>> nodeDelays;
    
    for (const auto& [source, target, delay] : delays) {
        Q_UNUSED(target);
        nodeDelays[source].append(delay);
    }
    
    if (nodeDelays.isEmpty()) {
        return std::nullopt;
    }
    
    // 计算每个节点的平均delay，找出平均delay最小的节点
    double minAvgDelay = std::numeric_limits<double>::infinity();
    quint32 controlNode = 0;
    QVector<double> controlNodeDelays;
    
    for (QMap<quint32, QVector<double>>::const_iterator it = nodeDelays.constBegin(); it != nodeDelays.constEnd(); ++it) {
        quint32 node = it.key();
        const QVector<double>& delaysList = it.value();
        
        double totalDelay = 0.0;
        for (double d : delaysList) {
            totalDelay += d;
        }
        double avgDelay = totalDelay / delaysList.size();
        
        if (avgDelay < minAvgDelay) {
            minAvgDelay = avgDelay;
            controlNode = node;
            controlNodeDelays = delaysList;
        }
    }
    
    if (controlNodeDelays.isEmpty()) {
        return std::nullopt;
    }
    
    // 计算控制节点的最大距离和平均距离
    double maxDistance = 0.0;
    double totalDistance = 0.0;
    for (double d : controlNodeDelays) {
        maxDistance = qMax(maxDistance, d);
        totalDistance += d;
    }
    double avgDistance = totalDistance / controlNodeDelays.size();
    
    return std::make_tuple(controlNode, maxDistance, avgDistance);
}

bool writeDelaysToCsv(const QString& csvFile, 
    const QVector<std::tuple<quint32, quint32, double>>& delays,
    QString* errorMsg) {
    
    QFileInfo inputInfo(csvFile);
    QString inputStem = inputInfo.baseName();
    QString outputFilename = QString("%1_delays_output.csv").arg(inputStem);
    QString outputPath = inputInfo.absoluteDir().filePath(outputFilename);
    
    QFile file(outputPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        if (errorMsg) {
            *errorMsg = QString("创建CSV写入器失败: %1").arg(outputPath);
        }
        return false;
    }
    
    QTextStream out(&file);
    out.setEncoding(QStringConverter::Utf8);
    
    // 写入表头
    out << "source\ttarget\tdelay" << Qt::endl;
    
    // 写入数据
    for (const auto& [source, target, delay] : delays) {
        out << source << "\t" << target << "\t" 
            << QString::number(delay, 'f', 3) << " ms" << Qt::endl;
    }
    
    file.close();
    qDebug().noquote() << QString("结果已保存到: %1").arg(outputPath);
    return true;
}

void printKCenterResult(QTextStream& logStream, 
    const KCenterResult& result, int runId, int k, const QString& percentage) {
    
    teeWriteln(logStream, QString("\n========== K-中心算法结果 (第%1次, k=%2, 节点数的%3) ==========")
        .arg(runId).arg(k).arg(percentage));
    
    // 输出找到的k个中心节点
    QStringList centerList;
    for (quint32 c : result.centers) {
        centerList.append(QString::number(c));
    }
    teeWriteln(logStream, QString("选中的 %1 个中心节点: [%2]").arg(k).arg(centerList.join(", ")));
    
    // 输出各节点与中心节点的关联
    teeWriteln(logStream, "\n各节点与中心节点的关联:");
    for (const auto& [node, center, delay] : result.nodeAssignments) {
        teeWriteln(logStream, QString("  节点 %1 -> 中心节点 %2 (延迟: %3 ms)")
            .arg(node).arg(center).arg(delay, 0, 'f', 3));
    }
    
    // 输出每个中心节点的统计信息
    teeWriteln(logStream, "\n每个中心节点的统计信息:");
    for (const auto& [center, count, maxDelayK, aveDelayK] : result.centerStats) {
        teeWriteln(logStream, QString("  中心节点 %1: 关联 %2 个节点, max_delay_k=%3 ms, ave_delay_k=%4 ms")
            .arg(center).arg(count).arg(maxDelayK, 0, 'f', 3).arg(aveDelayK, 0, 'f', 3));
    }
    
    // 输出全局统计
    teeWriteln(logStream, "\n全局统计:");
    teeWriteln(logStream, QString("  max_delay (所有中心节点max_delay_k的最大值): %1 ms")
        .arg(result.maxDelay, 0, 'f', 3));
    teeWriteln(logStream, QString("  ave_delay (所有中心节点ave_delay_k的平均值): %1 ms")
        .arg(result.aveDelay, 0, 'f', 3));
    
    teeWriteln(logStream, "============================================================\n");
}

void printDelayRatio(QTextStream& logStream, 
    double maxDelay, double aveDelay, 
    double controlMax, double controlAvg, 
    int k, const QString& percentage) {
    
    if (controlMax > 0.0 && controlAvg > 0.0) {
        double maxDelayRatio = maxDelay / controlMax;
        double avgDelayRatio = aveDelay / controlAvg;
        teeWriteln(logStream, QString("\n---------- 延迟优化比 (k=%1, %2) ----------")
            .arg(k).arg(percentage));
        teeWriteln(logStream, QString("max_delay_ratio: %1").arg(maxDelayRatio, 0, 'f', 6));
        teeWriteln(logStream, QString("avg_delay_ratio: %1").arg(avgDelayRatio, 0, 'f', 6));
        teeWriteln(logStream, "-------------------------------------------\n");
    }
}

void networkDeploymentFile(const QString& csvFile) {
    // 构建输出日志文件路径（与输入文件同目录）
    QFileInfo inputInfo(csvFile);
    QString logStem = inputInfo.baseName();
    QString logFilename = QString("%1_analysis.log").arg(logStem);
    QString logPath = inputInfo.absoluteDir().filePath(logFilename);
    
    QFile logFile(logPath);
    if (!logFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug().noquote() << QString("创建日志文件失败: %1").arg(logPath);
        return;
    }
    
    QTextStream logStream(&logFile);
    logStream.setEncoding(QStringConverter::Utf8);
    
    teeWriteln(logStream, QString("\n========== 处理文件: %1 ==========").arg(csvFile));
    
    QString errorMsg;
    auto [success, graphResult] = networkDeploymentDelay(csvFile, &errorMsg);

    // 输出最短路径计算结果
    if (success) {
        teeWriteln(logStream, QString("\n---------- 最短路径计算结果 ----------"));
        teeWriteln(logStream, QString("共计算出 %1 对节点间的最短路径").arg(graphResult.pathMap.size()));
        for (auto it = graphResult.pathMap.constBegin(); it != graphResult.pathMap.constEnd(); ++it) {
            const NodeShortestPath& nodePath = it.value();
            teeWriteln(logStream, QString("  节点 %1 -> 节点 %2 : 延迟 %3 ms")
                .arg(nodePath.source).arg(nodePath.target).arg(nodePath.distance, 0, 'f', 3));
        }
        teeWriteln(logStream, QString("--------------------------------------\n"));
    }

    if (success) {
        // 从 pathMap 构造 delays 向量用于后续算法
        QVector<std::tuple<quint32, quint32, double>> delays;
        for (auto it = graphResult.pathMap.constBegin(); it != graphResult.pathMap.constEnd(); ++it) {
            const NodeShortestPath& nodePath = it.value();
            delays.append(std::make_tuple(nodePath.source, nodePath.target, nodePath.distance));
        }
        
        // 按 source 和 target 排序
        std::sort(delays.begin(), delays.end(), [](const auto& a, const auto& b) {
            if (std::get<0>(a) != std::get<0>(b)) {
                return std::get<0>(a) < std::get<0>(b);
            }
            return std::get<1>(a) < std::get<1>(b);
        });
        
        // 找到控制节点（与其他节点平均delay最小的节点）
        std::optional<std::tuple<quint32, double, double>> controlNodeResult = findControlNode(delays);
        quint32 controlNode = 0;
        double controlMaxDistance = 0.0;
        double controlAvgDistance = 0.0;
        
        if (controlNodeResult.has_value()) {
            auto [cn, maxDist, avgDist] = controlNodeResult.value();
            controlNode = cn;
            controlMaxDistance = maxDist;
            controlAvgDistance = avgDist;
            
            teeWriteln(logStream, "\n========== 控制节点分析结果 ==========");
            teeWriteln(logStream, QString("控制节点: %1 (与其他节点平均delay最小)").arg(controlNode));
            teeWriteln(logStream, QString("控制节点到其他节点的最大距离: %1 ms").arg(controlMaxDistance, 0, 'f', 3));
            teeWriteln(logStream, QString("控制节点到其他节点的平均距离: %1 ms").arg(controlAvgDistance, 0, 'f', 3));
            teeWriteln(logStream, "=====================================\n");
        } else {
            teeWriteln(logStream, "无法找到控制节点（可能没有有效的delay数据）");
        }
        
        // 计算节点总数
        QSet<quint32> nodesSet;
        for (const auto& [source, target, delay] : delays) {
            Q_UNUSED(delay);
            nodesSet.insert(source);
            nodesSet.insert(target);
        }
        int nodeCount = nodesSet.size();
        
        // 第一次运行 K-中心算法: k = 节点数 * 0.01
        int k1 = static_cast<int>(qCeil(nodeCount * 0.01));
        std::optional<std::pair<double, double>> ratioK1;
        
        if (k1 > 0) {
            std::optional<KCenterResult> result1 = kCenterAlgorithm(delays, k1);
            if (result1.has_value()) {
                printKCenterResult(logStream, result1.value(), 1, k1, "1%");
                printDelayRatio(logStream, result1.value().maxDelay, result1.value().aveDelay,
                    controlMaxDistance, controlAvgDistance, k1, "1%");
                if (controlMaxDistance > 0.0 && controlAvgDistance > 0.0) {
                    ratioK1 = std::make_pair(
                        result1.value().maxDelay / controlMaxDistance,
                        result1.value().aveDelay / controlAvgDistance);
                }
            } else {
                teeWriteln(logStream, QString("K-中心算法第1次执行失败 (k=%1)").arg(k1));
            }
        }
        
        // 第二次运行 K-中心算法: k = 节点数 * 0.02
        int k2 = static_cast<int>(qCeil(nodeCount * 0.02));
        std::optional<std::pair<double, double>> ratioK2;
        
        if (k2 > 0 && k2 != k1) {
            std::optional<KCenterResult> result2 = kCenterAlgorithm(delays, k2);
            if (result2.has_value()) {
                printKCenterResult(logStream, result2.value(), 2, k2, "2%");
                printDelayRatio(logStream, result2.value().maxDelay, result2.value().aveDelay,
                    controlMaxDistance, controlAvgDistance, k2, "2%");
                if (controlMaxDistance > 0.0 && controlAvgDistance > 0.0) {
                    ratioK2 = std::make_pair(
                        result2.value().maxDelay / controlMaxDistance,
                        result2.value().aveDelay / controlAvgDistance);
                }
            } else {
                teeWriteln(logStream, QString("K-中心算法第2次执行失败 (k=%1)").arg(k2));
            }
        }
        
        // 汇总输出
        QString fileName = inputInfo.fileName();
        teeWriteln(logStream, "\n==================== 汇总输出 ====================");
        teeWriteln(logStream, QString("文件名称: %1").arg(fileName));
        
        if (ratioK1.has_value()) {
            teeWriteln(logStream, QString("k选择比例0.01时的最大距离优化比: %1")
                .arg(ratioK1.value().first, 0, 'f', 6));
            teeWriteln(logStream, QString("k选择比例0.01时的平均距离优化比: %1")
                .arg(ratioK1.value().second, 0, 'f', 6));
        } else {
            teeWriteln(logStream, "k选择比例0.01时的最大距离优化比: N/A");
            teeWriteln(logStream, "k选择比例0.01时的平均距离优化比: N/A");
        }
        
        if (ratioK2.has_value()) {
            teeWriteln(logStream, QString("k选择比例0.02时的最大距离优化比: %1")
                .arg(ratioK2.value().first, 0, 'f', 6));
            teeWriteln(logStream, QString("k选择比例0.02时的平均距离优化比: %1")
                .arg(ratioK2.value().second, 0, 'f', 6));
        } else {
            teeWriteln(logStream, "k选择比例0.02时的最大距离优化比: N/A");
            teeWriteln(logStream, "k选择比例0.02时的平均距离优化比: N/A");
        }
        
        teeWriteln(logStream, "====================================================\n");
        qDebug().noquote() << QString("日志已保存到: %1").arg(logPath);
        
        if (!writeDelaysToCsv(csvFile, delays, &errorMsg)) {
            qDebug().noquote() << QString("写入CSV文件失败: %1").arg(errorMsg);
        }
    } else {
        teeWriteln(logStream, QString("测试跳过: %1").arg(errorMsg));
    }
    
    logFile.close();
}

void networkDeployment() {
    // 定义要处理的文件列表
    QStringList csvFiles = {
        "D:\\张新常\\网络试验学习\\20260103\\网络拓扑\\Waxman-1000-1.txt",
//        "D:\\张新常\\网络试验学习\\20260103\\网络拓扑\\Waxman-2000-1.txt",
//        "D:\\张新常\\网络试验学习\\20260103\\网络拓扑\\Waxman-3000-1.txt",
//        "D:\\张新常\\网络试验学习\\20260103\\网络拓扑\\Waxman-4000-1.txt",
    };
    
    for (const QString& csvFile : csvFiles) {
        networkDeploymentFile(csvFile);
    }
}

} // namespace network_deployment
