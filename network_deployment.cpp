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
#include "network_bandwidth_allocation.h"

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
    nodeList.reserve(nodeSet.size());
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

    // 节点索引映射：node -> index in nodeList
    QMap<quint32, int> nodeToIndex;
    for (int i = 0; i < n; ++i) {
        nodeToIndex[nodeList[i]] = i;
    }

    // minDistances[i] = 节点 nodeList[i] 到最近中心的距离
    QVector<double> minDistances(n, std::numeric_limits<double>::infinity());
    // nearestCenter[i] = 节点 nodeList[i] 的最近中心节点
    QVector<quint32> nearestCenter(n, 0);

    QVector<quint32> centers;
    centers.reserve(k);
    QSet<quint32> centerSet;

    // ==================== 基于网络度和延迟的加权 k-center 算法 ====================

    // 1. 计算每个节点的网络度（degree）：统计该节点作为 source 或 target 出现的次数
    QMap<quint32, int> nodeDegree;
    for (const auto& [source, target, delay] : delays) {
        Q_UNUSED(delay);
        nodeDegree[source]++;
        nodeDegree[target]++;
    }

    // 2. 计算每个节点的平均延迟和中心性分数
    // centrality(v) = degree(v) / avgDelay(v)
    // 中心性越高的节点越适合作为控制器部署位置（高度数 + 低平均延迟）
    QMap<quint32, double> nodeCentrality;
    for (quint32 node : nodeList) {
        double totalDist = 0.0;
        int reachableCount = 0;
        for (quint32 other : nodeList) {
            if (node != other) {
                double dist = getDistance(node, other);
                if (!qIsInf(dist)) {
                    totalDist += dist;
                    reachableCount++;
                }
            }
        }
        double avgDelay = (reachableCount > 0) ? (totalDist / reachableCount) : std::numeric_limits<double>::infinity();
        double degree = static_cast<double>(nodeDegree.value(node, 0));
        // 中心性分数：度数越高、平均延迟越低，分数越高
        double centrality = (avgDelay > 0 && !qIsInf(avgDelay)) ? (degree / avgDelay) : 0.0;
        nodeCentrality[node] = centrality;
    }

    // 3. 选择第一个中心：选择中心性分数最高的节点
    quint32 bestFirstCenter = nodeList[0];
    double maxCentrality = -1.0;
    for (quint32 candidate : nodeList) {
        double centrality = nodeCentrality.value(candidate, 0.0);
        if (centrality > maxCentrality) {
            maxCentrality = centrality;
            bestFirstCenter = candidate;
        }
    }

    centers.append(bestFirstCenter);
    centerSet.insert(bestFirstCenter);

    // 初始化所有节点到第一个中心的距离
    for (int i = 0; i < n; ++i) {
        minDistances[i] = getDistance(nodeList[i], bestFirstCenter);
        nearestCenter[i] = bestFirstCenter;
    }

    // 4. 迭代选择剩余的 k-1 个中心
    // 使用加权 farthest-first 策略：
    // score(v) = minDistToCenter(v) × (1 + α / centrality(v))
    // 其中 α 是权重因子，使得远离现有中心且中心性较低的节点优先被选为新中心
    const double alpha = 0.5;  // 权重因子

    while (centers.size() < k) {
        // 计算每个非中心节点的加权分数
        double maxScore = -1.0;
        int bestIndex = -1;

        for (int i = 0; i < n; ++i) {
            quint32 node = nodeList[i];
            if (centerSet.contains(node)) {
                continue;
            }

            double minDist = minDistances[i];
            double centrality = nodeCentrality.value(node, 0.0);

            // 加权分数：距离越远、中心性越低，分数越高
            // 如果中心性为0或极小，使用基础分数
            double score;
            if (centrality > 0.000001) {
                score = minDist * (1.0 + alpha / centrality);
            } else {
                score = minDist * (1.0 + alpha * 1000.0);  // 中心性极低的节点给予高权重
            }

            if (score > maxScore) {
                maxScore = score;
                bestIndex = i;
            }
        }

        if (bestIndex == -1) {
            break; // 所有节点都已成为中心
        }

        quint32 nextCenter = nodeList[bestIndex];
        centers.append(nextCenter);
        centerSet.insert(nextCenter);

        // 更新所有非中心节点到新中心的距离（增量更新，O(n)每轮）
        for (int i = 0; i < n; ++i) {
            quint32 node = nodeList[i];
            if (centerSet.contains(node)) {
                continue;
            }
            double distToNewCenter = getDistance(node, nextCenter);
            if (distToNewCenter < minDistances[i]) {
                minDistances[i] = distToNewCenter;
                nearestCenter[i] = nextCenter;
            }
        }
    }

    // 5. 构建节点分配结果
    QVector<std::tuple<quint32, quint32, double>> nodeAssignments;
    QMap<quint32, QVector<QPair<quint32, double>>> centerToNodes;

    for (int i = 0; i < n; ++i) {
        quint32 node = nodeList[i];
        if (centerSet.contains(node)) {
            continue; // 跳过中心节点本身
        }
        double minDist = minDistances[i];
        quint32 assignedCenter = nearestCenter[i];
        nodeAssignments.append(std::make_tuple(node, assignedCenter, minDist));
        centerToNodes[assignedCenter].append(qMakePair(node, minDist));
    }

    // 6. 计算每个中心节点的统计信息
    QVector<std::tuple<quint32, int, double, double>> centerStats;
    double globalMaxDelay = 0.0;
    double totalAveDelay = 0.0;

    for (quint32 center : centers) {
        QVector<QPair<quint32, double>> associatedNodes = centerToNodes.value(center);
        int nodeCount = associatedNodes.size();

        if (nodeCount == 0) {
            centerStats.append(std::make_tuple(center, 0, 0.0, 0.0));
            continue;
        }

        double maxDelayK = 0.0;
        double totalDelay = 0.0;
        for (const auto& [node, dist] : associatedNodes) {
            Q_UNUSED(node);
            maxDelayK = qMax(maxDelayK, dist);
            totalDelay += dist;
        }
        double aveDelayK = totalDelay / nodeCount;

        centerStats.append(std::make_tuple(center, nodeCount, maxDelayK, aveDelayK));

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
    /*if (success) {
        teeWriteln(logStream, QString("\n---------- 最短路径计算结果 ----------"));
        teeWriteln(logStream, QString("共计算出 %1 对节点间的最短路径").arg(graphResult.pathMap.size()));
        for (auto it = graphResult.pathMap.constBegin(); it != graphResult.pathMap.constEnd(); ++it) {
            const NodeShortestPath& nodePath = it.value();
            teeWriteln(logStream, QString("  节点 %1 -> 节点 %2 : 延迟 %3 ms")
                .arg(nodePath.source).arg(nodePath.target).arg(nodePath.distance, 0, 'f', 3));
        }
        teeWriteln(logStream, QString("--------------------------------------\n"));
    }*/

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

                    //a为数组C的偏移量  c为数组B的偏移量
                    int a = (int)controlMaxDistance / 1000;
                    int c = (int)result1.value().maxDelay / 1000;
                    network_bandwidth_allocation::networkBandwidthAllocationCapabilityWork(csvFile, k1, a, c);
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
                    //a为数组C的偏移量  c为数组B的偏移量
                    int a = (int)controlMaxDistance / 1000;
                    int c = (int)result2.value().maxDelay / 1000;
                    network_bandwidth_allocation::networkBandwidthAllocationCapabilityWork(csvFile, k1, a, c);
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
        "D:\\张新常\\网络试验学习\\20260103\\网络拓扑\\Waxman-2000-1.txt",
        "D:\\张新常\\网络试验学习\\20260103\\网络拓扑\\Waxman-3000-1.txt",
        "D:\\张新常\\网络试验学习\\20260103\\网络拓扑\\Waxman-4000-1.txt",
    };
    
    for (const QString& csvFile : csvFiles) {
        networkDeploymentFile(csvFile);
    }
}

} // namespace network_deployment
