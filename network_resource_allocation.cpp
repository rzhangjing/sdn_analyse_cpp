#include "network_resource_allocation.h"
#include "read_netdata.h"
#include "graph.h"
#include "floyd_warshall.h"
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QTextStream>
#include <QRandomGenerator>
#include <QDebug>
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <limits>

namespace network_resource_allocation {

void teeWriteln(QTextStream& logStream, const QString& msg) {
    qDebug().noquote() << msg;
    logStream << msg << Qt::endl;
}

// ---------------------------------------------------------------------------
// 辅助函数：构建跳数矩阵
// ---------------------------------------------------------------------------
/// 用 Floyd-Warshall 重建 source -> target 的路径,计算跳数
static QMap<QPair<quint32, quint32>, quint32> buildHopMatrix(FloydWarshallResult& floydWarshall) {
    QMap<QPair<quint32, quint32>, quint32> result;

    QVector<quint32> nodes = floydWarshall.nodeList();
    for (quint32 src : nodes) {
        for (quint32 tgt : nodes) {
            if (src == tgt) continue;
            QVector<quint32> p = floydWarshall.path(src, tgt);
            if (!p.isEmpty() && p.size() > 1) {
                result.insert(qMakePair(src, tgt), static_cast<quint32>(p.size() - 1));
            }
        }
    }

    return result;
}

/// 以每条有向边跳数 = 1 为权重，用 Floyd-Warshall 计算所有节点对最短跳数矩阵
static QMap<QPair<quint32, quint32>, quint32> buildHopMatrix(const QVector<QSharedPointer<NetworkEdge>>& edges) {
    QMap<QPair<quint32, quint32>, quint32> result;
    
    // 构建图，每条边权重 = 1
    Graph g;
    for (const QSharedPointer<NetworkEdge>& e : edges) {
        QString err;
        g.addEdge(e->source, e->target, 1.0, e->bandwidth, &err);
    }
    
    // 执行 Floyd-Warshall
    FloydWarshallResult fw;
    QString err;
    if (!floydWarshall(g, fw, &err)) {
        return result;
    }
    
    // 转换为跳数矩阵
    auto allDist = fw.allDistances();
    for (const auto& [src, tgt, dist] : allDist) {
        if (qIsFinite(dist) && dist > 0.0) {
            result.insert(qMakePair(src, tgt), static_cast<quint32>(qRound(dist)));
        }
    }
    
    return result;
}

/// 构建以 `weightFn` 提取边权重的图
static void buildGraph(Graph& g,
                        const QVector<QSharedPointer<NetworkEdge>>& edges,
                        std::function<double(const QSharedPointer<NetworkEdge>&)> weightFn) {
    g.clear();
    for (const QSharedPointer<NetworkEdge>& e : edges)
    {
        double w = weightFn(e);
        if (w >= 0.0)
        {
            QString err;
            g.addEdge(e->source, e->target, w, e->bandwidth, &err);
        }
    }
}

/// 构建以延迟为权重的 Floyd 结果
static std::optional<FloydWarshallResult> buildDelayFw(const QVector<QSharedPointer<NetworkEdge>>& edges) {
    Graph g;
    buildGraph(g, edges, [](const QSharedPointer<NetworkEdge>& e) { return e->weight; });
    FloydWarshallResult fw;
    QString err;
    if (floydWarshall(g, fw, &err)) {
        return fw;
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// 步骤 1：读取文件
// ---------------------------------------------------------------------------

static bool step1LoadEdges(const QString& filePath, QVector<QSharedPointer<NetworkEdge>>& edges, QString& errorMsg) {
    auto [success, rawEdges] = read_netdata::readGraph(filePath, &errorMsg);
    if (!success) {
        errorMsg = QString("读取图文件失败 [%1]: %2").arg(filePath).arg(errorMsg);
        return false;
    }
    if (rawEdges.isEmpty()) {
        errorMsg = QString("图文件为空或无有效边: %1").arg(filePath);
        return false;
    }
    
    // Edge::weight = delay(ms)，Edge::bandWidth = bandwidth(Gbps)
    for (const Edge& e : rawEdges) {
        edges.append(QSharedPointer<NetworkEdge>::create(e.source, e.target, e.bandWidth, e.weight));
    }
    return true;
}

// ---------------------------------------------------------------------------
// 步骤 2：收集顶点集 V，随机选取边缘服务器 Vc
// ---------------------------------------------------------------------------
static QPair<QVector<quint32>, QVector<quint32>> step2CollectVerticesAndServers(
    const QVector<QSharedPointer<NetworkEdge>>& edges, int num) {
    
    QSet<quint32> nodeSet;
    for (const QSharedPointer<NetworkEdge>& e : edges) {
        nodeSet.insert(e->source);
        nodeSet.insert(e->target);
    }
    
    QVector<quint32> v;
    for (quint32 node : nodeSet) {
        v.append(node);
    }
    std::sort(v.begin(), v.end());
    
    int pick = qMin(num, static_cast<int>(v.size()));
    QVector<quint32> shuffled = v;
    
    // Fisher-Yates shuffle
    QRandomGenerator* rng = QRandomGenerator::global();
    for (int i = shuffled.size() - 1; i > 0; --i) {
        int j = rng->bounded(0, i + 1);
        qSwap(shuffled[i], shuffled[j]);
    }
    
    QVector<quint32> vc;
    for (int i = 0; i < pick; ++i) {
        vc.append(shuffled[i]);
    }
    
    return qMakePair(v, vc);
}

// ---------------------------------------------------------------------------
// 步骤 3 辅助：l_value 计算
// ---------------------------------------------------------------------------
static double lValue(quint32 h, double alphaMin, double alphaMax, double beta, int eAbs) {
    double hf = static_cast<double>(h);
    if (hf < alphaMin) {
        return 1.0;
    } else if (hf > alphaMax) {
        return static_cast<double>(eAbs);
    } else {
        return beta;
    }
}

// ---------------------------------------------------------------------------
// 步骤 3 辅助：判断链路是否在最短路径上
// ---------------------------------------------------------------------------
static bool isOnShortestPath(const NetworkEdge& e, quint32 ci, quint32 cj,
                             const FloydWarshallResult& delayFw) {
    double dCiCj = delayFw.distance(ci, cj);
    if (qIsInf(dCiCj)) return false;
    
    // 正向检查：ci -> e.source -> e.target -> cj
    double dCiSrc = delayFw.distance(ci, e.source);
    double dTgtCj = delayFw.distance(e.target, cj);
    if (qIsFinite(dCiSrc) && qIsFinite(dTgtCj) &&
        qAbs(dCiSrc + e.weight + dTgtCj - dCiCj) < 1e-9) {
        return true;
    }
    
    // 反向检查（无向图）：ci -> e.target -> e.source -> cj
    double dCiTgt = delayFw.distance(ci, e.target);
    double dSrcCj = delayFw.distance(e.source, cj);
    if (qIsFinite(dCiTgt) && qIsFinite(dSrcCj) &&
        qAbs(dCiTgt + e.weight + dSrcCj - dCiCj) < 1e-9) {
        return true;
    }
    
    return false;
}

// ---------------------------------------------------------------------------
// 步骤 3 辅助：找到 mcn
// ---------------------------------------------------------------------------
static quint32 findMcn(const NetworkEdge& e, quint32 ci, quint32 cj,
                       const FloydWarshallResult& delayFw,
                       const QMap<QPair<quint32, quint32>, quint32>& hopMap) {
    QVector<quint32> path = delayFw.path(ci, cj);
    if (path.isEmpty()) {
        return ci;
    }
    
    quint32 bestNode = ci;
    quint32 bestHops = std::numeric_limits<quint32>::max();
    
    for (quint32 node : path) {
        quint32 h = hopMap.value(qMakePair(node, e.target), std::numeric_limits<quint32>::max());
        if (h < bestHops) {
            bestHops = h;
            bestNode = node;
        }
    }
    
    return bestNode;
}

// ---------------------------------------------------------------------------
// 步骤 3 辅助：计算 YG_e_ci_cj
// ---------------------------------------------------------------------------

static double computeYG(const NetworkEdge& e, quint32 ci, quint32 cj,
                        const FloydWarshallResult& delayFw,
                        const QMap<QPair<quint32, quint32>, quint32>& hopMap,
                        double mina, double maxw, int eAbs,
                        double alphaMin, double alphaMax, double beta) {
    // IG_e_ci_cj：e 是否在 G 上 Ci→Cj 最短路径（按延迟）上
    if (isOnShortestPath(e, ci, cj, delayFw)) {
        return mina;
    }
    
    // IGp_e_ci_cj：lG_ci_cj == E_abs（即 h > alpha_max）
    quint32 hCiCj = hopMap.value(qMakePair(ci, cj), std::numeric_limits<quint32>::max());
    double lVal = lValue(hCiCj, alphaMin, alphaMax, beta, eAbs);
    if (qAbs(lVal - static_cast<double>(eAbs)) < 1e-9) {
        return maxw * qPow(static_cast<double>(eAbs), 2);
    }
    
    // 否则：dGh / dGv
    quint32 mcn = findMcn(e, ci, cj, delayFw, hopMap);
    double dgV = static_cast<double>(hopMap.value(qMakePair(e.source, mcn), 0));
    double hCiMcn = static_cast<double>(hopMap.value(qMakePair(ci, mcn), std::numeric_limits<quint32>::max()));
    double hCjMcn = static_cast<double>(hopMap.value(qMakePair(cj, mcn), std::numeric_limits<quint32>::max()));
    double dgH = qMin(hCiMcn, hCjMcn);
    
    if (dgV == 0.0) {
        return mina;
    }
    return dgH / dgV;
}

// ---------------------------------------------------------------------------
// 步骤 3 辅助：计算单条链路的 hs_e_G
// ---------------------------------------------------------------------------
static double computeHsForEdge(const NetworkEdge& e, const QVector<quint32>& vc,
                               const FloydWarshallResult& delayFw,
                               const QMap<QPair<quint32, quint32>, quint32>& hopMap,
                               double mina, double maxw, int eAbs,
                               double alphaMin, double alphaMax, double beta) {
    double total = 0.0;
    
    for (int i = 0; i < vc.size(); ++i) {
        for (int j = 0; j < vc.size(); ++j) {
            if (i == j) continue;
            quint32 ci = vc[i];
            quint32 cj = vc[j];
            total += computeYG(e, ci, cj, delayFw, hopMap, mina, maxw, eAbs, alphaMin, alphaMax, beta);
        }
    }
    
    return total;
}

// ---------------------------------------------------------------------------
// 步骤 3：计算每条链路的 hs_e_G 和 Wep
// ---------------------------------------------------------------------------
static void step3ComputeWep(QVector<QSharedPointer<NetworkEdge>>& edges, const QVector<quint32>& vc,
                            const FloydWarshallResult& delayFw,
                            const QMap<QPair<quint32, quint32>, quint32>& hopMap,
                            double mina, double maxw, int eAbs,
                            double alphaMin, double alphaMax, double beta) {
    for (const QSharedPointer<NetworkEdge>& e : edges) {
        e->hsEG = computeHsForEdge(*e, vc, delayFw, hopMap, mina, maxw, eAbs, alphaMin, alphaMax, beta);
        e->weightedCost = e->weight * e->hsEG;
    }
}

// ---------------------------------------------------------------------------
// 步骤 4：构建初始 Gc
// ---------------------------------------------------------------------------
static void step4BuildInitialGc(EecnBuild& ctx) {
    // 构建以 Wep 为权重的图
    buildGraph(ctx.gc, ctx.eSet, [](const QSharedPointer<NetworkEdge>& e) {
        return qMax(e->weightedCost, 0.0);
    });
    
    FloydWarshallResult wepFw;
    QString err;
    if (!floydWarshall(ctx.gc, wepFw, &err)) {
        ctx.gcInitial.clear();
        return;
    }
    
    // 收集所有服务器对最短路径上的边
    ctx.gcSet.clear();
    for (quint32 ci : ctx.vc) {
        for (quint32 cj : ctx.vc) {
            if (ci == cj) continue;
            
            QVector<quint32> path = wepFw.path(ci, cj);
            for (int i = 0; i < path.size() - 1; ++i) {
                ctx.gcSet.insert(qMakePair(path[i], path[i + 1]));
            }
        }
    }
    
    // 按 (source, target) 和 (target, source) 建立快速查找表（无向图双向映射）
    ctx.edgeMap.clear();
    for (const QSharedPointer<NetworkEdge>& e : ctx.eSet) {
        ctx.edgeMap.insert(qMakePair(e->source, e->target), e);
        ctx.edgeMap.insert(qMakePair(e->target, e->source), e);
    }
    
    ctx.gcInitial.clear();
    for (const auto& key : ctx.gcSet) {
        if (ctx.edgeMap.contains(key)) {
            ctx.gcInitial.append(ctx.edgeMap[key]);
        }
    }
}

// ---------------------------------------------------------------------------
// 步骤 5：检查跳数约束
// ---------------------------------------------------------------------------
static int countViolations(const QMap<QPair<quint32, quint32>, quint32>& gcHopMap,
                           const QMap<QPair<quint32, quint32>, quint32>& gHopMap,
                           const QVector<quint32>& vc,
                           double alphaMin, double alphaMax, double beta, int eAbs) {
    int violations = 0;
    
    for (quint32 ci : vc) {
        for (quint32 cj : vc) {
            if (ci == cj) continue;
            
            quint32 hG = gHopMap.value(qMakePair(ci, cj), std::numeric_limits<quint32>::max());
            quint32 hGc = gcHopMap.value(qMakePair(ci, cj), std::numeric_limits<quint32>::max());
            
            double limit = lValue(hG, alphaMin, alphaMax, beta, eAbs);
            quint32 threshold = static_cast<quint32>(qRound(limit * static_cast<double>(hG)));
            
            if (hGc > threshold) {
                ++violations;
            }
        }
    }
    
    return violations;
}

// ---------------------------------------------------------------------------
// 步骤 6：剪枝算法
// ---------------------------------------------------------------------------
static void tryRemoveEdges(QVector<QSharedPointer<NetworkEdge>>& gc, const QVector<QSharedPointer<NetworkEdge>>& candidates,
                           const QVector<quint32>& vc,
                           const QMap<QPair<quint32, quint32>, quint32>& gHopMap,
                           double alphaMin, double alphaMax, double beta, int eAbs,
                           int& currentViolations) {
    for (const QSharedPointer<NetworkEdge>& candidate : candidates) {
        // 尝试从 gc 中找到该边并移除
        int pos = -1;
        for (int i = 0; i < gc.size(); ++i) {
            if (gc[i]->source == candidate->source && gc[i]->target == candidate->target) {
                pos = i;
                break;
            }
        }
        
        if (pos >= 0) {
            QSharedPointer<NetworkEdge> removed = gc.takeAt(pos);
            QMap<QPair<quint32, quint32>, quint32> newHm = buildHopMatrix(gc);
            int newViolations = countViolations(newHm, gHopMap, vc, alphaMin, alphaMax, beta, eAbs);
            if (newViolations > currentViolations)
            {
                // 恢复
                gc.append(removed);
            }
        }
    }
}

static QVector<QSharedPointer<NetworkEdge>> step6Prune(const QVector<QSharedPointer<NetworkEdge>>& gcInitial,
                                       const QVector<QSharedPointer<NetworkEdge>>& allEdges,
                                       const QVector<quint32>& vc,
                                       const QMap<QPair<quint32, quint32>, quint32>& gHopMap,
                                       double alphaMin, double alphaMax, double beta, int eAbs) {
    // 初始 Gc = G 中所有链路
    QVector<QSharedPointer<NetworkEdge>> gc = allEdges;
    
    // A：G 中但不在初始 Gc 中的链路
    QSet<QPair<quint32, quint32>> gcInitialSet;
    for (const QSharedPointer<NetworkEdge>& e : gcInitial) {
        gcInitialSet.insert(qMakePair(e->source, e->target));
    }

    QVector<QSharedPointer<NetworkEdge>> a;
    for (const QSharedPointer<NetworkEdge>& e : allEdges) {
        if (!gcInitialSet.contains(qMakePair(e->source, e->target))) {
            a.append(e);
        }
    }
    
    // B：初始 Gc 中的链路
    QVector<QSharedPointer<NetworkEdge>> b = gcInitial;
    
    // 按 hs_e_G 降序排序
    auto sortByHsEG = [](const QSharedPointer<NetworkEdge>& x, const QSharedPointer<NetworkEdge>& y) {
        return y->hsEG < x->hsEG; // 降序
    };
    std::sort(a.begin(), a.end(), sortByHsEG);
    std::sort(b.begin(), b.end(), sortByHsEG);
    
    // 计算当前 Gc 的基线违约数
    QMap<QPair<quint32, quint32>, quint32> hm = buildHopMatrix(gc);
    int currentViolations = countViolations(hm, gHopMap, vc, alphaMin, alphaMax, beta, eAbs);
    
    // 处理 A
    tryRemoveEdges(gc, a, vc, gHopMap, alphaMin, alphaMax, beta, eAbs, currentViolations);
    // 处理 B
    tryRemoveEdges(gc, b, vc, gHopMap, alphaMin, alphaMax, beta, eAbs, currentViolations);
    
    return gc;
}

// ---------------------------------------------------------------------------
// 主算法：eecnGraphBuild
// ---------------------------------------------------------------------------
std::optional<EecnBuild> eecnGraphBuild(
    const QString& filePath,
    int edgeServerNum,
    double alphaMin,
    double alphaMax,
    double beta) {
    
    EecnBuild ctx(filePath, edgeServerNum, alphaMin, alphaMax, beta);
    
    // 步骤 1：读取文件
    QString err;
    if (!step1LoadEdges(filePath, ctx.eSet, err))
    {
        qDebug().noquote() << err;
        return std::nullopt;
    }
    ctx.eAbs = ctx.eSet.size();
    
    // 步骤 2：收集顶点、随机选取边缘服务器
    auto [v, vc] = step2CollectVerticesAndServers(ctx.eSet, edgeServerNum);
    ctx.v = v;
    ctx.vc = vc;
    
    // 步骤 2b：计算 maxw
    ctx.maxw = 0.0;
    for (const QSharedPointer<NetworkEdge>& e : ctx.eSet)
    {
        ctx.maxw = qMax(ctx.maxw, e->weight);
    }
    
    // 步骤 2c：构建以延迟为权重的原始图 G，并缓存 Floyd-Warshall 结果
    buildGraph(ctx.g, ctx.eSet, [](const QSharedPointer<NetworkEdge>& e) { return e->weight; });
    QString fwErr;
    if (!floydWarshall(ctx.g, ctx.floydWarshallRes, &fwErr))
    {
        qDebug() << "构建延迟矩阵失败：" << fwErr;
        return std::nullopt;
    }
    
    // 步骤 2c+：为每条边计算最短路径
    for (const QSharedPointer<NetworkEdge>& e : ctx.eSet) {
        e->path = ctx.floydWarshallRes.path(e->source, e->target);
    }

    // 步骤 2d：计算 G 上所有节点对的最短跳数矩阵
    ctx.gHopMap = buildHopMatrix(ctx.floydWarshallRes);
    
    // 步骤 3：计算每条链路的 hs_e_G 和加权成本 Wep（复用缓存的 FloydWarshall 结果）
    step3ComputeWep(ctx.eSet, ctx.vc, ctx.floydWarshallRes, ctx.gHopMap,
                    ctx.mina, ctx.maxw, ctx.eAbs, alphaMin, alphaMax, beta);
    
    // 步骤 4：以 Wep 为权重跑 Floyd，生成 gc 和初始 gcInitial
    step4BuildInitialGc(ctx);
    // 对ctx中的eSet按照hsEG降序排序
    std::sort(ctx.eSet.begin(), ctx.eSet.end(), [](const QSharedPointer<NetworkEdge>& a, const QSharedPointer<NetworkEdge>& b) {
        return a->hsEG > b->hsEG;
    });
    
    // 步骤 5：检查初始 Gc 是否满足跳数约束
    QMap<QPair<quint32, quint32>, quint32> gcHopMap = buildHopMatrix(ctx.gcInitial);
    ctx.hopViolations = countViolations(gcHopMap, ctx.gHopMap, ctx.vc,
                                         alphaMin, alphaMax, beta, ctx.eAbs);
    
    // 步骤 6：若满足约束则直接使用初始 Gc；否则进行剪枝
    QVector<QSharedPointer<NetworkEdge>> finalGc;
    if (ctx.hopViolations == 0) {
        finalGc = ctx.gcInitial;
    } else {
        finalGc = step6Prune(ctx.gcInitial, ctx.eSet, ctx.vc, ctx.gHopMap,
                             alphaMin, alphaMax, beta, ctx.eAbs);
    }
    
    // 计算结果
    double cost = 0.0;
    for (const QSharedPointer<NetworkEdge>& e : finalGc) {
        cost += e->weight;
    }
    
    EecnGraph graph;
    graph.servers = ctx.vc;
    graph.edges = finalGc;
    graph.costGc = cost;
    graph.eAbs = ctx.eAbs;
    ctx.result = graph;
    
    return ctx;
}

// ---------------------------------------------------------------------------
// 处理单个文件
// ---------------------------------------------------------------------------
static void networkAllocationPara(const QString& csvFile, QTextStream& logStream,
int edgeServerNum,  double alphaMin, double alphaMax, double beta) {
    // 输出计算参数
    teeWriteln(logStream, QString("Number of edge servers=%1, αmin=%2, αmax=%3, beta=%4")
                              .arg(edgeServerNum).arg(alphaMin).arg(alphaMax).arg(beta));

    std::optional<EecnBuild> ctxOpt = eecnGraphBuild(csvFile, edgeServerNum, alphaMin, alphaMax, beta);

    if (ctxOpt.has_value()) {
        const EecnBuild& ctx = ctxOpt.value();
        if (ctx.result.has_value()) {
            const EecnGraph& graph = ctx.result.value();
            teeWriteln(logStream, QString("EECN构建完成: 边缘服务器数=%1, 链路数=%2, cost_Gc=%3")
                                      .arg(graph.servers.size())
                                      .arg(graph.edges.size())
                                      .arg(graph.costGc, 0, 'f', 6));
        }
    } else {
        teeWriteln(logStream, "EECN构建失败");
    }
}

static void networkAllocationFile(const QString& csvFile) {
    QFileInfo inputInfo(csvFile);
    QString logStem = inputInfo.baseName();
    QString logFilename = QString("%1_allocation_analysis.log").arg(logStem);
    QString logPath = inputInfo.absoluteDir().filePath(logFilename);
    
    QFile logFile(logPath);
    if (!logFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug().noquote() << QString("创建日志文件失败: %1").arg(logPath);
        return;
    }
    
    QTextStream logStream(&logFile);
    logStream.setEncoding(QStringConverter::Utf8);
    
    teeWriteln(logStream, QString("\n========== 处理文件: %1 ==========").arg(csvFile));
    networkAllocationPara(csvFile, logStream, 20,  1.0, 7.0, 1.2);
    networkAllocationPara(csvFile, logStream, 20,  1.0, 5.0, 1.2);
    networkAllocationPara(csvFile, logStream, 20,  1.0, 5.0, 1.4);
    networkAllocationPara(csvFile, logStream, 20,  1.0, 5.0, 1.3);

    //networkAllocationPara(csvFile, logStream, 40,  1.0, 7.0, 1.2);
    //networkAllocationPara(csvFile, logStream, 40,  1.0, 5.0, 1.2);
    //networkAllocationPara(csvFile, logStream, 40,  1.0, 5.0, 1.4);
    //networkAllocationPara(csvFile, logStream, 40,  1.0, 5.0, 1.3);

    //networkAllocationPara(csvFile, logStream, 60,  1.0, 7.0, 1.2);
    //networkAllocationPara(csvFile, logStream, 60,  1.0, 5.0, 1.2);
    //networkAllocationPara(csvFile, logStream, 60,  1.0, 5.0, 1.4);
    //networkAllocationPara(csvFile, logStream, 60,  1.0, 5.0, 1.3);

    //networkAllocationPara(csvFile, logStream, 80,  1.0, 7.0, 1.2);
    //networkAllocationPara(csvFile, logStream, 80,  1.0, 5.0, 1.2);
    //networkAllocationPara(csvFile, logStream, 80,  1.0, 5.0, 1.4);
    //networkAllocationPara(csvFile, logStream, 80,  1.0, 5.0, 1.3);

    logFile.close();
}

void networkAllocation() {
    QStringList csvFiles = {
        "D:\\张新常\\网络试验学习\\20260103\\网络拓扑\\Waxman-1000-1.txt",
//        "D:\\张新常\\网络试验学习\\20260103\\网络拓扑\\Waxman-2000-1.txt",
//        "D:\\张新常\\网络试验学习\\20260103\\网络拓扑\\Waxman-3000-1.txt",
//        "D:\\张新常\\网络试验学习\\20260103\\网络拓扑\\Waxman-4000-1.txt",
    };
    
    for (const QString& csvFile : csvFiles) {
        networkAllocationFile(csvFile);
    }
}

} // namespace network_resource_allocation
