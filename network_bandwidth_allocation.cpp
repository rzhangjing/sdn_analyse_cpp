#include "network_bandwidth_allocation.h"
#include <QRandomGenerator>
#include <QDebug>
#include <QtMath>
#include <algorithm>
#include <cmath>

namespace network_bandwidth_allocation {

// 常量定义
constexpr int ARRAY_SIZE = 2000;
constexpr int EXPERIMENT_START = 99;
constexpr int EXPERIMENT_END = 1000;
constexpr int EXPERIMENT_COUNT = EXPERIMENT_END - EXPERIMENT_START; // 901

/// 初始化数组A
/// A[0]←0到20之间的随机数，然后A[i] = A[i-1] + max{0, -q与q之间的随机数}
static void initializeArrayA(QVector<double>& arrayA, double q) {
    QRandomGenerator* rng = QRandomGenerator::global();
    
    // A[0] 为 0 到 20 之间的随机数
    arrayA[0] = rng->generateDouble() * 20.0;
    
    // 填充 A[1] 到 A[999]
    for (int i = 1; i < 1000; ++i) {
        // -q 到 q 之间的随机数
        double randomStep = rng->generateDouble() * 2.0 * q - q;
        // max{0, 随机数}
        double step = qMax(0.0, randomStep);
        arrayA[i] = arrayA[i - 1] + step;
    }
}

/// 计算成功率
/// t从99到999，如果array[t] >= array_a[t]，s增加1
/// 返回 s/901 保留两位小数
static double calculateSuccessRate(const QVector<double>& array, const QVector<double>& arrayA) {
    int s = 0;
    for (int t = EXPERIMENT_START; t < EXPERIMENT_END; ++t) {
        if (array[t] >= arrayA[t]) {
            ++s;
        }
    }
    // s/901 保留两位小数
    double rate = static_cast<double>(s) / EXPERIMENT_COUNT;
    return static_cast<double>(qRound(rate * 100.0)) / 100.0;
}

/// 处理数组B并计算成功率
static double processArrayB(QVector<double>& arrayB, const QVector<double>& arrayA, int c) {
    // 阶段1: i从999到c，B[i] = A[i-c]
    for (int i = 999; i >= c; --i) {
        arrayB[i] = arrayA[i - c];
    }
    
    // 复制B到B'
    QVector<double> arrayBPrime = arrayB;
    
    // 阶段2: j从c到c+10，B[j] = B[j] + 5
    int endJ1 = qMin(c + 10, ARRAY_SIZE);
    for (int j = c; j < endJ1; ++j) {
        arrayB[j] += 5.0;
    }
    
    // 阶段3: j从c+10到999，B[j] = B[j] + 5 + X
    // X是j之前10个元素的最大增长值，增长值 = |B'[k] - B'[k-1]|
    int startJ2 = c + 10;
    for (int j = startJ2; j < 1000; ++j) {
        // 计算j之前10个元素的最大增长值
        double maxGrowth = 0.0;
        for (int k = j - 10; k < j; ++k) {
            if (k > 0 && k < ARRAY_SIZE) {
                double growth = qAbs(arrayBPrime[k] - arrayBPrime[k - 1]);
                if (growth > maxGrowth) {
                    maxGrowth = growth;
                }
            }
        }
        arrayB[j] = arrayB[j] + 5.0 + maxGrowth;
    }
    
    // 计算成功率: t从99到999，如果B[t] >= A[t]，s增加1
    return calculateSuccessRate(arrayB, arrayA);
}

/// 处理数组C并计算成功率
static double processArrayC(QVector<double>& arrayC, const QVector<double>& arrayA, int a) {
    // 阶段1: i从999到a，C[i] = A[i-a]
    for (int i = 999; i >= a; --i) {
        arrayC[i] = arrayA[i - a];
    }
    
    // 复制C到C'（未使用但保持逻辑一致）
    Q_UNUSED(arrayC);
    QVector<double> arrayCPrime = arrayC;
    
    // 阶段2: j从a到999，C[j] = C[j] + 5
    for (int j = a; j < 1000; ++j) {
        arrayC[j] += 5.0;
    }
    
    // 计算成功率: t从99到999，如果C[t] >= A[t]，s增加1
    return calculateSuccessRate(arrayC, arrayA);
}

std::tuple<double, double, double> calculateStatistics(const QVector<double>& rates) {
    if (rates.isEmpty()) {
        return std::make_tuple(0.0, 0.0, 0.0);
    }
    
    double maxRate = -std::numeric_limits<double>::infinity();
    double minRate = std::numeric_limits<double>::infinity();
    double totalRate = 0.0;
    
    for (double rate : rates) {
        maxRate = qMax(maxRate, rate);
        minRate = qMin(minRate, rate);
        totalRate += rate;
    }
    
    double avgRate = totalRate / rates.size();
    
    // 保留两位小数
    double maxRounded = static_cast<double>(qRound(maxRate * 100.0)) / 100.0;
    double avgRounded = static_cast<double>(qRound(avgRate * 100.0)) / 100.0;
    double minRounded = static_cast<double>(qRound(minRate * 100.0)) / 100.0;
    
    return std::make_tuple(maxRounded, avgRounded, minRounded);
}

ExperimentResult networkBandwidthAllocationCapability(int a, int c, double q) {
    // 1. 初始化3个数组
    QVector<double> arrayA(ARRAY_SIZE, 0.0);
    QVector<double> arrayB(ARRAY_SIZE, 0.0);
    QVector<double> arrayC(ARRAY_SIZE, 0.0);
    
    // 2. 对数组A赋值
    initializeArrayA(arrayA, q);
    
    // 3. 对数组B赋值并计算成功率
    double successRateB = processArrayB(arrayB, arrayA, c);
    
    // 4. 对数组C赋值并计算成功率
    double successRateC = processArrayC(arrayC, arrayA, a);
    
    return ExperimentResult(1, successRateB, successRateC);
}

void networkBandwidthAllocationCapabilityWork() {
    // 定义参数
    int a = 3;  // 数组C的偏移量
    int c = 2;  // 数组B的偏移量
    QVector<double> qValues = {2.0, 4.0, 6.0, 8.0}; // 不同的q值
    int runCount = 50; // 每个q值运行50次
    
    qDebug().noquote() << "\n========== 网络带宽分配能力测试 ==========";
    qDebug().noquote() << QString("参数: a=%1, c=%2").arg(a).arg(c);
    qDebug().noquote() << QString("每个q值运行%1 次\n").arg(runCount);
    
    for (double q : qValues) {
        qDebug().noquote() << QString("\n----- q = %1 -----").arg(q);
        
        // 存储所有实验的成功率
        QVector<double> bSuccessRates;
        QVector<double> cSuccessRates;
        bSuccessRates.reserve(runCount);
        cSuccessRates.reserve(runCount);
        
        // 运行50次实验
        for (int experimentId = 1; experimentId <= runCount; ++experimentId) {
            ExperimentResult result = networkBandwidthAllocationCapability(a, c, q);
            bSuccessRates.append(result.successRateB);
            cSuccessRates.append(result.successRateC);
            
            // 输出每次实验结果
            qDebug().noquote() << QString("实验 %1: B成功率=%2, C成功率=%3")
                .arg(experimentId, 2)
                .arg(result.successRateB, 0, 'f', 2)
                .arg(result.successRateC, 0, 'f', 2);
        }
        
        // 计算B数组的统计数据
        auto [bMax, bAvg, bMin] = calculateStatistics(bSuccessRates);
        
        // 计算C数组的统计数据
        auto [cMax, cAvg, cMin] = calculateStatistics(cSuccessRates);
        
        // 输出统计结果
        qDebug().noquote() << QString("\n--- q = %1 统计结果 ---").arg(q);
        qDebug().noquote() << QString("B数组: 最大成功率=%1, 平均成功率=%2, 最小成功率=%3")
            .arg(bMax, 0, 'f', 2).arg(bAvg, 0, 'f', 2).arg(bMin, 0, 'f', 2);
        qDebug().noquote() << QString("C数组: 最大成功率=%1, 平均成功率=%2, 最小成功率=%3")
            .arg(cMax, 0, 'f', 2).arg(cAvg, 0, 'f', 2).arg(cMin, 0, 'f', 2);
    }
    
    qDebug().noquote() << "\n========== 测试完成 ==========";
}

} // namespace network_bandwidth_allocation
