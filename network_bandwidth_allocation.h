#pragma once

#include <QVector>
#include <QString>
#include <tuple>

/// 实验结果
struct ExperimentResult {
    int experimentId;
    double successRateB;  // B数组的成功率
    double successRateC;  // C数组的成功率
    
    ExperimentResult() : experimentId(1), successRateB(0.0), successRateC(0.0) {}
    ExperimentResult(int id, double rateB, double rateC) 
        : experimentId(id), successRateB(rateB), successRateC(rateC) {}
};

namespace network_bandwidth_allocation {

/// 主入口：网络带宽分配能力测试
//a为数组C的偏移量  c为数组B的偏移量
void networkBandwidthAllocationCapabilityWork(QString csvFile, int k, int a, int c);

/// 网络带宽分配能力测试
/// 输入: a - 数组C的偏移量, c - 数组B的偏移量, q - A数组增长步长参数
ExperimentResult networkBandwidthAllocationCapability(int a, int c, double q);

/// 计算统计数据：最大值、平均值、最小值
std::tuple<double, double, double> calculateStatistics(const QVector<double>& rates);

} // namespace network_bandwidth_allocation
