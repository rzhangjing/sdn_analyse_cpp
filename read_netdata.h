#pragma once

#include <QFile>
#include <QTextStream>
#include <QString>
#include <QStringList>
#include <QVector>
#include <optional>
#include "edge.h"

namespace read_netdata {

/// 从带宽字符串解析（如 "1.956Gbps" -> 1.956）
/// 支持大小写：Gbps 或 gbps
std::optional<double> parseBandwidth(const QString& bandwidthStr);

/// 从延迟字符串解析（如 "317.591ms" -> 317.591）
/// 支持大小写：ms 或 MS
std::optional<double> parseDelay(const QString& delayStr);

/// 从 CSV 文件读取网络图数据
/// 文件使用 Tab 分隔，# 开头为注释行
/// 数据列：source, target, bandwidth, delay
/// 
/// 返回: QPair<bool, QVector<Edge>>
///   - first: true 表示成功，false 表示失败
///   - second: 边列表
///   - errorMsg: 错误信息（可选）
QPair<bool, QVector<Edge>> readGraph(const QString& filePath, QString* errorMsg = nullptr);

} // namespace read_netdata
