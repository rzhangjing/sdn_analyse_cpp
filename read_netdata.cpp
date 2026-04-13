#include "read_netdata.h"
#include <QDebug>

namespace read_netdata {

std::optional<double> parseBandwidth(const QString& bandwidthStr) {
    QString trimmed = bandwidthStr.trimmed();
    
    // 尝试移除 Gbps 或 gbps 后缀
    if (trimmed.endsWith("Gbps", Qt::CaseInsensitive)) {
        QString numStr = trimmed.chopped(4).trimmed();
        bool ok = false;
        double value = numStr.toDouble(&ok);
        if (ok) {
            return value;
        }
    }
    return std::nullopt;
}

std::optional<double> parseDelay(const QString& delayStr) {
    QString trimmed = delayStr.trimmed();
    
    // 尝试移除 ms 或 MS 后缀
    if (trimmed.endsWith("ms", Qt::CaseInsensitive)) {
        QString numStr = trimmed.chopped(2).trimmed();
        bool ok = false;
        double value = numStr.toDouble(&ok);
        if (ok) {
            return value;
        }
    }
    return std::nullopt;
}

QPair<bool, QVector<Edge>> readGraph(const QString& filePath, QString* errorMsg) {
    QVector<Edge> links;
    
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (errorMsg) {
            *errorMsg = QString("无法打开文件: %1").arg(filePath);
        }
        return qMakePair(false, links);
    }
    
    QTextStream in(&file);
    in.setEncoding(QStringConverter::Utf8);
    
    bool isFirstDataRow = true;
    
    while (!in.atEnd()) {
        QString line = in.readLine();
        
        // 跳过空行
        if (line.trimmed().isEmpty()) {
            continue;
        }
        
        // 跳过注释行（以 # 开头）
        if (line.trimmed().startsWith('#')) {
            continue;
        }
        
        // 按 Tab 分割
        QStringList fields = line.split('\t');
        
        // 确保至少有 4 列数据
        if (fields.size() < 4) {
            continue;
        }
        
        // 跳过标题行
        if (isFirstDataRow) {
            if (fields[0].trimmed() == "source") {
                isFirstDataRow = false;
                continue;
            }
            isFirstDataRow = false;
        }
        
        // 解析各个字段
        bool ok = false;
        quint32 source = fields[0].trimmed().toUInt(&ok);
        if (!ok) {
            if (errorMsg) {
                *errorMsg = QString("无效的 source 字段: %1").arg(fields[0]);
            }
            return qMakePair(false, links);
        }
                
        quint32 target = fields[1].trimmed().toUInt(&ok);
        if (!ok) {
            if (errorMsg) {
                *errorMsg = QString("无效的 target 字段: %1").arg(fields[1]);
            }
            return qMakePair(false, links);
        }
                
        auto bandwidthOpt = parseBandwidth(fields[2]);
        if (!bandwidthOpt.has_value()) {
            if (errorMsg) {
                *errorMsg = QString("无法解析带宽: %1").arg(fields[2]);
            }
            return qMakePair(false, links);
        }
                
        auto delayOpt = parseDelay(fields[3]);
        if (!delayOpt.has_value()) {
            if (errorMsg) {
                *errorMsg = QString("无法解析延迟: %1").arg(fields[3]);
            }
            return qMakePair(false, links);
        }
        
        // Edge: source, target, weight (delay), bandWidth
        links.append(Edge(source, target, delayOpt.value(), bandwidthOpt.value()));
    }
    
    file.close();
    return qMakePair(true, links);
}

} // namespace read_netdata
