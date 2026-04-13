#pragma once

#include <QtGlobal>

struct Edge {
    quint32 source;
    quint32 target;
    double weight;
    double bandWidth;

    Edge() : source(0), target(0), weight(0.0), bandWidth(0.0) {}

    Edge(quint32 source, quint32 target, double weight, double bandWidth)
        : source(source), target(target), weight(weight), bandWidth(bandWidth) {}
};
