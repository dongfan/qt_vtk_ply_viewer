#pragma once
#include <QString>
#include <QMap>

enum class AppMode {
    Pick,
    Inspection
};

struct HelpContent {
    QString title;
    QString description;
    QString recommended;
    QString caution;
};

using ModeHelpMap = QMap<AppMode, HelpContent>;
using HelpDB = QMap<QString, ModeHelpMap>;

class ParameterHelpDB {
public:
    static const HelpContent& get(const QString& key, AppMode mode);

private:
    static HelpDB db_;
    static HelpContent empty_;
};
