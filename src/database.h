#ifndef DATABASE_H
#define DATABASE_H

#include <vector>
#include <iostream>
#include <map>
#include "data.h"
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>
#include <QDirIterator>

class database
{
public:

    database(QString path_database_param);

    QString path_database;
    std::vector<data> vector_data;
    std::map<std::string, int> info_data;


};

#endif // DATABASE_H
