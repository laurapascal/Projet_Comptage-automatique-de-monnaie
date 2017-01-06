#ifndef DATABASE_H
#define DATABASE_H

#include <vector>
#include <iostream>
#include <map>
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>
#include <QDirIterator>

class database
{
public:

    database(QString path_database_param);

    QString path_database;
    std::map<QString,std::string> map_data;


};

#endif // DATABASE_H
