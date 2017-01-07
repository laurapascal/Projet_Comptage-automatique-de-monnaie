#include "database.h"

database::database(QString path_database_param)
:path_database(path_database_param)
{
    // Parcours de tous les sous-repertoires contenue dans le repertoire path_database
    QDirIterator dirIterator(path_database, QDir::NoDotAndDotDot | QDir::Dirs | QDir::NoSymLinks );
    while(dirIterator.hasNext())
    {
        dirIterator.next();
        QString path_ss_repertory = dirIterator.filePath(); // path de mon sous-repertoire
        QFileInfo fileinfo (dirIterator.filePath()); // permet plus tard de récuperer le nom du sous-repertoire

        // Parcours de tous les fichiers contenue dans les sous-repertoires
        QDir Dir_ss_repertory(path_ss_repertory);
        QFileInfoList fileList;
        fileList.append(Dir_ss_repertory.entryInfoList());
        for( int i = 2; i < fileList.size(); i++)
        {
            // Remplissage de notre map
            QString data_path = fileList[i].absoluteFilePath(); // path du fichier
            std::string data_value = fileinfo.baseName().toStdString();
            map_data[data_path] = data_value;
        }
    }

}

