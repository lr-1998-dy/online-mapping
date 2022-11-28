/*
 * @Author: Li Rui
 * @LastEditTime: 2022-09-07 09:21:20
 * @LastEditors: Li Rui
 * @Description: 
 */

#include <sys/types.h>
#include <dirent.h>
#include <stdio.h>
#include <errno.h>
#include <lidar_localization/tools/get_filenames.hpp>

namespace lidar_localization {
    FileGetter::FileGetter(){
    }

    int FileGetter::ScanFiles(std::string inputDirectory){
        inputDirectory_=inputDirectory;
        inputDirectory_ = inputDirectory_.append("/");

        DIR *p_dir;
        const char* str = inputDirectory_.c_str();

        p_dir = opendir(str);   
        if( p_dir == NULL)
        {
            LOG(ERROR) << "没有找到 " << inputDirectory_ << " 文件夹";
        }

        struct dirent *p_dirent;

        while ( p_dirent = readdir(p_dir))
        {
            std::string tmpFileName = p_dirent->d_name;
            if( tmpFileName == "." || tmpFileName == "..")
            {
                continue;
            }
            else
            {
                file_names_.push_back(tmpFileName);
            }
        }
        closedir(p_dir);
        return file_names_.size();
    }

    void FileGetter::GetFileNames(std::vector<std::string>& file_names){
        file_names=file_names_;
    }

}
