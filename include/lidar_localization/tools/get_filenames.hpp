/*
 * @Author: Li Rui
 * @LastEditTime: 2022-09-07 09:21:05
 * @LastEditors: Li Rui
 * @Description:读取文件夹下面的所有的文件 
 */
#ifndef LIDAR_LOCALIZATION_TOOLS_GET_FILENAMES_HPP_
#define LIDAR_LOCALIZATION_TOOLS_GET_FILENAMES_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include<vector>
#include "glog/logging.h"
#include <lidar_localization/tools/get_filenames.hpp>

namespace lidar_localization {
class FileGetter{
  public:
    FileGetter();
    int ScanFiles(std::string inputDirectory);
    void GetFileNames(std::vector<std::string>& file_names);

  private:
    std::string inputDirectory_;
    std::vector<std::string> file_names_;

};
}

#endif
