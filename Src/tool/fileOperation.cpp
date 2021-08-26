#include "fileOperation.h"
#include <string>
#include <vector>

#ifdef _WIN32

#include <direct.h>
#include <io.h>

#else
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#endif

#define PATH_DELIMITER '/'

bool FileOperation::createDirectory(const std::string &folder) {
    std::string folderBuilder;
    std::string sub;
    sub.reserve(folder.size());
    for (auto it = folder.begin(); it != folder.end(); ++it) {
        const char c = *it;
        sub.push_back(c);
        if (c == PATH_DELIMITER || it == folder.end() - 1) {
            folderBuilder.append(sub);
#ifdef _WIN32
            if (0 != _access(folderBuilder.c_str(), 0)) {
                // this folder not exist
                if (0 != ::_mkdir(folderBuilder.c_str())) {
                    // create failed
                    return false;
                }
            }
#else
            if (0 != access(folderBuilder.c_str(), 0)) {
                // this folder not exist
                if (0 != mkdir(folderBuilder.c_str(), 0777)) {
                    // create failed
                    return false;
                }
            }
#endif
            sub.clear();
        }
    }
    return true;
}

void FileOperation::getImgFileInOrder(const std::string &path, uint32_t &number, const std::string &pictureFormat, std::vector<cv::Mat> &imgList, int readMode) {
    cv::Mat img;
    std::string filePath;
    while (true) {
        filePath = path + std::to_string(number) + "." + pictureFormat;
        img = cv::imread(filePath, readMode);
        if (!img.data) {
            break;
        }
        imgList.push_back(img);
        number++;
    }
}

void FileOperation::saveImg(const std::string &path, uint32_t &number, const cv::Mat &img, const std::string &pictureFormat) {
    if (!img.empty()) {
        cv::imwrite(path + std::to_string(number) + pictureFormat, img);
        number++;
    } else{
        LOG::error("Try to save empty image!");
    }
}

uint32_t FileOperation::getFileSizeInOrder(const std::string &path, const std::string &format) {
    std::string filename;
    uint32_t number = 0;
    while (true) {
        filename = path + std::to_string(number) + format;
        std::fstream _file;
        _file.open(filename, std::ios::in);
        if (!_file) {
            break;
        }
        number++;
    }
    return number;
}

void FileOperation::getAllFilesName(std::string path, std::vector<std::string> &filesList, uint8_t mode) {
#ifdef _WIN32
    _finddata_t file;
    long lf;
    //输入文件夹路径
    if ((lf = (long) _findfirst(path.c_str(), &file)) == -1) {
        std::cout << path << " not found!!!" << std::endl;
    } else {
        while (_findnext(lf, &file) == 0) {
            //输出文件名
            //cout<<file.name<<endl;
            if ((file.attrib & _A_SUBDIR)) {
                if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0) {
                    switch (mode) {
                        case 0:
                            filesList.push_back(file.name);
                            break;
                        case 1:
                            break;
                        case 2:
                            getAllFilesName(path + "/" + file.name, filesList, mode);
                            break;
                        default:
                            break;
                    }
                }
            } else {
                filesList.push_back(file.name);
            }
        }
    }
    _findclose(lf);
#else
    DIR* dir;
    struct dirent* pStResult = NULL;
    struct dirent* pStEntry = NULL;

    if ((dir = opendir(path.c_str())) == NULL) {
        std::cout << path << " not found!!!" << std::endl;
    }

    while (readdir_r(dir, pStEntry, &pStResult) && pStResult != NULL) {
        if (strcmp(pStEntry->d_name, ".") == 0 || strcmp(pStEntry->d_name, "..") == 0)    ///current dir OR parrent dir
            continue;
        else if (pStEntry->d_type == 8)    ///file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            filesList.push_back(pStEntry->d_name);
        else if (pStEntry->d_type == 10)    ///link file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if (pStEntry->d_type == 4) {   ///dir
            switch (mode) {
            case 0:
                filesList.push_back(pStEntry->d_name);
                break;
            case 1:
                break;
            case 2:
                getAllFilesName(path + "/" + pStEntry->d_name, filesList, mode);
                break;
            default:break;
            }
            filesList.push_back(pStEntry->d_name);
            /*
                memset(base,'\0',sizeof(base));
                strcpy(base,basePath);
                strcat(base,"/");
                strcat(base,ptr->d_nSame);
                readFileList(base);
            */
        }
    }
    closedir(dir);
#endif

    //排序，按从小到大排序
    //sort(filesList.begin(), filesList.end());
}
