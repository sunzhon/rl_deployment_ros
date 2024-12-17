#ifndef _LOG_H_
#define _LOG_H_
#include "FileOperator.h"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h> 
#include <iostream> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <sstream>
namespace stcontroller{
class Log{
    public:
        Log(std::vector<std::string> files, std::string directory="none");
        ~Log();

    public:
        void saveData(string name, const vector<float>& data, const vector<string>& data_names);
        void saveData(string file_name, const vector<pair<string, float>>& data);
        void step();
    private:
        vector<FileOperator *> filesOperator;     
        uint8_t files_num;
        uint16_t file_buffer_size;
        uint16_t file_buffer_idx;
        map<std::string,int> files_index;

        long unsigned int t;   

};
}




#endif
