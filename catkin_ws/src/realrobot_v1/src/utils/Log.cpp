#include "Log.h"
namespace stcontroller{
Log::Log(std::vector<std::string> files, std::string directory){
    //0) get current time
    struct timeval tv;
    struct timezone tz;
    struct tm *tm;
    string month="0";
    string day="0";
    string  hour="0";
    string minute="0";
    string second="0";
    gettimeofday(&tv,&tz);
    std::ostringstream ostr;
    tm=localtime(&tv.tv_sec);
    if(tm->tm_mon+1<10)
        ostr<<month<<tm->tm_mon + 1 ;
    else
        ostr << tm->tm_mon +1;

    if(tm->tm_mday<10)
        ostr<<day<<tm->tm_mday ;
    else
        ostr << tm->tm_mday;

    if(tm->tm_hour<10)
        ostr<<hour<<tm->tm_hour;
    else
        ostr << tm->tm_hour;

    if(tm->tm_min<10)
        ostr<<minute<<tm->tm_min;
    else
        ostr << tm->tm_min;

    if(tm->tm_sec<10)
        ostr<<second<<tm->tm_sec;
    else
        ostr << tm->tm_sec;
//
//    ostr << tm->tm_mday;
//    ostr << tm->tm_hour;
//    ostr << tm->tm_min;
//    ostr << tm->tm_sec;
    std::string folder = ostr.str();
    std::string folder_path;
    std::string parent_path = std::string(getenv("HOME")) + "/workspace/experiment_data/";

    //1) init file path
    if(directory=="none"){
        folder_path = parent_path + folder;
    }else{
        folder_path = directory + folder;
    }
    //2) Creating a directory 
    if (mkdir(folder_path.c_str(), 0777) == -1) 
        cerr << "Error :  " << strerror(errno) << endl; 
    char mode[] = "w+";
    file_buffer_size = 40;

    files_num=files.size();
    filesOperator.resize(files_num);

    std::string file_path;
    for(int8_t idx=0;idx<files_num;idx++){
        file_path=folder_path +"/"+ files.at(idx) + ".csv";
        filesOperator.at(idx)=new FileOperator(file_path,mode,file_buffer_size);
        files_index.insert(pair<std::string, int>(files.at(idx),idx));  
    }
    //time counter, indicates the data row number
    t=0;
    //3) Information
    printf("##---------INFO---------##\n");
    printf("The date and time is :\n");
    printf("%d:%d:%d:%d:%02d:%02d %ld \n", tm->tm_year+1900,tm->tm_mon+1,tm->tm_mday, tm->tm_hour,tm->tm_min,tm->tm_sec, tv.tv_usec);
    printf("The data are stored at:\n");
    std::cout<<folder_path<<std::endl;
    printf("--------------------------\n");

    //4) store log which has the path of the data
    // 以写模式打开文件
    ofstream outfile;
    outfile.open(parent_path+"datapath.log",ios::in |ios::trunc);
    // 向文件写入用户输入的数据
    outfile << folder << endl;
    // 关闭打开的文件
    outfile.close();
}
Log::~Log(){
    for(uint8_t idx=0;idx<filesOperator.size();idx++)
        delete filesOperator.at(idx);
}

void  Log::saveData(string file_name, const vector<float>& data,  const vector<string>& data_names){
    //1) check whether file_name and its number is right 
    map<string,int>::iterator files_itr;
    files_itr=files_index.find(file_name);
    uint8_t num=files_itr->second;
    assert(num<files_num);

    //2) add data names (columns)
    if(t==0){
        std::string columns = string("Time");
        for(uint8_t idx=0;idx<data.size();idx++)
            columns +="\t" + data_names.at(idx);
        filesOperator[num]->AddData(0,columns);
    }else{
        //3) load data into a strings
        std::string a_row_data = to_string(t);
        for(uint8_t idx=0;idx<data.size();idx++){
            a_row_data +="\t" + std::to_string(data.at(idx));
        }
        //tempory store data
        filesOperator[num]->AddData(t%file_buffer_size,a_row_data);
    }

    //save data into file
    if(t%file_buffer_size==(file_buffer_size-1)){//save data into file
            filesOperator[num]->SaveToFile();
    }

}

void  Log::saveData(string file_name, const vector<pair<string, float>>& data){
    //1) check whether file_name and its number is right 
    map<string,int>::iterator files_itr;
    files_itr=files_index.find(file_name);
    uint8_t num = files_itr->second;
    assert(num < files_num);
    //2) add data names (columns)
    if(t == 0){
        std::string columns = string("Time");
        pair<string, float> iter;
        //pair<string, float>::const_iterator iter;
        for (int i=0; i < data.size(); i++)
        {
            iter = data.at(i);
            columns += "\t" + iter.first;
        }
        
        // for (int i = 0; i < data.size(); i++)
        // {
        //     iter = data.at(i).begin();
        //     columns +="\t" + iter->first;
        // }
        // for(map<string, float>::const_iterator iter=data.begin(); iter!=data.end(); iter++){
        //     columns +="\t" + iter->first;
        // }
        filesOperator[num]->AddData(0,columns);
    }else{
        //3) load data into a strings
        std::string a_row_data = to_string(t);

        pair<string, float> iter;
        //pair<string, float>::const_iterator iter;
        for (int i=0; i < data.size(); i++)
        {
            iter = data.at(i);
            a_row_data += "\t" + to_string(iter.second);
        }

        // map<string, float>::const_iterator iter;
        // for (int i = 0; i < data.size(); i++)
        // {
        //     iter = data.at(i).begin();
        //     a_row_data +="\t" + std::to_string(iter->second);
        // }
        // for(map<string, float>::const_iterator iter=data.begin(); iter!=data.end(); iter++){
        //     a_row_data +="\t" + std::to_string(iter->second);
        // }
        //tempory store data
        filesOperator[num]->AddData(t%file_buffer_size,a_row_data);
    }

    //save data into file
    if(t%file_buffer_size==(file_buffer_size-1)){//save data into file
        filesOperator[num]->SaveToFile();
    }

}




void Log::step(){
    t++;
    if(t>100000000)
        t=1;
}
}
