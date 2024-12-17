/*
 * FileOperator.cpp
 *
 *  Created on: 2018-3-8
 *      Author: suntao
 */

//#include <utils/FileOperator.h>
#include "FileOperator.h"
namespace stcontroller {

POpenFile *g_pOPenFile = NULL;
PCloseFile *g_pCloseFile = NULL;
PReadFile *g_pReadFile = NULL;
PGetFileSize *g_pGetFileSize = NULL;

FILE * g_OpenFile(const char *psFileName, const char *psMode) {
	if (g_pOPenFile == NULL) {
		FILE * pFile;//= NULL;
		//fopen_s(&pFile, psFileName, psMode);
		pFile=fopen(psFileName,psMode);
		if(pFile==NULL){
			perror("open file fial\n");
			return NULL;
		}else
			return pFile;
	} else {
		return (FILE *) (*g_pOPenFile)(psFileName, psMode);
	}
}

// 关闭文件
bool g_CloseFile(FILE *pFile) {
	if (g_pCloseFile == NULL) {
		return fclose(pFile) == 0;
	} else {
		return (*g_pCloseFile)(pFile);
	}
}
// 读取文件
size_t g_ReadFile(FILE *fp, void *buffer, size_t size) {
	if (g_pReadFile == NULL) {
		return (int) fread(buffer, 1, size, fp);
	} else {
		return (*g_pReadFile)(fp, buffer, size);
	}
}
// 获取文件长度
size_t g_GetFileSize(const char *psFileName) {
	if (g_pGetFileSize == NULL) {
		FILE * fp ;//= NULL;
		fp=fopen(psFileName,"rw");
		fseek(fp, 0, SEEK_END);
		long size = ftell(fp);
		fseek(fp, 0, SEEK_SET);
		fclose(fp);
		return size;
	} else {
		return (*g_pGetFileSize)(psFileName);
	}
}

FileOperator::FileOperator( const string & pathname, char * mode,unsigned int size){
	m_strFileName=pathname;
	pmode=mode;
	buffer_size=size;
	m_Data.resize(buffer_size);
	FP = g_OpenFile(m_strFileName.c_str(),pmode);
}
// 设置文件名
void FileOperator::SetFileName(const char * filename) {
	assert(filename != NULL);
	m_strFileName = filename;
}
// 获得文件名
const char* FileOperator::GetFileName() const {
	return m_strFileName.c_str();
}
// 测试是否加载成功
bool FileOperator::Loaded() const {
	return m_bLoad;
}
// 加载文件
bool FileOperator::LoadFromFile() {
	m_Data.clear();
	m_bLoad = false;
	FILE * fp = NULL;
	fp = g_OpenFile(m_strFileName.c_str(), "rb");
	if (NULL == fp) {
		return false;
	}
	size_t size = g_GetFileSize(m_strFileName.c_str());
	char* buffer = new char[size + 2];
	if (g_ReadFile(fp, buffer, size) != size) {
		g_CloseFile(fp);
		return false;
	}
	buffer[size] = '\r';
	buffer[size + 1] = '\n';
	g_CloseFile(fp);
	vector<const char*> lines;
	lines.reserve(256);
	size_t count = 0;
	const size_t size_1 = size + 2;
	for (size_t i = 0; i < size_1; ++i) {
		if ((buffer[i] == '\r') || (buffer[i] == '\n')) {
			buffer[i] = 0;
			count = 0;
		} else {
			if (count == 0) {
				lines.push_back(&buffer[i]);
			}
			++count;
		}
	}
	for (auto iter = lines.begin(); iter != lines.end(); ++iter) {
		m_Data.push_back(*iter);
	}
	m_bLoad = true;
	return true;
}
// 保存文件
bool FileOperator::SaveToFile()  {
	if(FP==NULL){
		FP=g_OpenFile(m_strFileName.c_str(),"w+");
			if (NULL == FP) {
				perror("open file fail");
				return false;
			}
	}
	string str;
	const size_t size = m_Data.size();
	for (size_t i = 0; i < size; ++i) {
		str = m_Data[i];
		str += "\n";
		fwrite(str.c_str(), sizeof(char), str.length(), FP);
	}
	return true;
}
// 加入一行数据
void FileOperator::AddData(unsigned int index,const string str) {
	assert(index<buffer_size);
	if ("" != str) {
		m_Data.at(index)=str;
	}
}
bool FileOperator::OpenFile(){
	//fopen_s(&fp, m_strFileName.c_str(), "wb");
	FP=g_OpenFile(m_strFileName.c_str(),"w+");
	if(FP==NULL){
		return false;
		perror("fail to open file");
	}
	else
	return true;
}

} /* namespace stcontroller */
