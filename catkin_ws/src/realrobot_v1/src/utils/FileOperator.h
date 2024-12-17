/*
 * fileoperator.h
 *
 *  Created on: 2018-3-8
 *      Author: suntao
 */

#ifndef FILEOPERATOR_H_
#define FILEOPERATOR_H_
#include <iostream>
#include <stdio.h>
#include <string>
#include <assert.h>
#include <vector>
using namespace std;

namespace stcontroller {

typedef void* (POpenFile)(const char *, const char *);
typedef bool (PCloseFile)(void*);
typedef size_t(PReadFile)(void*, void*, size_t);
typedef size_t(PGetFileSize)(const char *);
extern POpenFile *g_pOPenFile ;
extern PCloseFile *g_pCloseFile;
extern PReadFile *g_pReadFile ;
extern PGetFileSize *g_pGetFileSize;
FILE * g_OpenFile(const char *psFileName, const char *psMode);
bool g_CloseFile(FILE *pFile);
size_t g_ReadFile(FILE *fp, void *buffer, size_t size);
size_t g_GetFileSize(const char *psFileName);

// 文件操作类
class FileOperator {
public:
	FileOperator(const string & pathname,char * mode,unsigned int size);
	virtual ~FileOperator(){
		fclose(FP);
	};
	// 设置文件名
	void SetFileName(const char * filename);
	// 获得文件名
	const char * GetFileName() const;
	// 加载文件
	bool LoadFromFile();
	// 保存文件
	bool SaveToFile() ;
	bool OpenFile() ;
	// 测试是否加载成功
	bool Loaded() const;
	// 加入一行数据
	void AddData(unsigned int index,const string str) ;
private:
	vector<string> m_Data;
	unsigned int buffer_size;
	string m_strFileName;
	bool m_bLoad;
	char * pmode;
	FILE * FP;
};

} /* namespace stcontroller */

#endif /* FILEOPERATOR_H_ */
