// Copyright 2017 University of Kentucky 
// Aly Shehata

#pragma once
#include <stdio.h>
#include <stdint.h>
#include <winsock.h>
#include <string>

using namespace std;
class UnityIPCManager
{
public:
	UnityIPCManager(string pipeName);
	void connect();
	void sendData(const void *buf, size_t size);
	~UnityIPCManager();

private:
	string myPipeName;
	HANDLE pipeHandle;
};

