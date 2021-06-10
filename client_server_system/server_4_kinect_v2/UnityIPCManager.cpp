// Copyright 2017 University of Kentucky
// Aly Shehata

#include "UnityIPCManager.h"

UnityIPCManager::UnityIPCManager(string pipeName)
{
	myPipeName = pipeName;
	connect();
}

void UnityIPCManager::connect()
{
	string name = "\\\\.\\pipe\\" + myPipeName;
	wstring temp(name.begin(), name.end());
	LPCWSTR pipename = temp.c_str(); // can be any name, but must start '\\.\pipe\'
	//printf("Create pipe '%s'\r\n", pipename);
	pipeHandle = CreateNamedPipe(pipename,
								 PIPE_ACCESS_DUPLEX,
								 PIPE_TYPE_MESSAGE | PIPE_WAIT,
								 1,		// # instances
								 64,	// out buff
								 0,		// in buff
								 10000, // timeout, 0 = default of 50ms
								 NULL); // security attrs
}

void UnityIPCManager::sendData(const void *buf, size_t size)
{
	LPDWORD numBytesWritten = 0;
	try
	{
		if (WriteFile(pipeHandle, buf, size, numBytesWritten, NULL))
		{
			printf("Data written to pipe\r\n");
		}

		else
		{
			//printf("Pipe write failed attempting reconnect\r\n");
			CloseHandle(pipeHandle);
			connect();
		}
	}
	catch (...)
	{
		//printf("Pipe write failed attempting reconnect\r\n");
		CloseHandle(pipeHandle);
		connect();
	}
}

UnityIPCManager::~UnityIPCManager()
{
	CloseHandle(pipeHandle);
}
