#pragma once


#include "stdafx.h"
#include <vector>
using namespace std;

template <typename T>
T **AllocateDynamicArray(int nRows, int nCols)
{
	T **dynamicArray;

	dynamicArray = new (std::nothrow) T*[nRows];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();

	}

	for (int i = 0; i < nRows; i++)
	{
		dynamicArray[i] = new (std::nothrow) T[nCols];

		if (dynamicArray[i] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}


	}

	return dynamicArray;
}

template <typename T>
void DeallocateDynamicArray(T** dArray, int nRows, int nCols)
{
	if (!dArray)
		return;

	for (int x = 0; x < nRows; x++)
	{
		delete[] dArray[x];
	}

	delete[] dArray;

}

template <typename T>
T ***Allocate3DDynamicArray(int nX, int nY, int nZ)
{
	T ***dynamicArray;

	dynamicArray = new (std::nothrow) T**[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}

	for (int x = 0; x < nX; x++)
	{
		if (x % 1000 ==0)
		{
			cout << "allocating 3D memory for " << x << endl;
		}


		dynamicArray[x] = new (std::nothrow) T*[nY];

		if (dynamicArray[x] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int y = 0; y < nY; y++)
		{
			dynamicArray[x][y] = new (std::nothrow) T[nZ];
			if (dynamicArray[x][y] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}
		}
	}

	for (int x = 0; x < nX; x++)
	for (int y = 0; y < nY; y++)
	for (int z = 0; z < nZ; z++)
	{
		dynamicArray[x][y][z] = 0;
	}
	return dynamicArray;

}

template <typename T>
void Deallocate3DDynamicArray(T*** dArray, int nX, int nY)
{
	if (!dArray)
		return;
	for (int x = 0; x < nX; x++)
	{
		for (int y = 0; y < nY; y++)
		{
			delete[] dArray[x][y];
		}

		delete[] dArray[x];
	}

	delete[] dArray;

}



template <typename T>
T ****Allocate4DDynamicArray(int nM, int nX, int nY, int nZ)
{
	T ****dynamicArray;

	dynamicArray = new (std::nothrow) T***[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}
	for (int m = 0; m < nM; m++)
	{
		if (m%100 ==0)
		cout << "allocating 4D memory for " << m << endl;

		dynamicArray[m] = new (std::nothrow) T**[nX];

		if (dynamicArray[m] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int x = 0; x < nX; x++)
		{
			dynamicArray[m][x] = new (std::nothrow) T*[nY];

			if (dynamicArray[m][x] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}

			for (int y = 0; y < nY; y++)
			{
				dynamicArray[m][x][y] = new (std::nothrow) T[nZ];
				if (dynamicArray[m][x][y] == NULL)
				{
					cout << "Error: insufficient memory.";
					g_ProgramStop();
				}
			}
		}
	}
	return dynamicArray;

}

template <typename T>
void Deallocate4DDynamicArray(T**** dArray, int nM, int nX, int nY)
{
	if (!dArray)
		return;
	for (int m = 0; m < nM; m++)
	{
		for (int x = 0; x < nX; x++)
		{
			for (int y = 0; y < nY; y++)
			{
				delete[] dArray[m][x][y];
			}

			delete[] dArray[m][x];
		}
		delete[] dArray[m];
	}
	delete[] dArray;

}

extern void g_ProgramStop();


template <typename T>
T *****Allocate5DDynamicArray(int nM, int nX, int nY, int nZ, int nW)
{
	T *****dynamicArray;

	dynamicArray = new (std::nothrow) T****[nX];

	if (dynamicArray == NULL)
	{
		cout << "Error: insufficient memory.";
		g_ProgramStop();
	}
	for (int m = 0; m < nM; m++)
	{
		cout << "allocating 5D memory for " << m << endl;

		dynamicArray[m] = new (std::nothrow) T***[nX];

		if (dynamicArray[m] == NULL)
		{
			cout << "Error: insufficient memory.";
			g_ProgramStop();
		}

		for (int x = 0; x < nX; x++)
		{
			dynamicArray[m][x] = new (std::nothrow) T**[nY];

			if (dynamicArray[m][x] == NULL)
			{
				cout << "Error: insufficient memory.";
				g_ProgramStop();
			}

			for (int y = 0; y < nY; y++)
			{
				dynamicArray[m][x][y] = new (std::nothrow) T*[nZ];
				if (dynamicArray[m][x][y] == NULL)
				{
					cout << "Error: insufficient memory.";
					g_ProgramStop();
				}

				for (int w = 0; w < nW; w++)
				{
					dynamicArray[m][x][y][w] = new (std::nothrow) T[nW];
					if (dynamicArray[m][x][y][w] == NULL)
					{
						cout << "Error: insufficient memory.";
						g_ProgramStop();
					}
				}
			}
		}
	}
	return dynamicArray;

}

template <typename T>
void Deallocate5DDynamicArray(T**** dArray, int nM, int nX, int nY, int nW)
{
	if (!dArray)
		return;
	for (int m = 0; m < nM; m++)
	{
		for (int x = 0; x < nX; x++)
		{
			for (int y = 0; y < nY; y++)
			{
				for (int w = 0; w < nW; w++)
				{
					delete[] dArray[m][x][y][w];
				}

				delete[] dArray[m][x][y];


			}

			delete[] dArray[m][x];
		}
		delete[] dArray[m];
	}
	delete[] dArray;

}

std::string CString2StdString(CString str)
{	 // Convert a TCHAR string to a LPCSTR
	CT2CA pszConvertedAnsiString(str);

	// construct a std::string using the LPCSTR input
	std::string strStd(pszConvertedAnsiString);

	return strStd;
}



void g_ProgramStop()
{
	cout << "AgentLite Program stops. Press any key to terminate. Thanks!" << endl;
	getchar();
	exit(0);
};


vector<int> ParseLineToIntegers(string line)
{
	vector<int> SeperatedIntegers;
	string subStr;
	istringstream ss(line);


	char Delimiter = ';';


	while (std::getline(ss, subStr, Delimiter))
	{
		int integer = atoi(subStr.c_str());
		SeperatedIntegers.push_back(integer);
	}
	return SeperatedIntegers;
}

