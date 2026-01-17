#pragma once

#include <array>
#include <string>
#include <variant>
#include <fstream>

template <size_t N>
class CSVLogger
{
public:

	typedef std::variant<long, double, std::string> CSVLoggerData;
	enum VarientIndex
	{
		INTEGER = 0,
		FLOATING = 1,
		STRING = 2
	};

	CSVLogger() = delete;
	CSVLogger(std::array<std::string, N> columns, std::string path);

	void logData(std::array<CSVLoggerData, N> data);
	void flush();

private:
	const size_t column_count = N;
	const std::array<std::string, N> columns;
	const std::string path;
	const size_t last = N - 1;//So we only need to calculate it once
	std::ofstream file;
};

template <size_t N>
CSVLogger<N>::CSVLogger(std::array<std::string, N> columns, std::string path) :
	columns{ columns },
	path{ path },
	file{ path }
{
	for(size_t i = 0; i < N; i++)
	{
		file << columns[i];
		if(i != last)
		{
			file << ",";
		}
	}
	file << std::endl;
}

template <size_t N>
void CSVLogger<N>::logData(std::array<CSVLoggerData, N> data)
{
	for(size_t i = 0; i < N; i++)
	{
		switch(data[i].index())
		{
			case INTEGER:
			{
				file << std::get<long>(data[i]);
			}break;
			case FLOATING:
			{
				file << std::get<double>(data[i]);
			}break;
			case STRING:
			{
				file << std::get<std::string>(data[i]);
			}break;
		}
		if(i != last)
		{
			file << ",";
		}
	}
	file << std::endl;
}

template <size_t N>
void CSVLogger<N>::flush()
{
	file.flush();
}
