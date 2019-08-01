#include "thread_recorder.hh"

threadRecorder* threadRecorder::recorder = NULL;

threadRecorder::threadRecorder(std::string& _fileName)
{
    fileName = _fileName;
    outStream.open(_fileName, std::ios::out);
    if (!outStream.is_open()) {
        assert(false);
    }
    outStream << "tick,";
    outStream << "id,";
    outStream << "addr,";
    outStream << "size";
    outStream << "\n";
}

threadRecorder::~threadRecorder()
{
    outStream.close();
}

void
threadRecorder::newInstance(std::string _file)
{
    recorder = new threadRecorder(_file);
}

threadRecorder*
threadRecorder::getInstance()
{
    return recorder;
}

void
threadRecorder::recordMemAccess(
    uint32_t _id,
    Addr _addr,
    uint64_t _size,
    Tick _tick)
{
    outStream << _tick << ",";
    outStream << _id << ",";
    outStream << _addr << ",";
    outStream << _size << "\n";
}

