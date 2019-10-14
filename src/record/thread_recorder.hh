#ifndef __THREAD_RECORDER_HH__
#define __THREAD_RECORDER_HH__

#include <fstream>
#include <iostream>

#include "base/types.hh"
#include "debug/ThreadRecord.hh"

class threadRecorder {
public:
    threadRecorder(std::string& _fileName);
    ~threadRecorder();

    static threadRecorder* recorder;
    static void newInstance(std::string _file);
    static threadRecorder* getInstance();

    /* thread id, address, size, current tick */
    void recordMemAccess(uint32_t, Addr, uint64_t, Tick);

    std::ofstream outStream;

private:
    std::string fileName;

};

#endif
