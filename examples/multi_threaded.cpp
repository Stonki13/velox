#include "velox/velox.h"
#include <cstdio>
#include <thread>
int main(){ velox::World w(velox::BackendType::Cpu); w.setThreadSafetyPolicy(velox::ThreadSafetyPolicy::Relaxed); w.addStaticPlane({0,1,0},0); bool hit=false; std::thread t([&]{hit=w.rayCast({0,2,0},{0,-1,0},4).hit;}); t.join(); std::printf("multi_threaded: %s\n",hit?"PASS":"FAIL"); return hit?0:1; }
