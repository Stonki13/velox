#include <iostream>
#include <cstdint>
#include "velox/arena.h"

using namespace velox;

int main() {
    ArenaAllocator arena(4096);
    
    // Burn 1 byte to misalign the bump pointer.
    arena.allocate(1, 1);
    
    for (size_t alignment : {1u, 2u, 4u, 8u, 16u, 32u, 64u, 128u}) {
        void* ptr = arena.allocate(16, alignment);
        std::cout << "Alignment " << alignment << ": ptr=" << ptr 
                  << " ptr%" << alignment << "=" << (reinterpret_cast<uintptr_t>(ptr) % alignment) << std::endl;
    }
    
    return 0;
}
