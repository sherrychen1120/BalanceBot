#include "Thread.h"

#include "error.h"

namespace rtos {

Thread::Thread(void (*task)(void const *argument), void *argument,
        osPriority priority, uint32_t stack_size, unsigned char *stack_pointer) {
#ifdef CMSIS_OS_RTX
    _thread_def.pthread = task;
    _thread_def.tpriority = priority;
    _thread_def.stacksize = stack_size;
    if (stack_pointer != NULL) {
        _thread_def.stack_pointer = stack_pointer;
        _dynamic_stack = false;
    } else {
        _thread_def.stack_pointer = new unsigned char[stack_size];
        if (_thread_def.stack_pointer == NULL)
            error("Error allocating the stack memory");
        _dynamic_stack = true;
    }
#endif
    _tid = osThreadCreate(&_thread_def, argument);
}

osStatus Thread::terminate() {
    return osThreadTerminate(_tid);
}

osStatus Thread::set_priority(osPriority priority) {
    return osThreadSetPriority(_tid, priority);
}

osPriority Thread::get_priority() {
    return osThreadGetPriority(_tid);
}

int32_t Thread::signal_set(int32_t signals) {
    return osSignalSet(_tid, signals);
}

osEvent Thread::signal_wait(int32_t signals, uint32_t millisec) {
    return osSignalWait(signals, millisec);
}

osStatus Thread::wait(uint32_t millisec) {
    return osDelay(millisec);
}

osStatus Thread::yield() {
    return osThreadYield();
}

osThreadId Thread::gettid() {
    return osThreadGetId();
}

Thread::~Thread() {
    terminate();
    if (_dynamic_stack) {
        delete[] (_thread_def.stack_pointer);
    }
}

}
