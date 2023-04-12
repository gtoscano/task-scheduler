#ifndef EXEC_QUEUE_H
#define EXEC_QUEUE_H

#include "myqueue.h"
#include <vector>

class ExecQueue {

    public:
        ExecQueue(int n);
        bool is_full();
        bool is_empty();
        bool enqueue(int value, int my_time);
        std::vector<int> dequeue();
        int get_next();
        bool elapse_time();
        int count_open_queues();
        int get_total_time();
        std::vector<int>  get_in_time();
        std::vector<std::vector<int>> get_queues();
        void print();
    private:
        std::vector<MyQueue> queues;
        int total_time;
};

#endif //EXEC_QUEUE
