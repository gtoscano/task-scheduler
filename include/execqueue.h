#ifndef EXEC_QUEUE_H
#define EXEC_QUEUE_H

#include "myqueue.h"
#include <vector>

class ExecQueue {

    public:
        explicit ExecQueue(int n);
        bool is_full() const;
        bool is_empty() const;
        bool enqueue(int value, int my_time);
        std::vector<int> dequeue();
        int get_next() const;
        bool elapse_time();
        int count_open_queues() const;
        [[nodiscard]] int get_total_time() const;
        std::vector<int>  get_in_time() const;
        std::vector<std::vector<int>> get_queues() const;
        void print() const;
    private:
        std::vector<MyQueue> queues;
        int total_time;
};

#endif //EXEC_QUEUE_H
