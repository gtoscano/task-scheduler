#ifndef MY_QUEUE_H
#define MY_QUEUE_H

#include <vector>

class MyQueue {
    public:
        MyQueue();
        int pop();
        bool push(int value, int my_time);
        int get_time_left();
        int get_in_time();
        std::vector<int> get_queue();
        bool elapse_time(int value);
        bool is_ready();
        bool is_accepting();
        int get_counter();
    private:
        std::vector<int> queue;
        int counter;
        int time_left;
        int in_time;
};
#endif //MY_QUEUE
