#ifndef MY_QUEUE_H
#define MY_QUEUE_H

#include <vector>

class MyQueue {
    public:
        MyQueue() = default;
        [[nodiscard]] int pop();
        bool push(int value, int my_time);
        [[nodiscard]] int get_time_left() const;
        [[nodiscard]] int get_in_time() const;
        [[nodiscard]] std::vector<int> get_queue() const;
        bool elapse_time(int value);
        [[nodiscard]] bool is_ready() const;
        [[nodiscard]] bool is_accepting() const;
        [[nodiscard]] int get_counter() const;
    private:
        std::vector<int> queue;
        int counter = 0;
        int time_left = 0;
        int in_time = 0;
};
#endif //MY_QUEUE_H
