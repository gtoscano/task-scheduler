#include "myqueue.h"

#include <vector>


bool MyQueue::is_ready() const {
    if (time_left==0 && counter >0) {
        return true;
    }
    return false;
}
bool MyQueue::push(int value, int my_time) {
    if (time_left == 0 && counter == 0) {
        ++counter;
        time_left  = my_time;
        in_time += my_time;
        queue.push_back(value);
        return true;
    }
    return false;
}

int MyQueue::get_time_left() const {
    if (counter > 0){
        return time_left;
    }
    else {
        return -1;
    }
}


int MyQueue::get_in_time() const {
    return in_time;
}

std::vector<int> MyQueue::get_queue() const {
    return queue;
}


bool MyQueue::elapse_time(int value) {
    if (counter > 0 && value <= time_left) {
        time_left -= value;
        return true;
    }
    return false;
}

int MyQueue::pop() {
    if (counter>0 && time_left ==0) {
        --counter;
        int ret = queue.back();
        return ret;
    }
    return -1;
}
bool MyQueue::is_accepting() const {

    if (counter>0) return false;
    return true;
}

int MyQueue::get_counter() const {
    return counter;
}

