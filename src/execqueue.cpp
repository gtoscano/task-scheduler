#include "execqueue.h"
#include <algorithm>
#include <iostream>
#include <vector>

ExecQueue::ExecQueue(int n) {
    for (int i=0; i<n; ++i) {
        queues.emplace_back();
    }
    total_time = 0;

}

bool ExecQueue::is_full() const {
    return std::ranges::all_of(queues, [](auto& queue) { return queue.is_accepting(); });
}

/*
bool ExecQueue::is_full() {
    for (auto& queue : queues) {
        if (!queue.is_accepting()) {
            return false;
        }
    }
    return true;
}*/

bool ExecQueue::is_empty() const {
    int sum = 0;
    for (auto& queue : queues) {
        sum += queue.get_counter();
    }
    if(sum == 0) {
        return true;
    }

    return false;
}

int ExecQueue::count_open_queues() const {
    int counter = 0;
    for (auto& queue : queues) {
        if (queue.is_accepting()) {
            ++counter;
        }
    }
    return counter;
}

bool ExecQueue::enqueue(int value, int my_time) {
    for (auto& queue : queues) {
        if (queue.is_accepting()) {
            queue.push(value, my_time);
            return true;
        }
    }
    return false;
}

int ExecQueue::get_next() const {
    auto min_value = std::numeric_limits<int>::max(); 
    for (auto& queue : queues) {
        auto res = queue.get_time_left();
        if (res > -1 && res < min_value) {
            min_value = res;
        }
    }
    return min_value;
}
bool ExecQueue::elapse_time() {

    if (is_empty()){ 
        return false;
    }
    auto value = get_next();
    total_time += value;
    for (auto& queue : queues) {
        queue.elapse_time(value);
    }
    return true;
}

std::vector<int> ExecQueue::dequeue() {
    std::vector<int> ret;
    elapse_time();

    for (auto& queue : queues) {
        if (queue.is_ready()) {
            auto value = queue.pop();
            ret.push_back(value);
        }
    }
    return ret;
}

int ExecQueue::get_total_time() const {
    return total_time;

}

std::vector<int> ExecQueue::get_in_time() const {
    std::vector<int> ret;
    for (auto& queue : queues) {
        auto value = queue.get_in_time();
        ret.emplace_back(value);
    }
    return ret;
}
std::vector<std::vector<int>> ExecQueue::get_queues() const {
    std::vector<std::vector<int>> ret;
    for (auto& queue : queues) {
        auto value = queue.get_queue();
        ret.emplace_back(value);
    }
    return ret;
}
void ExecQueue::print() const {
    int counter = 0;
    std::cout<<"Makespan: "<<total_time<<"\n";
    for (auto& queue : queues) {
        std::cout<<"Processor "<<counter<<" (makespan="<<queue.get_in_time()<<"): ";
        for(const auto& q: queue.get_queue()) {
            std::cout<<q<<" -> ";
        }
        std::cout<<"\n";
        ++counter;
    }
}

