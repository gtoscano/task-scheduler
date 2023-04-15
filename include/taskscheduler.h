//
// Created by Gregorio Toscano on 4/14/23.
//

#ifndef TASKSCHEDULER_H
#define TASKSCHEDULER_H

#include "execqueue.h"

#include <nlohmann/json.hpp>

#include <queue>
#include <list>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

class TaskScheduler {
public:
    explicit TaskScheduler(const std::string &config_file);

    int find_best_num_cpus(int min_cpus, int max_cpus);
    int find_makespan(int min_cpus, int max_cpus, int desired_makespan);
    ExecQueue lpt_algorithm(int m);
private:
    std::unordered_map<int, int> tasks_;
    std::vector<std::pair<int, int>> dependencies_;
    std::unordered_map<int, std::list<int>> dag_;
    std::vector<int> roots_;
    std::unordered_map<int, std::list<int>> rev_dag_;
    std::vector<int> rev_roots_;
    std::unordered_map<int, int> costs_;


    static std::unordered_map<int, int> read_tasks(const nlohmann::json &json_data);
    static std::vector<std::pair<int, int>> read_dependencies(const nlohmann::json &json_data);
    void build_dag();
    static std::vector<int> find_genesis_nodes(const std::unordered_map<int, std::list<int>> &dag);
    static std::unordered_map<int, std::list<int>> reverse_dag(const std::unordered_map<int, std::list<int>> &dag);
    std::unordered_map<int, int> compute_cost();
    void dequeue(std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>> &tasks_queue, std::set<int> &already_executed, int node);

    bool load_config_file(const std::string &config_file);

    void print_priority(std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>> q) const;
};
#endif //TASKSCHEDULER_H
